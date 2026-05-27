#!/usr/bin/env python3
"""
seabed_estimator — Node 1 of the Dirichlet voxel mapping pipeline.
===============================================================================

Maintains a single scalar Kalman filter that tracks, at scan rate, the typical
seabed backscatter intensity ``mu_I`` (with variance var_I) and publishes the
state on ``state_topic`` once per scan.

The depth half of the estimator was removed: with elevation-band evidence
spreading in voxel_mapper, the global ``mu_h`` is no longer used anywhere
downstream — bathymetry emerges directly from the per-voxel Bayesian
aggregation. Only the intensity track remains.

mu_I is essential — return_classifier derives the single SEABED vs
(STRUCTURE/OBJECT) intensity threshold ``tau_str_obj = lambda_str_obj * mu_I``
from the live estimate.

PER-SCAN MEASUREMENT
  z_I = median of the non-zero return intensities, after gating out values
        more than ``intensity_outlier_sigma`` standard deviations above mu_I.

PROCESS NOISE  (mission-scale prior)
  A random walk whose total drift over a fixed scan horizon equals the
  expected peak-to-peak variation:
      q_I = (seabed_intensity_variation / sqrt(_N_SCANS_FOR_DRIFT)) ** 2
  Only the variation is user-tunable from the YAML; the scan horizon is a
  module-level constant (the mission's true scan count is usually unknown).

MEASUREMENT NOISE  (adaptive)
  A sensor-intrinsic floor plus a coupling coefficient times the within-scan
  MAD-based dispersion of the measured quantity:
      sigma_meas = floor + coupling * MAD
      R          = sigma_meas ** 2
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
import sensor_msgs_py.point_cloud2 as pc2
import numpy.lib.recfunctions as rfn

from sonar_map_msgs.msg import SeabedEstimatorState


# ── Defaults — overridable from the YAML config / launch arguments ────────────
INPUT_TOPIC                = 'sonar_scan'
STATE_TOPIC                = 'seabed_estimator_state'
MU_INTENSITY_TOPIC         = 'seabed_mu_intensity'   # std_msgs/Float64 — for plotting

INTENSITY_OUTLIER_SIGMA    = 2.0      # [-]  gate width for z_I outlier removal
MIN_RETURNS                = 10       # min returns in a scan to run an update

I_PRIOR_MEAN               = 530.0    # [intensity]
I_PRIOR_STD                = 300.0    # [intensity]

SEABED_INTENSITY_VARIATION = 400.0    # [intensity]  mission peak-to-peak intensity

# Random-walk horizon used to derive the per-scan process-noise std from the
# mission-scale variation: per-scan std = variation / sqrt(_N_SCANS_FOR_DRIFT).
# Kept as a module-level constant rather than a ROS parameter — the mission's
# true scan count is usually unknown, and tuning the estimator via the
# variation alone (with a fixed horizon) is the more practical knob.
_N_SCANS_FOR_DRIFT = 2000.0

I_MEAS_NOISE_FLOOR         = 50.0     # [intensity]  sensor-intrinsic intensity floor
I_MEAS_NOISE_COUPLING      = 1.0      # [-]          MAD coupling for intensity


def _read_xyz_intensity(msg: PointCloud2) -> np.ndarray:
    """Return an (N, 4) float32 array of [x, y, z, intensity]."""
    raw = pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                          skip_nans=True)
    if isinstance(raw, np.ndarray):
        if raw.size == 0:
            return np.empty((0, 4), dtype=np.float32)
        if raw.dtype.names is not None:
            return rfn.structured_to_unstructured(raw, dtype=np.float32)
        return raw.reshape(-1, 4).astype(np.float32)
    rows = list(raw)
    if not rows:
        return np.empty((0, 4), dtype=np.float32)
    return np.array(rows, dtype=np.float32).reshape(-1, 4)


def _mad(x: np.ndarray) -> float:
    """Median absolute deviation — a robust, outlier-resistant dispersion."""
    if x.size == 0:
        return 0.0
    med = np.median(x)
    return float(np.median(np.abs(x - med)))


class ScalarKalman:
    """A one-dimensional Kalman filter with an explicit predict/update split."""

    def __init__(self, mean: float, std: float, process_var: float):
        self.mu = float(mean)
        self.var = float(std) ** 2
        self.q = float(process_var)        # already a variance

    def predict(self):
        self.var += self.q

    def update(self, z_meas: float, r_meas: float):
        """Correct the state with a measurement of variance ``r_meas``."""
        k = self.var / (self.var + r_meas)
        self.mu += k * (z_meas - self.mu)
        self.var = (1.0 - k) * self.var


class SeabedEstimatorNode(Node):

    def __init__(self):
        super().__init__('seabed_estimator')

        self.declare_parameter('input_topic',                INPUT_TOPIC)
        self.declare_parameter('state_topic',                STATE_TOPIC)
        self.declare_parameter('mu_intensity_topic',         MU_INTENSITY_TOPIC)
        self.declare_parameter('intensity_outlier_sigma',    INTENSITY_OUTLIER_SIGMA)
        self.declare_parameter('min_returns',                MIN_RETURNS)
        self.declare_parameter('i_prior_mean',               I_PRIOR_MEAN)
        self.declare_parameter('i_prior_std',                I_PRIOR_STD)
        self.declare_parameter('seabed_intensity_variation', SEABED_INTENSITY_VARIATION)
        self.declare_parameter('i_meas_noise_floor',         I_MEAS_NOISE_FLOOR)
        self.declare_parameter('i_meas_noise_coupling',      I_MEAS_NOISE_COUPLING)

        def gp(n):
            return self.get_parameter(n).value

        self.outlier_sig_ = float(gp('intensity_outlier_sigma'))
        self.min_returns_ = max(1, int(gp('min_returns')))
        self.i_floor_     = float(gp('i_meas_noise_floor'))
        self.i_coupling_  = float(gp('i_meas_noise_coupling'))

        # Process noise from mission-scale prior: a random walk whose total
        # drift over _N_SCANS_FOR_DRIFT scans equals the expected variation.
        q_i = (float(gp('seabed_intensity_variation')) / np.sqrt(_N_SCANS_FOR_DRIFT)) ** 2

        self.kf_i_ = ScalarKalman(gp('i_prior_mean'), gp('i_prior_std'), q_i)

        self.pub_ = self.create_publisher(
            SeabedEstimatorState, gp('state_topic'), 10)
        # Scalar mirror of mu_i — convenient for Plotjuggler / rqt_plot /
        # Foxglove without configuring a field accessor.
        self.mu_pub_ = self.create_publisher(
            Float64, gp('mu_intensity_topic'), 10)
        self.sub_ = self.create_subscription(
            PointCloud2, gp('input_topic'), self._scan_cb, 1)

        self._scan_count_ = 0
        self.get_logger().info(
            f'seabed_estimator: {gp("input_topic")} -> {gp("state_topic")} | '
            f'I_prior={self.kf_i_.mu:.0f} +/-{np.sqrt(self.kf_i_.var):.0f} | '
            f'q_I={q_i:.2e}')

    def _scan_cb(self, msg: PointCloud2):
        try:
            pts = _read_xyz_intensity(msg)
        except Exception as exc:  # noqa: BLE001 — never let a bad scan kill the node
            self.get_logger().error(f'Failed to read PointCloud2: {exc}',
                                    throttle_duration_sec=5.0)
            return

        pts = pts[np.isfinite(pts).all(axis=1)] if pts.size else pts

        # ── Prediction: the seabed intensity may drift between scans ──────────
        self.kf_i_.predict()

        updated_i = False
        if pts.shape[0] >= self.min_returns_:
            intens = pts[:, 3].astype(np.float64)

            # ── Intensity measurement: median of gated non-zero returns ───────
            nz = intens[intens > 0.0]
            if nz.size > 0:
                gate = self.kf_i_.mu + self.outlier_sig_ * np.sqrt(self.kf_i_.var)
                kept = nz[nz <= gate]
                if kept.size == 0:           # gate removed everything — fall back
                    kept = nz
                z_i     = float(np.median(kept))
                sigma_i = self.i_floor_ + self.i_coupling_ * _mad(kept)
                self.kf_i_.update(z_i, max(sigma_i, 1e-6) ** 2)
                updated_i = True

        self._publish(msg, updated_i)

    def _publish(self, src_msg: PointCloud2, did_update: bool):
        out = SeabedEstimatorState()
        out.header.stamp = src_msg.header.stamp
        out.header.frame_id = src_msg.header.frame_id
        out.mu_i  = float(self.kf_i_.mu)
        out.var_i = float(self.kf_i_.var)
        self.pub_.publish(out)
        # Plot-friendly scalar mirror of mu_i.
        self.mu_pub_.publish(Float64(data=float(self.kf_i_.mu)))

        self._scan_count_ += 1
        if self._scan_count_ <= 5 or self._scan_count_ % 50 == 0:
            self.get_logger().info(
                f'scan #{self._scan_count_} '
                f'[{"updated" if did_update else "predict-only"}] | '
                f'I={out.mu_i:.0f} +/-{np.sqrt(out.var_i):.0f}')


def main(args=None):
    rclpy.init(args=args)
    node = SeabedEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
