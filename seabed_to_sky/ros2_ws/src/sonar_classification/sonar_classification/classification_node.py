#!/usr/bin/env python3
"""
classification_node — stages 4-6 of the per-sweep algorithm: score, accumulate,
commit.

    4. SCORING      evaluate log L_{i,c} for every class (eq 34), read the
                    spatial prediction pi_c(N_v) (eq 16) from the FROZEN (t-1)
                    snapshot, and form the responsibilities (eq 9).
    5. ACCUMULATION add r_i to the Dirichlet concentrations of the hit voxel
                    (eq 13), with one designated carrier per region and voxel
                    for the morphology descriptor.
    6. COMMIT       refresh the neighbourhood snapshot and report MAP labels
                    (eq 15).

I6 — ORDER INDEPENDENCE. Every responsibility in a sweep is computed against
the same frozen snapshot and the same fixed class parameters BEFORE any of them
is accumulated, so the map cannot depend on the order in which returns are
processed. This is not a stylistic choice: reading live neighbour beliefs
mid-sweep would make the result a function of the message's point ordering.

I4 — NO LEARNING. Class parameters are read once from classes.yaml and never
mutated. The only online accumulation anywhere in this system is the Dirichlet
over voxels.

THE DEDUP GATE d_i is computed here and nowhere else, because this is the only
place that knows which voxel a return landed in. Within a region the returns
share one descriptor: the region is ONE observation about each voxel it covers,
not N_R observations, so d_i marks the first return of each (region, voxel)
pair in the sweep and only that return carries the shape evidence.

TOPICS
  ``semantic_voxels`` (PointCloud2) — the live map: voxel centre, MAP label id,
      display confidence, accumulated evidence M_v, and one ``theta_<class>``
      field per class carrying the full posterior mean (eq 14). One float32
      field per class means the whole belief is inspectable in Foxglove without
      a custom message.
  ``class_census`` (Float32MultiArray, debug) — the same per-class voxel counts
      that go to the CSV, published live for plotting while the run is going.

DEBUG CLASS CENSUS. With ``debug_census`` enabled the node counts, every
``census_period_sec`` seconds, how many voxels each class currently owns and
appends a row to a CSV. ``ros2 run sonar_classification plot_class_census``
turns that file into one plot with one curve per class.
"""

import os

import numpy as np
import numpy.lib.recfunctions as rfn

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Header
import sensor_msgs_py.point_cloud2 as pc2
from ament_index_python.packages import get_package_share_directory

from sonar_classification.argus import likelihood
from sonar_classification.argus.census import ClassCensusLog
from sonar_classification.argus.classes import FACTOR_NAMES, load_classes
from sonar_classification.argus.voxel_map import VoxelMap

#: Fields read from ``semantic_returns`` (measurement_node's OUT_FIELDS).
IN_FIELDS = ('x', 'y', 'z', 'count_raw', 'h', 'rho_var',
             'lidar_observed', 'lidar_present', 'clearance',
             'shape_flag', 'extent', 'thickness', 'region_id')


def _read_fields(msg: PointCloud2, names) -> np.ndarray:
    """(N, len(names)) float32 read. skip_nans OFF — NaN is the abstention marker."""
    raw = pc2.read_points(msg, field_names=tuple(names), skip_nans=False)
    if isinstance(raw, np.ndarray):
        if raw.size == 0:
            return np.empty((0, len(names)), dtype=np.float32)
        if raw.dtype.names is not None:
            return rfn.structured_to_unstructured(raw, dtype=np.float32)
        return raw.reshape(-1, len(names)).astype(np.float32)
    rows = list(raw)
    if not rows:
        return np.empty((0, len(names)), dtype=np.float32)
    return np.array(rows, dtype=np.float32).reshape(-1, len(names))


class ClassificationNode(Node):

    def __init__(self):
        super().__init__('classification_node')

        self.declare_parameter('input_topic', 'semantic_returns')
        self.declare_parameter('voxel_topic', 'semantic_voxels')
        self.declare_parameter('census_topic', 'class_census')
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('classes_file', '')

        self.declare_parameter('voxel_size', 0.20)
        self.declare_parameter('alpha_0', 0.01)
        self.declare_parameter('eps', 0.5)

        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('w_ref', 5.0)
        self.declare_parameter('w_display_min', 1.0)
        self.declare_parameter('log_every', 50)

        # ── debug: per-class voxel census ────────────────────────────────────
        self.declare_parameter('debug_census', False)
        self.declare_parameter('census_period_sec', 10.0)
        self.declare_parameter('census_csv_path',
                               '~/ros2_ws/class_census.csv')
        self.declare_parameter('census_min_evidence', 1.0)

        def gp(n):
            return self.get_parameter(n).value

        classes_file = str(gp('classes_file')).strip()
        if not classes_file:
            classes_file = os.path.join(
                get_package_share_directory('sonar_classification'),
                'config', 'classes.yaml')
        self._cs = load_classes(classes_file)
        self._names = list(self._cs.names)

        # ``ablation.spatial_prior: false`` forces the uniform prediction
        # regardless of eps, so the study isolates that one behaviour.
        eps = float(gp('eps'))
        if not self._cs.ablation.get('spatial_prior', True):
            eps = 0.0
        self._map = VoxelMap(n_classes=self._cs.n_classes,
                             resolution=float(gp('voxel_size')),
                             alpha_0=float(gp('alpha_0')),
                             eps=eps)

        self._map_frame = str(gp('map_frame'))
        self._w_ref = max(float(gp('w_ref')), 1e-6)
        self._w_disp_min = float(gp('w_display_min'))
        self._log_every = max(int(gp('log_every')), 1)

        voxel_field_names = (['x', 'y', 'z', 'label', 'confidence', 'evidence']
                             + [f'theta_{n}' for n in self._names])
        self._voxel_layout = [
            PointField(name=nm, offset=4 * i,
                       datatype=PointField.FLOAT32, count=1)
            for i, nm in enumerate(voxel_field_names)
        ]

        self._voxel_pub = self.create_publisher(
            PointCloud2, str(gp('voxel_topic')), 10)
        self.create_subscription(
            PointCloud2, str(gp('input_topic')), self._sweep_cb, 10)
        self.create_timer(1.0 / max(float(gp('publish_rate')), 1e-3),
                          self._publish_voxels)

        # ── debug census wiring ──────────────────────────────────────────────
        self._census_log = None
        self._census_pub = None
        self._census_min_ev = float(gp('census_min_evidence'))
        self._census_t0 = None
        if bool(gp('debug_census')):
            period = max(float(gp('census_period_sec')), 0.1)
            self._census_log = ClassCensusLog(
                path=str(gp('census_csv_path')),
                class_names=self._names,
                resolution=self._map.resolution,
                min_evidence=self._census_min_ev,
                period=period)
            self._census_pub = self.create_publisher(
                Float32MultiArray, str(gp('census_topic')), 10)
            self.create_timer(period, self._census_tick)
            self.get_logger().info(
                f'DEBUG census ON: every {period}s -> {self._census_log.path} '
                f'(one row per tick, one column per class; voxels with '
                f'M_v < {self._census_min_ev} are not counted). Plot it with: '
                f'ros2 run sonar_classification plot_class_census '
                f'{self._census_log.path}')

        self._sweeps = 0
        enabled = [f for f in FACTOR_NAMES if self._cs.factors_enabled.get(f, True)]
        self.get_logger().info(
            f'classification_node: {gp("input_topic")} -> {gp("voxel_topic")} | '
            f'classes={self._names} ({classes_file}) | factors={enabled} | '
            f'voxel={self._map.resolution}m alpha_0={self._map.alpha_0} | '
            + (f'spatial prediction ON eps={self._map.eps}m '
               f'({len(self._map._offsets)} neighbour voxels)'
               if self._map.use_prior else
               'spatial prediction OFF (uniform pi, vectorised fast path)'))

    # ── the dedup gate d_i ───────────────────────────────────────────────────

    def _dedup_gate(self, region_id: np.ndarray, keys: np.ndarray) -> np.ndarray:
        """d_i: 1 for the FIRST return of each (region, voxel) pair this sweep.

        Returns not in a region get 0 — they carry no descriptor anyway, and
        the morphology factor's absent branch does not depend on d_i.
        """
        d = np.zeros(region_id.shape[0], dtype=np.float64)
        member = np.isfinite(region_id) & (region_id >= 0.0)
        if not np.any(member):
            return d
        idx = np.flatnonzero(member)
        pairs = np.stack([region_id[idx].astype(np.int64), keys[idx]], axis=1)
        # np.unique's return_index gives the first occurrence of each pair.
        _, first = np.unique(pairs, axis=0, return_index=True)
        d[idx[first]] = 1.0
        return d

    # ── stages 4-6 ───────────────────────────────────────────────────────────

    def _sweep_cb(self, msg: PointCloud2):
        present = {f.name for f in msg.fields}
        missing = [nm for nm in IN_FIELDS if nm not in present]
        if missing:
            self.get_logger().error(
                f'Input cloud is missing {missing} — is it coming from '
                'measurement_node?', throttle_duration_sec=10.0)
            return

        pts = _read_fields(msg, IN_FIELDS)
        if pts.shape[0] == 0:
            return
        xyz_ok = np.isfinite(pts[:, :3]).all(axis=1)
        pts = pts[xyz_ok]
        if pts.shape[0] == 0:
            return

        x, y, z = (pts[:, 0].astype(np.float64),
                   pts[:, 1].astype(np.float64),
                   pts[:, 2].astype(np.float64))
        keys = self._map.keys_of(x, y, z)
        region_id = pts[:, 12].astype(np.float64)

        m = likelihood.Measurements(
            h=pts[:, 4].astype(np.float64),
            rho_var=pts[:, 5].astype(np.float64),
            count_raw=pts[:, 3].astype(np.float64),
            observed=pts[:, 6].astype(np.float64),
            present=pts[:, 7].astype(np.float64),
            clearance=pts[:, 8].astype(np.float64),
            shape_flag=pts[:, 9].astype(np.float64),
            extent=pts[:, 10].astype(np.float64),
            thickness=pts[:, 11].astype(np.float64),
            dedup=self._dedup_gate(region_id, keys),
            n_region=None,
        )

        # ── 4. scoring — everything against the frozen (t-1) state ───────────
        log_L, by_factor = likelihood.log_likelihood(m, self._cs)
        pi = self._map.spatial_prediction(keys)
        r = likelihood.responsibilities(log_L, np.log(pi))

        # ── 5. accumulation — only after ALL responsibilities exist (I6) ─────
        self._map.accumulate(keys, r)

        # ── 6. commit — refresh the snapshot the next sweep will read ────────
        self._map.commit()

        self._sweeps += 1
        if self._sweeps <= 5 or self._sweeps % self._log_every == 0:
            self._log_sweep(len(pts), r, by_factor)

    def _log_sweep(self, n, r, by_factor):
        """Which factor is pulling which class — the tuning readout.

        Per-factor MEAN log-likelihood per class, alongside the mean
        responsibility share. Because the responsibility is a ratio, only the
        SPREAD of a factor's row across classes matters; a factor whose values
        barely differ between two classes cannot separate them, however large
        its absolute numbers are.
        """
        share = ' '.join(f'{nm}={s:.2f}'
                         for nm, s in zip(self._names, r.mean(axis=0)))
        self.get_logger().info(
            f'sweep #{self._sweeps}: {n} returns | map={len(self._map)} voxels '
            f'| mean responsibility: {share}')
        for f in FACTOR_NAMES:
            if not self._cs.factors_enabled.get(f, True):
                continue
            row = by_factor[f].mean(axis=0)
            txt = ' '.join(f'{nm}={v:+.2f}' for nm, v in zip(self._names, row))
            self.get_logger().info(f'    log {f:<9}: {txt}')

    # ── debug: the per-class voxel census ────────────────────────────────────

    def _census_tick(self):
        counts, mass, n_vox = self._map.census(self._census_min_ev)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._census_t0 is None:
            self._census_t0 = now
        t = now - self._census_t0

        self._census_log.write(t_sec=t, sweeps=self._sweeps,
                               n_voxels=n_vox, counts=counts, mass=mass)

        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = ','.join(self._names)
        dim.size = len(self._names)
        dim.stride = len(self._names)
        msg.layout.dim = [dim]
        msg.data = [float(c) for c in counts]
        self._census_pub.publish(msg)

        top = ' '.join(f'{nm}={int(c)}' for nm, c in zip(self._names, counts))
        self.get_logger().info(
            f'census t={t:6.1f}s sweeps={self._sweeps} voxels={n_vox} | {top}')

    # ── the live map ─────────────────────────────────────────────────────────

    def _publish_voxels(self):
        if len(self._map) == 0:
            return
        keys, m, M = self._map.snapshot_arrays()
        theta = self._map.posterior_mean(m, M)                  # eq 14
        label = np.argmax(m, axis=1)                            # eq 15
        p_max = theta[np.arange(theta.shape[0]), label]

        # Display confidence only: how peaked the posterior is, discounted by
        # how little evidence it rests on. Nothing here feeds back into the map.
        conf = p_max * np.tanh(M / self._w_ref)

        keep = M >= self._w_disp_min
        if not np.any(keep):
            return
        centres = self._map.centres(keys[keep])

        out = np.empty((int(keep.sum()), 6 + self._cs.n_classes),
                       dtype=np.float32)
        out[:, 0:3] = centres
        out[:, 3] = label[keep]
        out[:, 4] = conf[keep]
        out[:, 5] = M[keep]
        out[:, 6:] = theta[keep]

        header = Header()
        header.frame_id = self._map_frame
        header.stamp = self.get_clock().now().to_msg()
        self._voxel_pub.publish(
            pc2.create_cloud(header, self._voxel_layout, out))


def main(args=None):
    rclpy.init(args=args)
    node = ClassificationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
