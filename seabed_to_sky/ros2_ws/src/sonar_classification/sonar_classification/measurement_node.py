#!/usr/bin/env python3
"""
measurement_node — stages 2-3 of the per-sweep algorithm: build the measurement
tuple y_i = (p_i, I_i, l_i, m_i) for every retained leading-edge return.

    y_i = ( p_i,        position in the map frame
            I_i,        radiometrically compensated backscatter [dB]  (eq 1)
            l_i,        (o_i, s_lidar_i, g_tilde_i) overhead support  (eq 27)
            m_i )       (s^m_i, e_R, w_R) region morphology          (eq 31)

Stage 1 (detection: conditioning, leading-edge extraction, RANSAC + rolling-MAD
robustification, region growing) runs UPSTREAM in the leading-edge extractor and
is deliberately not duplicated here — that is the detection path, and this
package owns the measurement and inference paths.

I1 — TWO-PATH SEPARATION. Detection runs on a conditioned copy of the sonar
image; every MEASURED value is read from the pristine image at the detected
coordinates. The intensity arriving on the input cloud is the raw count at the
detected cell, and this node compensates it (eq 1) without ever consulting the
conditioned image. No adaptive amplitude processing happens anywhere on this
path (I3): the compensation is a fixed function of (range, raw count) only.

WHAT THIS NODE ADDS, CHANNEL BY CHANNEL

  * POSITION. The slant range r_i from the sonar to the return gives the
    elevation-induced height variance rho_var_i (eq 22); a rolling local bed fit
    (BottomGrid) gives z_bed_hat(x_i, y_i) and hence h_i (eq 18). h_i is NaN
    until the bed reference has warmed up there, and the position factor
    abstains rather than guessing.
  * INTENSITY. I_i via eq 1 from the raw count and r_i. NaN below the noise
    floor; the saturation flag rides along for diagnosis.
  * LIDAR. The overhead column (eq 26) around the return's own XY, yielding
    coverage o_i, the k_min-VALIDATED presence bit s_lidar_i, and the clearance
    g_tilde_i measured FROM THE WATERLINE (not from the return — see
    factor_lidar). The count itself is deliberately not forwarded: it is a
    property of the trajectory and must not reach the likelihood (I8).
  * MORPHOLOGY. The region's beam-corrected bearing extent e_R and range
    thickness w_R, plus a region id for the downstream dedup gate (see
    region_descriptor for the two source modes).

WHY THE QUERIES ARE XY-ONLY. The sonar images the seabed from underwater while
the LiDAR scans from the surface, so a submerged structure return sits directly
BELOW its above-water LiDAR return — same x, y, separated by the large waterline
gap in z. A 3-D match would fold that gap into every distance and kill all
support even directly under a quay. The vertical column above the return is the
signal that actually discriminates "under structure" from "open bed".

OUTPUT is one PointCloud2 per sweep on ``semantic_returns``, carrying the tuple
as named float32 fields — self-describing, inspectable in Foxglove/RViz, and
requiring no custom message package.
"""

import os

import numpy as np
import numpy.lib.recfunctions as rfn

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

from sonar_classification.argus import geometry, region_descriptor
from sonar_classification.argus.bottom_grid import BottomGrid
from sonar_classification.argus.classes import load_classes

#: Wire layout of ``semantic_returns``. The classification node reads these by
#: name, so the order here is documentation, not a contract.
OUT_FIELDS = (
    'x', 'y', 'z',
    'slant_range',      # r_i [m]
    'count_raw',        # A_i, the pristine uint16 amplitude
    'saturated',        # 1 = count at uint16 full scale (possible clipping)
    'h',                # h_i (eq 18), NaN where the bed is unknown
    'rho_var',          # rho_var_i (eq 22)
    'lidar_observed',   # o_i, NaN before any LiDAR cloud
    'lidar_present',    # s_lidar_i (k_min validated)
    'clearance',        # g_tilde_i [m], waterline referenced
    'shape_flag',       # s^m_i, NaN when no extractor ran
    'extent',           # e_R [m], beam-width corrected
    'thickness',        # w_R [m]
    'region_id',        # per-sweep region identity for the dedup gate
)

#: Legacy descriptor channels this node can derive (e_R, w_R) from.
LEGACY_SHAPE_FIELDS = ('shape_flag', 'region_size', 'region_logarea')
#: Fields the extractor publishes once it reports the two lengths directly.
NATIVE_SHAPE_FIELDS = ('shape_flag', 'region_extent', 'region_thickness')


def _read_fields(msg: PointCloud2, names) -> np.ndarray:
    """Read the named fields into an (N, len(names)) float32 array.

    ``skip_nans`` stays OFF: NaN is DATA in the descriptor channels (it is the
    hurdle's abstention marker), and a whole-row NaN filter would silently drop
    exactly the returns the morphology factor needs to see. Row survival is
    gated on the geometric core by the caller instead.
    """
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


class MeasurementNode(Node):

    def __init__(self):
        super().__init__('measurement_node')

        self.declare_parameter('input_topic', 'sonar_scan')
        self.declare_parameter('lidar_topic', '/cloud_registered')
        self.declare_parameter('odom_topic', 'odometry')
        self.declare_parameter('output_topic', 'semantic_returns')
        self.declare_parameter('classes_file', '')

        # Rolling local bed fit behind h_i.
        self.declare_parameter('bed_cell_size', 0.5)
        self.declare_parameter('bed_quantile', 0.25)
        self.declare_parameter('bed_forget', 0.10)

        # Coverage mask and waterline.
        self.declare_parameter('coverage_mode', 'radius')
        self.declare_parameter('coverage_radius', 3.0)
        self.declare_parameter('waterline_offset', 0.0)

        # Morphology source: 'derived' (legacy descriptors) or 'native'.
        self.declare_parameter('shape_source', 'derived')

        # Consistency check of global.sonar_tilt_deg against the static TF.
        self.declare_parameter('tilt_check_parent', 'os_imu')
        self.declare_parameter('tilt_check_child', 'blueview_sonar')
        self.declare_parameter('tilt_check_tolerance_deg', 2.0)

        # Range-axis corrections applied to ALREADY-RECORDED data.
        self.declare_parameter('range_offset', 0.0)
        self.declare_parameter('sonar_frame', '')

        def gp(n):
            return self.get_parameter(n).value

        classes_file = str(gp('classes_file')).strip()
        if not classes_file:
            classes_file = os.path.join(
                get_package_share_directory('sonar_classification'),
                'config', 'classes.yaml')
        self._cs = load_classes(classes_file)

        self._bed = BottomGrid(cell_size=float(gp('bed_cell_size')),
                               quantile=float(gp('bed_quantile')),
                               forget=float(gp('bed_forget')))

        self._coverage_mode = str(gp('coverage_mode')).lower()
        if self._coverage_mode not in ('radius', 'always'):
            raise ValueError('coverage_mode must be "radius" or "always"')
        self._coverage_radius = max(float(gp('coverage_radius')),
                                    self._cs.eps_xy)
        self._waterline_offset = float(gp('waterline_offset'))
        self._shape_source = str(gp('shape_source')).lower()
        if self._shape_source not in ('derived', 'native'):
            raise ValueError('shape_source must be "derived" or "native"')
        self._legacy_gap = bool(self._cs.ablation.get('lidar_legacy_gap_ref', False))
        self._range_offset = float(gp('range_offset'))
        self._sonar_frame = str(gp('sonar_frame')).strip()

        # Cached inputs
        self._lidar_tree = None     # cKDTree over LiDAR XY
        self._lidar_z = None        # (M,) z of each LiDAR point
        self._lidar_seen = False    # any cloud at all? False => o_i is NaN
        self._sonar_xyz = None      # sonar origin, for the slant range
        self._z_waterline = None    # the surface the vehicle floats on

        self._pub = self.create_publisher(
            PointCloud2, str(gp('output_topic')), 10)
        self.create_subscription(
            PointCloud2, str(gp('input_topic')), self._sweep_cb, 1)
        self.create_subscription(
            PointCloud2, str(gp('lidar_topic')), self._lidar_cb, 1)
        self.create_subscription(
            Odometry, str(gp('odom_topic')), self._odom_cb, 1)

        self._out_layout = [
            PointField(name=nm, offset=4 * i,
                       datatype=PointField.FLOAT32, count=1)
            for i, nm in enumerate(OUT_FIELDS)
        ]
        # ── tilt cross-check against the static TF ───────────────────────────
        # theta is NOT used to transform anything (returns arrive already
        # georeferenced) — only as the height sensitivity s_i = r_i cos(theta)
        # of eq 21. But it describes the same mounting angle the static TF
        # does, so the two are a duplicated source of truth and can drift
        # apart silently. This warns when they have.
        self._tilt_parent = str(gp('tilt_check_parent')).strip()
        self._tilt_child = str(gp('tilt_check_child')).strip()
        self._tilt_tol = float(gp('tilt_check_tolerance_deg'))
        self._tilt_checked = not (self._tilt_parent and self._tilt_child)
        self._tilt_attempts = 0
        # Always listening: the sonar-origin lookup below needs it too.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._origin_warned = False

        self._sweeps = 0
        self.get_logger().info(
            f'measurement_node: {gp("input_topic")} + {gp("lidar_topic")} '
            f'-> {gp("output_topic")} | classes={list(self._cs.names)} '
            f'({classes_file}) | '
            f'eps_xy={self._cs.eps_xy}m k_min={self._cs.k_min} '
            f'coverage={self._coverage_mode}'
            + (f'(r={self._coverage_radius}m)' if self._coverage_mode == 'radius' else '')
            + f' | shape={self._shape_source}'
            + (' | ABLATION: clearance referenced to the RETURN'
               if self._legacy_gap else ''))

    # ── where the ranges are measured FROM, and to what ──────────────────────

    def _sonar_origin(self, map_frame: str) -> np.ndarray:
        """The sonar's own position in the map frame — NOT the vehicle pose.

        The slant range must be measured from the ACOUSTIC ORIGIN of the array,
        because that is what the sonar's own range axis is referenced to ("the
        range FROM THE SONAR HEAD", bvt_magimage.h:172). The odometry pose is
        the vehicle/IMU frame, which on this rig sits 0.22 m forward and 0.42 m
        above the sonar — so using it biases every slant range, and hence every
        compensated intensity, by a geometry-dependent amount.

        Falls back to the odometry position when TF is unavailable, warning
        once: a slightly wrong origin is better than dropping the sweep, but
        silently wrong would not be.
        """
        if self._sonar_frame:
            try:
                tf = self._tf_buffer.lookup_transform(
                    map_frame, self._sonar_frame, rclpy.time.Time())
                t = tf.transform.translation
                return np.array([t.x, t.y, t.z], dtype=np.float64)
            except Exception:  # noqa: BLE001 — TF may not be up yet
                if not self._origin_warned:
                    self._origin_warned = True
                    self.get_logger().warn(
                        f'no {map_frame} -> {self._sonar_frame} transform; '
                        'measuring slant range from the ODOMETRY pose instead. '
                        'That is the vehicle frame, not the acoustic origin, so '
                        'every slant range carries a geometry-dependent bias.')
        return self._sonar_xyz

    def _apply_range_offset(self, xyz: np.ndarray, origin: np.ndarray) -> np.ndarray:
        """Move every return radially, to undo a constant range-axis offset.

        The BlueView driver labels each image row arithmetically,
        ``ranges[i] = start_range + i*inc`` (blooview_interface cache_manager.cpp
        :56-60), but the SDK's actual R-Theta grid does not begin at the
        requested start range. The result is that EVERY range is off by the same
        constant — a pure offset, not a scale, since the bin spacing matches the
        SDK's range resolution to six significant figures.

        A constant range error is NOT a rigid translation and therefore CANNOT
        be absorbed by a TF: the displacement is along each beam, so it fans out
        with bearing. Undoing it means moving each point along ITS OWN beam,
        which is what this does. Doing it here means recorded bags can be
        corrected without touching the driver or re-collecting.

        Positive ``range_offset`` = the published range is TOO LARGE by that
        much, so points move toward the sonar.
        """
        d = xyz - origin[None, :]
        r = np.linalg.norm(d, axis=1)
        good = r > max(self._range_offset, 0.0) + 1e-6
        scale = np.where(good, (r - self._range_offset) / np.maximum(r, 1e-9), 1.0)
        return origin[None, :] + d * scale[:, None]

    # ── the tilt cross-check ─────────────────────────────────────────────────

    def _check_tilt_against_tf(self) -> None:
        """Compare global.sonar_tilt_deg with the mounting pitch in the TF tree.

        Looks up the STATIC parent->child transform (the mounting one), not
        map->sonar, so the vehicle's own live pitch never contaminates the
        comparison. Reports only: theta is a declared mounting constant, and
        silently overriding it from a transform that happened to be available
        would make the model depend on TF timing.
        """
        if self._tilt_checked:
            return
        self._tilt_attempts += 1
        try:
            tf = self._tf_buffer.lookup_transform(
                self._tilt_parent, self._tilt_child, rclpy.time.Time())
        except Exception:  # noqa: BLE001 — TF may simply not be up yet
            if self._tilt_attempts >= 50:
                self._tilt_checked = True
                self.get_logger().info(
                    f'tilt cross-check skipped: no {self._tilt_parent} -> '
                    f'{self._tilt_child} transform after {self._tilt_attempts} '
                    f'sweeps. Using the configured theta = '
                    f'{np.rad2deg(self._cs.tilt_rad):.1f} deg.')
            return

        q = tf.transform.rotation
        pitch = float(Rotation.from_quat([q.x, q.y, q.z, q.w])
                      .as_euler('xyz', degrees=True)[1])
        configured = float(np.rad2deg(self._cs.tilt_rad))
        # The TF states how the child sits in the parent; a downtilt is the
        # magnitude of that pitch, whichever sign convention the mount used.
        tf_tilt = abs(pitch)
        self._tilt_checked = True

        if abs(tf_tilt - configured) > self._tilt_tol:
            self.get_logger().warn(
                f'TILT MISMATCH: classes.yaml global.sonar_tilt_deg = '
                f'{configured:.1f} deg, but the {self._tilt_parent} -> '
                f'{self._tilt_child} static transform carries {tf_tilt:.1f} deg. '
                f'They describe the same mounting angle. Nothing is '
                f'double-rotated (theta only scales the elevation blur '
                f'rho_var_i, eq 22), but the blur is being computed with the '
                f'wrong sensitivity: s_i is off by '
                f'{100.0 * abs(np.cos(np.deg2rad(configured)) / np.cos(np.deg2rad(tf_tilt)) - 1.0):.0f}%. '
                f'Set global.sonar_tilt_deg: {tf_tilt:.1f}.')
        else:
            self.get_logger().info(
                f'tilt cross-check OK: classes.yaml {configured:.1f} deg vs '
                f'{self._tilt_parent} -> {self._tilt_child} TF {tf_tilt:.1f} deg.')

    # ── the calibration anchor check ─────────────────────────────────────────

    # ── cached inputs ────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._sonar_xyz = np.array([p.x, p.y, p.z], dtype=np.float64)
        # The vehicle floats on the waterline, so its own pose fixes it up to a
        # rigid mounting offset (negative when the pose origin sits above the
        # water, which it usually does).
        self._z_waterline = float(p.z) + self._waterline_offset

    def _lidar_cb(self, msg: PointCloud2):
        try:
            xyz = _read_fields(msg, ('x', 'y', 'z'))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Bad LiDAR cloud: {exc}',
                                   throttle_duration_sec=5.0)
            return
        if xyz.size:
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if xyz.shape[0] > 0:
            self._lidar_tree = cKDTree(xyz[:, :2].astype(np.float64))
            self._lidar_z = xyz[:, 2].astype(np.float64)
        else:
            self._lidar_tree = None
            self._lidar_z = None
        # Even an empty decoded cloud flips this: from here on, "no points in
        # the column" is a genuine observation rather than sensor silence.
        self._lidar_seen = True

    # ── stage 3: the overhead column (eqs 26-27) ─────────────────────────────

    def _overhead_column(self, xyz: np.ndarray):
        """(o_i, s_lidar_i, g_tilde_i) per return.

        o_i answers "did the LiDAR sweep this column at all". With
        ``coverage_mode: radius`` a column counts as observed when LiDAR points
        exist within ``coverage_radius`` of its XY — the scanner demonstrably
        put returns in that neighbourhood, so a genuinely empty column there is
        informative. Outside that, o_i = 0 and the whole factor becomes 1 for
        every class, which is the SAFE direction: a false "not observed" only
        costs evidence, whereas a false "observed empty" charges surface-
        piercing classes for support they never had a chance to show.
        """
        n = xyz.shape[0]
        nan = np.full(n, np.nan, dtype=np.float64)
        if not self._lidar_seen:
            return nan, nan.copy(), nan.copy()        # no data: abstain
        if self._lidar_tree is None or n == 0:
            return (np.zeros(n), np.zeros(n), np.full(n, np.nan))

        xy = xyz[:, :2].astype(np.float64)

        if self._coverage_mode == 'always':
            observed = np.ones(n, dtype=np.float64)
        else:
            near = self._lidar_tree.query_ball_point(
                xy, r=self._coverage_radius, return_length=True)
            observed = (np.asarray(near) > 0).astype(np.float64)

        # K_i (eq 26) and, from it, the VALIDATED presence bit plus the
        # waterline-referenced clearance (eq 27).
        balls = self._lidar_tree.query_ball_point(xy, r=self._cs.eps_xy)
        lz = self._lidar_z
        present = np.zeros(n, dtype=np.float64)
        clearance = np.full(n, np.nan, dtype=np.float64)
        z_ref_scene = (self._z_waterline if self._z_waterline is not None
                       else np.nan)

        for i, members in enumerate(balls):
            if len(members) < self._cs.k_min:
                continue                       # the count is a GATE, no more
            present[i] = 1.0
            z_low = float(np.min(lz[members]))
            # Waterline reference (eq 27). The ablation restores the older
            # return-referenced gap, which couples this channel to the position
            # factor and reintroduces the bridge problem.
            clearance[i] = (z_low - xyz[i, 2]) if self._legacy_gap \
                else (z_low - z_ref_scene)

        return observed, present, clearance

    # ── per-sweep processing ─────────────────────────────────────────────────

    def _sweep_cb(self, msg: PointCloud2):
        if self._sonar_xyz is None:
            self.get_logger().warn('No odometry yet — skipping sweep.',
                                   throttle_duration_sec=5.0)
            return

        present_fields = {f.name for f in msg.fields}
        wanted = (NATIVE_SHAPE_FIELDS if self._shape_source == 'native'
                  else LEGACY_SHAPE_FIELDS)
        has_shape = all(nm in present_fields for nm in wanted)
        names = ('x', 'y', 'z', 'intensity') + (tuple(wanted) if has_shape else ())

        try:
            pts = _read_fields(msg, names)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to read input cloud: {exc}',
                                    throttle_duration_sec=5.0)
            return
        if pts.shape[0] == 0:
            return

        # Gate row survival on the geometric core only (descriptor NaNs are data).
        core_ok = np.isfinite(pts[:, :4]).all(axis=1) & (pts[:, 3] > 0.0)
        pts = pts[core_ok]
        if pts.shape[0] == 0:
            return

        xyz = pts[:, :3].astype(np.float64)
        counts = pts[:, 3].astype(np.float64)          # PRISTINE raw count (I1)

        # ── stage 2: measurement ─────────────────────────────────────────────
        # Ranges are referenced to the sonar's acoustic origin, not the vehicle
        # pose, and are corrected for the driver's constant range-axis offset
        # BEFORE anything derived from them is computed.
        origin = self._sonar_origin(msg.header.frame_id)
        if self._range_offset != 0.0:
            xyz = self._apply_range_offset(xyz, origin)
        slant = np.linalg.norm(xyz - origin[None, :], axis=1)
        saturated = counts >= 65535.0
        rho_var = geometry.elevation_variance(slant, self._cs)

        # Update the bed fit THEN query it, so the reference starts warming up
        # from the very first sweep instead of lagging one behind.
        self._bed.update(xyz[:, 0], xyz[:, 1], xyz[:, 2])
        z_bed = self._bed.query(xyz[:, 0], xyz[:, 1])
        h = geometry.height_above_bed(xyz[:, 2], z_bed)

        # ── stage 3: LiDAR query ─────────────────────────────────────────────
        observed, lidar_present, clearance = self._overhead_column(xyz)

        # ── morphology descriptor ────────────────────────────────────────────
        n = xyz.shape[0]
        if not has_shape:
            flag = np.full(n, np.nan)
            e_r = np.full(n, np.nan)
            w_r = np.full(n, np.nan)
            rid = np.full(n, -1.0)
        elif self._shape_source == 'native':
            flag, e_r, w_r, rid = region_descriptor.build(
                pts[:, 4].astype(np.float64),
                extent=pts[:, 5].astype(np.float64),
                thickness=pts[:, 6].astype(np.float64),
                mode='native')
        else:
            flag, e_r, w_r, rid = region_descriptor.build(
                pts[:, 4].astype(np.float64),
                n_beams=pts[:, 5].astype(np.float64),
                log_area=pts[:, 6].astype(np.float64),
                region_range=slant,
                beam_spacing_rad=self._cs.beam_spacing_rad,
                beamwidth_rad=self._cs.beamwidth_rad,
                mode='derived')

        self._publish(msg, xyz, slant, counts, saturated, h,
                      rho_var, observed, lidar_present, clearance,
                      flag, e_r, w_r, rid)

        self._check_tilt_against_tf()
        self._sweeps += 1
        if self._sweeps <= 5 or self._sweeps % 50 == 0:
            fin = counts[counts > 0]
            cov = observed[np.isfinite(observed)]
            i_txt = (f'A median={np.median(fin):.0f} counts' if fin.size
                     else 'A: every count at or below zero')
            cov_txt = (f'lidar observed={cov.mean():.0%}, '
                       f's=1 frac={np.mean(lidar_present > 0.5):.2f}'
                       if cov.size else 'lidar: no cloud yet (factor abstains)')
            shape_txt = (f'region returns={int(np.sum(flag == 1.0))}'
                         if has_shape else 'shape: no extractor on this cloud')
            self.get_logger().info(
                f'sweep #{self._sweeps}: {n} returns | {i_txt} | '
                f'bed cov={np.isfinite(h).mean():.0%} | {cov_txt} | {shape_txt}')

    def _publish(self, src, xyz, slant, counts, saturated, h,
                 rho_var, observed, present, clearance, flag, e_r, w_r, rid):
        out = np.empty((xyz.shape[0], len(OUT_FIELDS)), dtype=np.float32)
        out[:, 0:3] = xyz
        out[:, 3] = slant
        out[:, 4] = counts
        out[:, 5] = saturated
        out[:, 6] = h
        out[:, 7] = rho_var
        out[:, 8] = observed
        out[:, 9] = present
        out[:, 10] = clearance
        out[:, 11] = flag
        out[:, 12] = e_r
        out[:, 13] = w_r
        out[:, 14] = rid
        self._pub.publish(pc2.create_cloud(src.header, self._out_layout, out))


def main(args=None):
    rclpy.init(args=args)
    node = MeasurementNode()
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
