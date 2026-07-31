#!/usr/bin/env python3
"""
return_classifier — Node 2 of the Dirichlet voxel mapping pipeline.
===============================================================================

Despite the historical name, this node assigns no class label. Classification
is the SOFT per-voxel Dirichlet responsibility r_{i,c} ∝ L_{i,c}·π_c(N_v)
(eq 16) assembled downstream in voxel_mapper. This node takes each raw sonar
leading-edge return and attaches the RAW measurement channels
y_i = (p_i, I_i, ℓ_i, m_i) (eq 9) that the FOUR FACTORS of the paper's
class-conditional likelihood consume:

    L_{i,c} = p(p_i|c) · p(I_i|p_i,c) · p(ℓ_i|c) · p(m_i|c)          (eq 24)

  * POSITION p(p_i|c) (eqs 25–31): x, y, z (boresight, eq 26 at ϕ=0), the
    slant range r_i (ϱ_i of eq 30 is computed downstream from it), and the
    local bottom reference z_bed = ẑ_bed(x_i, y_i) needed for h_i = z_i − ẑ_bed
    (eq 25). ẑ_bed comes from a rolling BottomGrid updated with every scan:
    only a LOW z-quantile per cell enters, so returns from walls and objects
    (above the bed) bias the reference little — and the position factor widens
    ω_c for structure classes precisely because this reference is approximate.
    z_bed is NaN until the grid has warmed up there (factor abstains).
  * INTENSITY p(I_i|p_i,c) (eq 34): the raw backscatter I_i. Class intensity
    means μ_c are ABSOLUTE values from classes.yaml, so the scan-level seabed
    context (μ_I, σ_I) from seabed_estimator is TELEMETRY ONLY and OPTIONAL —
    a missing estimator state never drops a scan (fields go out as 0.0).
  * LIDAR p(ℓ_i|c) (eqs 35–37) — the PRIMARY LiDAR channels: an
    overhead-column query around the RETURN's own XY. Per return,
        K_i = { q ∈ P_L : ||(x_q,y_q) − (x_i,y_i)|| ≤ ε_xy }         (eq 35)
        over_count k_i = |K_i|
        over_gap   g_i = min_{q∈K_i} z_q − z_i                       (eq 36)
    g_i is NaN when k_i = 0 (hurdle: magnitude modelled only when present).
    Before ANY LiDAR cloud has been received, over_count itself is NaN so the
    factor ABSTAINS downstream rather than asserting "not structure"; an
    empty-but-received cloud is a genuine k_i = 0 observation.
  * SHAPE p(m_i|c) (eqs 39–42): the six region-descriptor channels computed by
    the leading-edge extractor (shape_flag, region_size, region_logarea,
    region_solidity, region_shadow, region_texture) ride through unchanged
    when the incoming cloud carries them; when it does not (old bags,
    extractor off), shape_flag is NaN and the factor abstains.

WHY OVERHEAD/NEAREST MATCHES ARE XY-ONLY: the sonar images the seabed from
underwater while the LiDAR scans from the surface, so a submerged structure
return sits directly BELOW its above-water LiDAR return — same x,y, separated
by the (large, roughly constant) waterline gap in z. An R^3 match folds that
gap into every distance and kills all support even directly under a structure;
the vertical LiDAR column above the return is the signal that actually
discriminates "under structure" from "open seabed". The gap g_i then carries
the useful part of z as a signed magnitude instead of a distance penalty.

LEGACY q*-BASED CHANNELS (kept for the debug observers, NOT used by eq 24):
        q*    = nearest LiDAR point to the return p_i, in the XY plane
        d_i   = || p_i,xy − q*_xy ||                    (lidar_dist)
        k_i   = | { LiDAR pts c : || c_xy − q*_xy || ≤ ε_L
                                   [and |c_z − q*_z| ≤ ε_Lz] } |   (lidar_count)
The horizontal radius ε_L gates the legacy count around q*; an OPTIONAL z band
of half-height ε_Lz (param ``eps_Lz``) can be intersected into that COUNT only
(a cylinder around q*), so LiDAR at a wildly different height than q* no longer
inflates it. ε_Lz ≤ 0 disables the band (the default).

Zero-intensity / non-detection beams are dropped: the paper's categorical has no
FREE class, so only genuine leading-edge returns are forwarded.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import numpy.lib.recfunctions as rfn
from scipy.spatial import cKDTree

from sonar_map_msgs.msg import ClassifiedReturns, SeabedEstimatorState
from sonar_map.dirichlet.bottom_grid import BottomGrid


# ── Defaults — overridable from the YAML config / launch arguments ────────────
INPUT_TOPIC   = 'sonar_scan'
LIDAR_TOPIC   = '/cloud_registered'
STATE_TOPIC   = 'seabed_estimator_state'
ODOM_TOPIC    = 'odometry'
OUTPUT_TOPIC  = 'classified_returns'

EPS_XY        = 0.30     # [m]  ε_xy — overhead-column radius around the RETURN (eq 35)
EPS_L         = 0.15     # [m]  ε_L — legacy support-count radius around q* (debug only)
EPS_LZ        = 0.0      # [m]  ε_Lz — z half-height of the legacy count cylinder; ≤0 = pure XY

BED_CELL_SIZE = 0.5      # [m]  BottomGrid cell size for ẑ_bed(x, y)
BED_QUANTILE  = 0.25     # low z-quantile per cell (robust to structure above the bed)
BED_FORGET    = 0.10     # EMA blend of the per-scan quantile into the stored cell

# Shape/morphology descriptor fields (eqs 39–40), in ClassifiedReturns order.
# Copied through when the leading-edge extractor put them on the cloud.
SHAPE_FIELDS  = ('shape_flag', 'region_size', 'region_logarea',
                 'region_solidity', 'region_shadow', 'region_texture')


def _read_fields(msg: PointCloud2, fields: tuple,
                 skip_nans: bool = True) -> np.ndarray:
    """Read the requested fields into an (N, len(fields)) float32 array.

    skip_nans=False is required whenever the descriptor channels are read: their
    NaNs are DATA (hurdle abstention, eq 42) and must not drop rows — the caller
    gates row survival on the geometric core itself.
    """
    raw = pc2.read_points(msg, field_names=fields, skip_nans=skip_nans)
    if isinstance(raw, np.ndarray):
        if raw.size == 0:
            return np.empty((0, len(fields)), dtype=np.float32)
        if raw.dtype.names is not None:
            return rfn.structured_to_unstructured(raw, dtype=np.float32)
        return raw.reshape(-1, len(fields)).astype(np.float32)
    rows = list(raw)
    if not rows:
        return np.empty((0, len(fields)), dtype=np.float32)
    return np.array(rows, dtype=np.float32).reshape(-1, len(fields))


class ReturnClassifierNode(Node):

    def __init__(self):
        super().__init__('return_classifier')

        self.declare_parameter('input_topic',   INPUT_TOPIC)
        self.declare_parameter('lidar_topic',   LIDAR_TOPIC)
        self.declare_parameter('state_topic',   STATE_TOPIC)
        self.declare_parameter('odom_topic',    ODOM_TOPIC)
        self.declare_parameter('output_topic',  OUTPUT_TOPIC)
        self.declare_parameter('eps_xy',        EPS_XY)
        self.declare_parameter('eps_L',         EPS_L)
        self.declare_parameter('eps_Lz',        EPS_LZ)
        self.declare_parameter('bed_cell_size', BED_CELL_SIZE)
        self.declare_parameter('bed_quantile',  BED_QUANTILE)
        self.declare_parameter('bed_forget',    BED_FORGET)

        def gp(n):
            return self.get_parameter(n).value

        self.eps_xy_ = max(float(gp('eps_xy')), 1e-6)  # ε_xy — eq-35 column radius
        self.eps_l_  = max(float(gp('eps_L')), 1e-6)   # legacy q*-disc radius
        self.eps_lz_ = float(gp('eps_Lz'))   # ≤0 ⇒ z band disabled (pure XY)

        # Rolling ẑ_bed(x, y) reference for the position factor (eq 25); updated
        # once per scan, queried at the same scan's XY. Pure NumPy — no ROS.
        self._bottom_grid_ = BottomGrid(cell_size=float(gp('bed_cell_size')),
                                        quantile=float(gp('bed_quantile')),
                                        forget=float(gp('bed_forget')))

        # Latest inputs cached between callbacks
        self._estimator_state = None     # SeabedEstimatorState (telemetry, OPTIONAL)
        self._lidar_tree      = None     # cKDTree of LiDAR XY (horizontal)
        self._lidar_z         = None     # (M,) float64 z of each LiDAR pt
        self._lidar_received_ = False    # any cloud at all? False ⇒ over_count NaN
        self._usv_xyz         = None     # (3,) float64 sonar origin
        self._warned_no_state_ = False   # warn ONCE about missing estimator state

        self.pub_ = self.create_publisher(
            ClassifiedReturns, gp('output_topic'), 10)
        self.sub_scan_  = self.create_subscription(
            PointCloud2, gp('input_topic'), self._scan_cb, 1)
        self.sub_lidar_ = self.create_subscription(
            PointCloud2, str(gp('lidar_topic')), self._lidar_cb, 1)
        self.sub_state_ = self.create_subscription(
            SeabedEstimatorState, gp('state_topic'), self._state_cb, 10)
        self.sub_odom_  = self.create_subscription(
            Odometry, gp('odom_topic'), self._odom_cb, 1)

        self._scan_count_ = 0
        self.get_logger().info(
            f'return_classifier: {gp("input_topic")} + {gp("lidar_topic")} '
            f'-> {gp("output_topic")} | eq-24 channels: position(x,y,z,r,z_bed) '
            f'intensity(I) lidar(k,g) shape(m) | '
            f'eps_xy={self.eps_xy_}m (overhead column, eqs 35-36) | '
            f'bed cell={float(gp("bed_cell_size"))}m '
            f'q={float(gp("bed_quantile"))} forget={float(gp("bed_forget"))} | '
            f'legacy eps_L={self.eps_l_}m'
            + (f' ∩ |Δz|≤{self.eps_lz_}m (z band)' if self.eps_lz_ > 0.0
               else ' [z band off]'))

    # ── input caches ──────────────────────────────────────────────────────────

    def _state_cb(self, msg: SeabedEstimatorState):
        self._estimator_state = msg

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._usv_xyz = np.array([p.x, p.y, p.z], dtype=np.float64)

    def _lidar_cb(self, msg: PointCloud2):
        try:
            xyz = _read_fields(msg, ('x', 'y', 'z'))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Bad LiDAR cloud: {exc}',
                                   throttle_duration_sec=5.0)
            return
        xyz = xyz[np.isfinite(xyz).all(axis=1)] if xyz.size else xyz
        # XY (horizontal) tree — shared by the eq-35 overhead-column query (disc
        # around the RETURN) and the legacy q* match. z is cached separately so
        # the gap g_i (eq 36) and the optional legacy count-cylinder band
        # (eps_Lz) can be applied without folding z into the horizontal match.
        if xyz.shape[0] > 0:
            self._lidar_tree = cKDTree(xyz[:, :2])
            self._lidar_z    = xyz[:, 2].astype(np.float64)
        else:
            self._lidar_tree = None
            self._lidar_z    = None
        # Even an empty decoded cloud flips this: absence of points is then a
        # genuine "nothing overhead" observation (k=0), not sensor silence.
        self._lidar_received_ = True

    # ── overhead-column LiDAR channels ℓ_i = (k_i, g_i) (eqs 35–36) ───────────

    def _overhead_column(self, xyz: np.ndarray):
        """Return (over_count, over_gap) per return, both float32 (N,).

        K_i (eq 35) is the set of LiDAR points within ε_xy of the RETURN's own
        XY — the vertical column above it. From it (eq 36):
            over_count k_i = |K_i|
            over_gap   g_i = min_{q∈K_i} z_q − z_i   (NaN when k_i = 0)
        Abstention semantics: before any LiDAR cloud has been received both
        channels are NaN (the eq-37 factor abstains); once a cloud exists,
        k_i = 0 is a real "no structure overhead" observation.
        """
        n = xyz.shape[0]
        if not self._lidar_received_:
            return (np.full(n, np.nan, dtype=np.float32),
                    np.full(n, np.nan, dtype=np.float32))
        if self._lidar_tree is None or n == 0:
            return (np.zeros(n, dtype=np.float32),
                    np.full(n, np.nan, dtype=np.float32))

        balls = self._lidar_tree.query_ball_point(xyz[:, :2], r=self.eps_xy_)
        lz    = self._lidar_z
        counts = np.zeros(n, dtype=np.float64)
        gaps   = np.full(n, np.nan, dtype=np.float64)
        for i, members in enumerate(balls):
            if members:
                counts[i] = float(len(members))
                gaps[i]   = float(np.min(lz[members])) - xyz[i, 2]
        return counts.astype(np.float32), gaps.astype(np.float32)

    # ── legacy q*-based LiDAR support (d, k) — debug observers only ───────────

    def _lidar_support(self, xyz: np.ndarray):
        """Return legacy (d, k) per return, both float32 (N,). NOT used by eq 24.

        d is always HORIZONTAL (XY only):
            d = || p_xy − q*_xy ||                    nearest-LiDAR distance [m]
        k is the support count around q*:
            ε_Lz ≤ 0:  k = # LiDAR within ε_L of q* in XY            (a disc)
            ε_Lz > 0:  k = # LiDAR within ε_L of q* in XY AND within
                           ε_Lz of q*'s z                            (a cylinder)
        """
        n = xyz.shape[0]
        if self._lidar_tree is None or n == 0:
            return (np.full(n, np.inf, dtype=np.float32),
                    np.zeros(n, dtype=np.float32))

        xy = xyz[:, :2]                                  # horizontal only
        d_arr, idx = self._lidar_tree.query(xy, k=1)
        d_arr = np.asarray(d_arr, dtype=np.float64).reshape(n)
        idx   = np.asarray(idx,   dtype=np.int64 ).reshape(n)

        qstar = np.asarray(self._lidar_tree.data, dtype=np.float64)[idx]  # (n,2) XY

        if self.eps_lz_ <= 0.0 or self._lidar_z is None:
            # z band disabled → original vectorised XY-disc count.
            counts = self._lidar_tree.query_ball_point(
                qstar, r=self.eps_l_, return_length=True)
            counts = np.asarray(counts, dtype=np.float64)
        else:
            # Cylinder: XY disc ∩ z band of half-height ε_Lz centred on q*'s z.
            # Need the member indices (not just the length) to apply the band.
            qstar_z = self._lidar_z[idx]                 # (n,)
            ball    = self._lidar_tree.query_ball_point(qstar, r=self.eps_l_)
            lz      = self._lidar_z
            counts  = np.empty(n, dtype=np.float64)
            for i in range(n):
                members = ball[i]
                if members:
                    dz = np.abs(lz[members] - qstar_z[i])
                    counts[i] = float(np.count_nonzero(dz <= self.eps_lz_))
                else:
                    counts[i] = 0.0
        return d_arr.astype(np.float32), counts.astype(np.float32)

    # ── main per-scan processing ──────────────────────────────────────────────

    def _scan_cb(self, msg: PointCloud2):
        if self._usv_xyz is None:
            self.get_logger().warn('No odometry yet — skipping scan.',
                                   throttle_duration_sec=5.0)
            return

        # Shape channels ride on the same cloud when the extractor emitted them.
        present   = {f.name for f in msg.fields}
        has_shape = all(nm in present for nm in SHAPE_FIELDS)
        fields    = ('x', 'y', 'z', 'intensity') + (SHAPE_FIELDS if has_shape
                                                    else ())
        try:
            # skip_nans OFF: descriptor NaNs are the hurdle abstention signal
            # (eq 42) and must not drop rows — gate on the core only, below.
            pts = _read_fields(msg, fields, skip_nans=False)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to read PointCloud2: {exc}',
                                    throttle_duration_sec=5.0)
            return
        if pts.size:
            pts = pts[np.isfinite(pts[:, :4]).all(axis=1)]
        if pts.shape[0] == 0:
            return

        # Drop non-detections (zero/negative intensity): the paper's categorical
        # has no FREE class, so only genuine leading-edge returns are forwarded.
        keep = pts[:, 3] > 0.0
        pts = pts[keep]
        if pts.shape[0] == 0:
            return

        xyz       = pts[:, :3].astype(np.float64)
        intensity = pts[:, 3].astype(np.float64)
        if has_shape:
            shape = pts[:, 4:4 + len(SHAPE_FIELDS)].astype(np.float64)
        else:
            # No extractor on this cloud: shape_flag NaN ⇒ eq-42 factor abstains.
            shape = np.full((pts.shape[0], len(SHAPE_FIELDS)), np.nan)

        # Seabed-intensity context — TELEMETRY ONLY (eq 24 uses absolute class
        # intensities μ_c), so a missing estimator state never costs a scan.
        if self._estimator_state is not None:
            mu_i    = float(self._estimator_state.mu_i)
            var_i   = float(self._estimator_state.var_i)
            sigma_i = float(np.sqrt(max(var_i, 0.0)))
        else:
            if not self._warned_no_state_:
                self.get_logger().warn(
                    'No seabed_estimator state — publishing μ_I=σ_I=0. '
                    'Telemetry only: the eq-24 factors do not consume it.')
                self._warned_no_state_ = True
            mu_i, sigma_i = 0.0, 0.0

        # Slant range = distance from the sonar (USV) to each return.
        slant = np.linalg.norm(xyz - self._usv_xyz[None, :], axis=1)

        # Bottom reference ẑ_bed (eq 25): update THEN query, so the reference
        # starts warming up from the very first scan instead of lagging one.
        self._bottom_grid_.update(xyz[:, 0], xyz[:, 1], xyz[:, 2])
        z_bed = np.asarray(self._bottom_grid_.query(xyz[:, 0], xyz[:, 1]),
                           dtype=np.float64)

        # Primary LiDAR channels ℓ_i = (k_i, g_i) (eqs 35–36).
        over_k, over_g = self._overhead_column(xyz)
        # Legacy q*-based (d, k) — kept alive for the debug observers.
        d_arr, k_arr = self._lidar_support(xyz)

        self._publish(msg, xyz, intensity, slant, z_bed, over_k, over_g,
                      d_arr, k_arr, shape, mu_i, sigma_i)

        self._scan_count_ += 1
        if self._scan_count_ <= 5 or self._scan_count_ % 50 == 0:
            bed_cov  = float(np.isfinite(z_bed).mean()) if z_bed.size else 0.0
            finite_k = over_k[np.isfinite(over_k)]
            k_med = float(np.median(finite_k)) if finite_k.size else float('nan')
            over_frac = (float(np.mean(finite_k > 0)) if finite_k.size
                         else float('nan'))
            self.get_logger().info(
                f'scan #{self._scan_count_}: {intensity.size} returns | '
                f'z_bed cov={bed_cov:.0%} | over k med={k_med:.0f} '
                f's=1 frac={over_frac:.2f} | shape={"on" if has_shape else "off"} | '
                f'μ_I={mu_i:.0f} σ_I={sigma_i:.0f}')

    def _publish(self, src_msg, xyz, intensity, slant, z_bed, over_k, over_g,
                 d_arr, k_arr, shape, mu_i, sigma_i):
        out = ClassifiedReturns()
        out.header.stamp = src_msg.header.stamp
        out.header.frame_id = src_msg.header.frame_id
        out.mu_i    = float(mu_i)
        out.sigma_i = float(sigma_i)
        out.x            = xyz[:, 0].astype(np.float32).tolist()
        out.y            = xyz[:, 1].astype(np.float32).tolist()
        out.z            = xyz[:, 2].astype(np.float32).tolist()
        out.intensity    = intensity.astype(np.float32).tolist()
        out.slant_range  = slant.astype(np.float32).tolist()
        out.z_bed        = z_bed.astype(np.float32).tolist()
        out.over_count   = over_k.astype(np.float32).tolist()
        out.over_gap     = over_g.astype(np.float32).tolist()
        out.lidar_dist   = d_arr.astype(np.float32).tolist()
        out.lidar_count  = k_arr.astype(np.float32).tolist()
        out.shape_flag      = shape[:, 0].astype(np.float32).tolist()
        out.region_size     = shape[:, 1].astype(np.float32).tolist()
        out.region_logarea  = shape[:, 2].astype(np.float32).tolist()
        out.region_solidity = shape[:, 3].astype(np.float32).tolist()
        out.region_shadow   = shape[:, 4].astype(np.float32).tolist()
        out.region_texture  = shape[:, 5].astype(np.float32).tolist()
        self.pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ReturnClassifierNode()
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
