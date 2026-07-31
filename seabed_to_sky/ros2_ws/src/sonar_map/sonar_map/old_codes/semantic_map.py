#!/usr/bin/env python3
"""
semantic_map.py
---------------
Classifies voxels in the sonar map into semantic classes and publishes
a new point cloud with a ``semantic`` field.

CLASSES
  SEABED    (0)  — default
  STRUCTURE (1)  — voxel that has LiDAR support in XY AND high intensity

CLASSIFICATION RULE FOR STRUCTURE
  A sonar voxel at (xi, yi, zi) with mean intensity Ii is labelled STRUCTURE iff:

    Condition 1 (LiDAR support):
      The number of LiDAR points within an XY circle of radius ``epsilon``
      centred at (xi, yi) is >= 1.

    Condition 2 (Intensity gate):
      Ii  >  lambda_s * I_avg_seabed

  where I_avg_seabed is estimated online as the median intensity across all
  published sonar voxels (robust proxy — seabed returns dominate the map).

PARAMETERS
  sonar_topic    str    Input sonar map topic.         Default: sonar_map
  lidar_topic    str    LiDAR accumulated cloud topic. Default: /cloud_registered
  output_topic   str    Semantic map output topic.     Default: sonar_map_semantic
  map_frame      str    TF frame for the output.       Default: odom
  epsilon        float  XY neighbourhood radius [m].   Default: 0.5
  lambda_s       float  Intensity multiplier.          Default: 1.5
  publish_rate   float  Publication rate [Hz].         Default: 2.0

OUTPUT FIELDS
  x, y, z, prob, intensity, source, semantic
  semantic: 0.0 = SEABED  |  1.0 = STRUCTURE
"""

import numpy as np
import numpy.lib.recfunctions as rfn

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

from scipy.spatial import cKDTree


# ── Semantic labels ────────────────────────────────────────────────────────────
LABEL_SEABED    = np.float32(0.0)
LABEL_STRUCTURE = np.float32(1.0)

# ── Defaults ───────────────────────────────────────────────────────────────────
SONAR_TOPIC   = 'sonar_map'
LIDAR_TOPIC   = '/cloud_registered'
OUTPUT_TOPIC  = 'sonar_map_semantic'
MAP_FRAME     = 'odom'
EPSILON            = 0.5    # [m] XY radius for LiDAR neighbourhood
LAMBDA_S           = 1.0    # intensity threshold multiplier
SEABED_PERCENTILE  = 80.0   # keep bottom N% of intensities to estimate seabed level
PUBLISH_RATE       = 2.0    # [Hz]


# ── Helpers ────────────────────────────────────────────────────────────────────

def _read_fields(msg: PointCloud2, fields: tuple) -> np.ndarray:
    """
    Read the requested fields from a PointCloud2 into an (N, len(fields)) float32
    array.  Returns an empty array if the cloud is empty or the read fails.
    """
    raw = pc2.read_points(msg, field_names=fields, skip_nans=True)
    if isinstance(raw, np.ndarray):
        if raw.size == 0:
            return np.empty((0, len(fields)), dtype=np.float32)
        if raw.dtype.names is not None:
            return rfn.structured_to_unstructured(raw, dtype=np.float32)
        return raw.reshape(-1, len(fields)).astype(np.float32)
    rows = list(raw)
    if not rows:
        return np.empty((0, len(fields)), dtype=np.float32)
    try:
        return np.array(rows, dtype=np.float32).reshape(-1, len(fields))
    except Exception:
        pts = np.empty((len(rows), len(fields)), dtype=np.float32)
        for i, r in enumerate(rows):
            for j in range(len(fields)):
                pts[i, j] = float(r[j])
        return pts


# ── Node ───────────────────────────────────────────────────────────────────────

class SemanticMapNode(Node):
    """
    Subscribes to the sonar voxel map and an accumulated LiDAR cloud.
    Classifies each sonar voxel as SEABED (0) or STRUCTURE (1) and
    republishes the map with an additional ``semantic`` float32 field.
    """

    def __init__(self):
        super().__init__('semantic_map')

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter('sonar_topic',  SONAR_TOPIC)
        self.declare_parameter('lidar_topic',  LIDAR_TOPIC)
        self.declare_parameter('output_topic', OUTPUT_TOPIC)
        self.declare_parameter('map_frame',    MAP_FRAME)
        self.declare_parameter('epsilon',           EPSILON)
        self.declare_parameter('lambda_s',          LAMBDA_S)
        self.declare_parameter('seabed_percentile', SEABED_PERCENTILE)
        self.declare_parameter('publish_rate',      PUBLISH_RATE)

        def gp(n): return self.get_parameter(n).value

        self._map_frame         = str(gp('map_frame'))
        self._epsilon           = float(gp('epsilon'))
        self._lambda_s          = float(gp('lambda_s'))
        self._seabed_percentile = float(gp('seabed_percentile'))

        sonar_topic  = str(gp('sonar_topic'))
        lidar_topic  = str(gp('lidar_topic'))
        output_topic = str(gp('output_topic'))
        rate         = float(gp('publish_rate'))

        # ── internal state ────────────────────────────────────────────────
        # sonar_map is read as (N,6): x,y,z,prob,intensity,source
        self._sonar_pts: np.ndarray | None = None
        # LiDAR XY KD-tree rebuilt on every new lidar cloud
        self._lidar_tree: cKDTree | None = None

        # ── output PointField definitions ─────────────────────────────────
        # Mirrors sonar_map fields + one extra 'semantic' float32
        self._fields_out = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='prob',      offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='source',    offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='semantic',  offset=24, datatype=PointField.FLOAT32, count=1),
        ]

        # ── ROS2 I/O ──────────────────────────────────────────────────────
        self._pub = self.create_publisher(PointCloud2, output_topic, 10)

        self.create_subscription(PointCloud2, sonar_topic, self._sonar_cb,  10)
        self.create_subscription(PointCloud2, lidar_topic, self._lidar_cb,  10)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'SemanticMap ready\n'
            f'  epsilon  = {self._epsilon} m\n'
            f'  lambda_s = {self._lambda_s}\n'
            f'  sonar    : {sonar_topic}\n'
            f'  lidar    : {lidar_topic}\n'
            f'  output   : {output_topic}'
        )

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _sonar_cb(self, msg: PointCloud2):
        """Store latest sonar voxel map as (N,6) float32."""
        pts = _read_fields(msg, ('x', 'y', 'z', 'prob', 'intensity', 'source'))
        if pts.shape[0] > 0:
            self._sonar_pts = pts

    def _lidar_cb(self, msg: PointCloud2):
        """
        Build a 2-D KD-tree from the XY coordinates of the incoming LiDAR cloud.
        Called every time a new LiDAR cloud arrives (typically the accumulated
        /cloud_registered map from spark_fast_lio).
        """
        pts = _read_fields(msg, ('x', 'y', 'z'))
        if pts.shape[0] == 0:
            return
        xy = pts[:, :2].astype(np.float64)
        self._lidar_tree = cKDTree(xy)

    # ── classification ─────────────────────────────────────────────────────────

    def _classify(self, sonar: np.ndarray) -> np.ndarray:
        """
        Classify each sonar voxel.

        Parameters
        ----------
        sonar : (N, 6) float32  columns: x, y, z, prob, intensity, source

        Returns
        -------
        labels : (N,) float32
            LABEL_SEABED (0.0) or LABEL_STRUCTURE (1.0) per voxel.
        """
        n      = sonar.shape[0]
        labels = np.full(n, LABEL_SEABED, dtype=np.float32)

        # ── Estimate average seabed intensity from the map itself ──────────
        # Take the bottom 80 % of non-zero returns (by intensity) and
        # compute their median — this excludes bright structure/object voxels
        # from biasing the seabed estimate upward.
        intensities  = sonar[:, 4]
        nonzero_I    = intensities[intensities > 0.0]
        if nonzero_I.size > 0:
            p_cut        = float(np.percentile(nonzero_I, self._seabed_percentile))
            bottom_pct   = nonzero_I[nonzero_I <= p_cut]
            I_avg_seabed = float(np.median(bottom_pct))
        else:
            I_avg_seabed = 1.0
        I_threshold  = self._lambda_s * I_avg_seabed

        # ── Condition 2: Intensity gate ────────────────────────────────────
        high_I = intensities > I_threshold  # (N,)

        # Early exit: no voxel passes the intensity gate
        if not np.any(high_I):
            return labels

        # ── Condition 1: LiDAR support in XY ──────────────────────────────
        if self._lidar_tree is None:
            self.get_logger().warn(
                'No LiDAR cloud received yet — all voxels labelled SEABED.',
                throttle_duration_sec=5.0)
            return labels

        # Only query the KD-tree for voxels that already pass the intensity
        # gate (avoids querying all N voxels when most fail condition 2).
        candidate_idx  = np.where(high_I)[0]
        candidate_xy   = sonar[candidate_idx, :2].astype(np.float64)

        # query_ball_point with return_length=True gives the count directly.
        lidar_counts   = self._lidar_tree.query_ball_point(
            candidate_xy, r=self._epsilon, return_length=True)
        has_lidar      = np.asarray(lidar_counts, dtype=np.int32) > 0

        # ── Mark structures ────────────────────────────────────────────────
        structure_idx = candidate_idx[has_lidar]
        labels[structure_idx] = LABEL_STRUCTURE

        n_struct = int(has_lidar.sum())
        self.get_logger().info(
            f'Classified {n} voxels → '
            f'{n_struct} STRUCTURE / {n - n_struct} SEABED | '
            f'I_avg_seabed={I_avg_seabed:.1f}  I_thr={I_threshold:.1f}  '
            f'lambda_s={self._lambda_s}  epsilon={self._epsilon}m',
            throttle_duration_sec=2.0)

        return labels

    # ── publisher ──────────────────────────────────────────────────────────────

    def _publish(self):
        if self._sonar_pts is None:
            return

        sonar  = self._sonar_pts                    # (N, 6)
        labels = self._classify(sonar)              # (N,)

        out = np.column_stack([sonar, labels]).astype(np.float32)  # (N, 7)

        header          = Header()
        header.frame_id = self._map_frame
        header.stamp    = self.get_clock().now().to_msg()

        self._pub.publish(pc2.create_cloud(header, self._fields_out, out))


# ── entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
