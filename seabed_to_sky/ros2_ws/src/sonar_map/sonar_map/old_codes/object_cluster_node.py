#!/usr/bin/env python3
"""
object_cluster_node.py
----------------------
Outlier rejection for STRUCTURE (1.0) and OBJECT (2.0) voxels using DBSCAN.

Subscribes to the full sonar map and, every time ``bundle_size`` new classified
voxels appear in the map, runs DBSCAN independently on each semantic class.
Clusters smaller than ``min_cluster_size`` are treated as outliers and dropped.
The cleaned clusters are published with an extra ``cluster_id`` field.

PARAMETERS
  input_topic                    str    Sonar map topic.               Default: sonar_map
  structure_topic                str    Structure clusters output.     Default: structure_clusters
  object_topic                   str    Object clusters output.        Default: object_clusters
  map_frame                      str    TF frame for published clouds. Default: odom
  bundle_size                    int    Re-run DBSCAN every N voxels.  Default: 200

  Two-stage DBSCAN per class:
    Stage 1 — outlier rejection (fine scale):
      Removes isolated voxels with no close neighbours.
      eps_outlier is small (a few voxel widths); only truly lone voxels are dropped.
    Stage 2 — cluster grouping (coarse scale):
      Groups surviving clean voxels into coherent structures/objects.
      eps_cluster is larger; connects fragments of the same physical feature.

  structure_outlier_eps          float  Stage-1 eps for structures [m].  Default: 0.15
  structure_outlier_min_samples  int    Stage-1 min_samples.             Default: 3
  structure_cluster_eps          float  Stage-2 eps for structures [m].  Default: 0.5
  structure_cluster_min_samples  int    Stage-2 min_samples.             Default: 5
  structure_min_cluster_size     int    Min voxels to keep a cluster.    Default: 20
  object_outlier_eps             float  Stage-1 eps for objects [m].     Default: 0.1
  object_outlier_min_samples     int    Stage-1 min_samples.             Default: 3
  object_cluster_eps             float  Stage-2 eps for objects [m].     Default: 0.2
  object_cluster_min_samples     int    Stage-2 min_samples.             Default: 5
  object_min_cluster_size        int    Min voxels to keep a cluster.    Default: 10

OUTPUT FIELDS (both topics)
  x, y, z, prob, intensity, semantic, cluster_id
  cluster_id — integer cluster index (0-based); outliers are not published.
"""

import numpy as np
import numpy.lib.recfunctions as rfn

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

from sklearn.cluster import DBSCAN


# ── Semantic labels ────────────────────────────────────────────────────────────
LABEL_STRUCTURE = np.float32(1.0)
LABEL_OBJECT    = np.float32(2.0)

# ── Defaults ───────────────────────────────────────────────────────────────────
INPUT_TOPIC        = 'sonar_map'
STRUCTURE_TOPIC    = 'structure_clusters'
OBJECT_TOPIC       = 'object_clusters'
MAP_FRAME          = 'odom'
BUNDLE_SIZE = 200

# Structures — stage 1: outlier rejection (fine scale, ~3 × voxel_size)
STRUCTURE_OUTLIER_EPS         = 0.15
STRUCTURE_OUTLIER_MIN_SAMPLES = 3
# Structures — stage 2: cluster grouping (coarse scale)
STRUCTURE_CLUSTER_EPS         = 0.5
STRUCTURE_CLUSTER_MIN_SAMPLES = 5
STRUCTURE_MIN_CLUSTER_SIZE    = 20

# Objects — stage 1: outlier rejection
OBJECT_OUTLIER_EPS            = 0.10
OBJECT_OUTLIER_MIN_SAMPLES    = 3
# Objects — stage 2: cluster grouping
OBJECT_CLUSTER_EPS            = 0.2
OBJECT_CLUSTER_MIN_SAMPLES    = 5
OBJECT_MIN_CLUSTER_SIZE       = 10


# ── Helpers ────────────────────────────────────────────────────────────────────

def _read_fields(msg: PointCloud2, fields: tuple) -> np.ndarray:
    """Read requested fields from a PointCloud2 into (N, len(fields)) float32."""
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

class ObjectClusterNode(Node):
    """
    Subscribes to sonar_map (fields: x, y, z, prob, intensity, source, semantic).
    Filters STRUCTURE (1.0) and OBJECT (2.0) voxels, runs DBSCAN on the full
    accumulated set of each class every ``bundle_size`` new classified voxels,
    and publishes cleaned point clouds with a ``cluster_id`` field.
    """

    def __init__(self):
        super().__init__('object_cluster_node')

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter('input_topic',                   INPUT_TOPIC)
        self.declare_parameter('structure_topic',               STRUCTURE_TOPIC)
        self.declare_parameter('object_topic',                  OBJECT_TOPIC)
        self.declare_parameter('map_frame',                     MAP_FRAME)
        self.declare_parameter('bundle_size',                   BUNDLE_SIZE)
        self.declare_parameter('structure_outlier_eps',         STRUCTURE_OUTLIER_EPS)
        self.declare_parameter('structure_outlier_min_samples', STRUCTURE_OUTLIER_MIN_SAMPLES)
        self.declare_parameter('structure_cluster_eps',         STRUCTURE_CLUSTER_EPS)
        self.declare_parameter('structure_cluster_min_samples', STRUCTURE_CLUSTER_MIN_SAMPLES)
        self.declare_parameter('structure_min_cluster_size',    STRUCTURE_MIN_CLUSTER_SIZE)
        self.declare_parameter('object_outlier_eps',            OBJECT_OUTLIER_EPS)
        self.declare_parameter('object_outlier_min_samples',    OBJECT_OUTLIER_MIN_SAMPLES)
        self.declare_parameter('object_cluster_eps',            OBJECT_CLUSTER_EPS)
        self.declare_parameter('object_cluster_min_samples',    OBJECT_CLUSTER_MIN_SAMPLES)
        self.declare_parameter('object_min_cluster_size',       OBJECT_MIN_CLUSTER_SIZE)

        def gp(n): return self.get_parameter(n).value

        self._map_frame   = str(gp('map_frame'))
        self._bundle_size = int(gp('bundle_size'))

        self._struct_outlier_eps      = float(gp('structure_outlier_eps'))
        self._struct_outlier_min_samp = int(gp('structure_outlier_min_samples'))
        self._struct_cluster_eps      = float(gp('structure_cluster_eps'))
        self._struct_cluster_min_samp = int(gp('structure_cluster_min_samples'))
        self._struct_min_cluster      = int(gp('structure_min_cluster_size'))

        self._obj_outlier_eps         = float(gp('object_outlier_eps'))
        self._obj_outlier_min_samp    = int(gp('object_outlier_min_samples'))
        self._obj_cluster_eps         = float(gp('object_cluster_eps'))
        self._obj_cluster_min_samp    = int(gp('object_cluster_min_samples'))
        self._obj_min_cluster         = int(gp('object_min_cluster_size'))

        input_topic     = str(gp('input_topic'))
        structure_topic = str(gp('structure_topic'))
        object_topic    = str(gp('object_topic'))

        # ── internal state ────────────────────────────────────────────────
        # Track count of classified voxels at the last DBSCAN run so we
        # trigger again only after bundle_size new ones have been added.
        self._last_classified_count = 0

        # ── output PointField definitions ─────────────────────────────────
        # sonar_map fields minus 'source', plus 'cluster_id', 'alpha', 'beta'
        self._fields_out = [
            PointField(name='x',          offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',          offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',          offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='prob',       offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity',  offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='semantic',   offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='cluster_id', offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name='alpha',      offset=28, datatype=PointField.FLOAT32, count=1),
            PointField(name='beta',       offset=32, datatype=PointField.FLOAT32, count=1),
        ]

        # ── ROS2 I/O ──────────────────────────────────────────────────────
        self._pub_structure = self.create_publisher(PointCloud2, structure_topic, 10)
        self._pub_object    = self.create_publisher(PointCloud2, object_topic,    10)

        self.create_subscription(PointCloud2, input_topic, self._sonar_cb, 1)

        self.get_logger().info(
            f'ObjectClusterNode ready\n'
            f'  input        : {input_topic}\n'
            f'  structure out: {structure_topic}\n'
            f'  object out   : {object_topic}\n'
            f'  bundle_size  : {self._bundle_size} voxels\n'
            f'  STRUCTURE  outlier eps={self._struct_outlier_eps} m '
            f'min_samp={self._struct_outlier_min_samp} | '
            f'cluster eps={self._struct_cluster_eps} m '
            f'min_samp={self._struct_cluster_min_samp} '
            f'min_size={self._struct_min_cluster}\n'
            f'  OBJECT     outlier eps={self._obj_outlier_eps} m '
            f'min_samp={self._obj_outlier_min_samp} | '
            f'cluster eps={self._obj_cluster_eps} m '
            f'min_samp={self._obj_cluster_min_samp} '
            f'min_size={self._obj_min_cluster}'
        )

    # ── callback ───────────────────────────────────────────────────────────────

    def _sonar_cb(self, msg: PointCloud2):
        # sonar_map fields: x, y, z, prob, intensity, source, semantic, alpha, beta
        pts = _read_fields(msg, ('x', 'y', 'z', 'prob', 'intensity', 'source', 'semantic', 'alpha', 'beta'))
        if pts.shape[0] == 0:
            return

        # col indices: 0=x 1=y 2=z 3=prob 4=intensity 5=source 6=semantic 7=alpha 8=beta
        semantic = pts[:, 6]
        classified_mask = (semantic == LABEL_STRUCTURE) | (semantic == LABEL_OBJECT)
        n_classified = int(classified_mask.sum())

        # Only re-run when bundle_size new classified voxels have accumulated
        if n_classified - self._last_classified_count < self._bundle_size:
            return

        self._last_classified_count = n_classified
        classified = pts[classified_mask]  # (M, 7): x,y,z,prob,intensity,source,semantic

        self.get_logger().info(
            f'DBSCAN triggered: {n_classified} classified voxels '
            f'({int((semantic == LABEL_STRUCTURE).sum())} structure, '
            f'{int((semantic == LABEL_OBJECT).sum())} object)'
        )

        for label, pub, outlier_eps, outlier_min_samp, cluster_eps, cluster_min_samp, min_clust in [
            (LABEL_STRUCTURE, self._pub_structure,
             self._struct_outlier_eps, self._struct_outlier_min_samp,
             self._struct_cluster_eps, self._struct_cluster_min_samp,
             self._struct_min_cluster),
            (LABEL_OBJECT, self._pub_object,
             self._obj_outlier_eps, self._obj_outlier_min_samp,
             self._obj_cluster_eps, self._obj_cluster_min_samp,
             self._obj_min_cluster),
        ]:
            mask  = classified[:, 6] == label
            cloud = classified[mask]
            if cloud.shape[0] < outlier_min_samp:
                continue
            self._cluster_and_publish(
                cloud, label, pub, msg.header.stamp,
                outlier_eps, outlier_min_samp,
                cluster_eps, cluster_min_samp, min_clust)

    # ── clustering ─────────────────────────────────────────────────────────────

    def _cluster_and_publish(
            self,
            cloud:            np.ndarray,
            label:            float,
            pub,
            stamp,
            outlier_eps:      float,
            outlier_min_samp: int,
            cluster_eps:      float,
            cluster_min_samp: int,
            min_cluster:      int,
    ):
        """
        Two-stage DBSCAN: outlier rejection then cluster grouping.

        Stage 1 — outlier rejection (fine scale):
          Run DBSCAN with ``outlier_eps`` (small, ~3× voxel_size).
          Any voxel labelled noise (-1) has no close neighbours → true outlier,
          dropped immediately.  Voxels that survive belong to at least one
          tight local neighbourhood.

        Stage 2 — cluster grouping (coarse scale):
          Run DBSCAN again on the survivors with ``cluster_eps`` (larger).
          This connects fragments of the same physical feature into one cluster.
          Clusters smaller than ``min_cluster`` are dropped as residual noise.

        Parameters
        ----------
        cloud            : (N, 7) float32  [x,y,z,prob,intensity,source,semantic]
        label            : semantic label of the class being processed
        pub              : ROS2 publisher for the output cloud
        stamp            : message timestamp to forward
        outlier_eps      : Stage-1 eps [m]
        outlier_min_samp : Stage-1 min_samples
        cluster_eps      : Stage-2 eps [m]
        cluster_min_samp : Stage-2 min_samples
        min_cluster      : minimum voxels to keep a final cluster
        """
        label_name = 'STRUCTURE' if label == LABEL_STRUCTURE else 'OBJECT'
        xyz = cloud[:, :3].astype(np.float64)

        # ── Stage 1: outlier rejection ─────────────────────────────────────
        stage1_labels = DBSCAN(
            eps=outlier_eps,
            min_samples=outlier_min_samp,
            n_jobs=-1,
        ).fit_predict(xyz)

        inlier_mask = stage1_labels >= 0
        n_outliers  = int((~inlier_mask).sum())

        if not inlier_mask.any():
            self.get_logger().warn(
                f'{label_name}: all {cloud.shape[0]} voxels rejected as outliers '
                f'(outlier_eps={outlier_eps} m)',
                throttle_duration_sec=5.0,
            )
            return

        clean_cloud = cloud[inlier_mask]
        xyz_clean   = xyz[inlier_mask]

        # ── Stage 2: cluster grouping ──────────────────────────────────────
        stage2_labels = DBSCAN(
            eps=cluster_eps,
            min_samples=cluster_min_samp,
            n_jobs=-1,
        ).fit_predict(xyz_clean)

        unique_ids, counts = np.unique(
            stage2_labels[stage2_labels >= 0], return_counts=True)
        valid_ids = unique_ids[counts >= min_cluster]

        if valid_ids.size == 0:
            self.get_logger().warn(
                f'{label_name}: no clusters survived after grouping '
                f'(n_clean={clean_cloud.shape[0]}, '
                f'cluster_eps={cluster_eps} m, min_cluster={min_cluster})',
                throttle_duration_sec=5.0,
            )
            return

        final_mask  = np.isin(stage2_labels, valid_ids)
        inliers     = clean_cloud[final_mask]
        cluster_ids = stage2_labels[final_mask].astype(np.float32)

        self.get_logger().info(
            f'{label_name}: {n_outliers} outliers removed (stage 1) | '
            f'{clean_cloud.shape[0] - inliers.shape[0]} residual noise (stage 2) | '
            f'{inliers.shape[0]} inliers in {valid_ids.size} clusters'
        )

        # Output columns: x, y, z, prob, intensity, semantic, cluster_id, alpha, beta
        out = np.column_stack([
            inliers[:, 0],   # x
            inliers[:, 1],   # y
            inliers[:, 2],   # z
            inliers[:, 3],   # prob
            inliers[:, 4],   # intensity
            inliers[:, 6],   # semantic  (skip source at index 5)
            cluster_ids,
            inliers[:, 7],   # alpha
            inliers[:, 8],   # beta
        ]).astype(np.float32)

        header          = Header()
        header.frame_id = self._map_frame
        header.stamp    = stamp

        pub.publish(pc2.create_cloud(header, self._fields_out, out))

        n_rejected = cloud.shape[0] - inliers.shape[0]
        self.get_logger().info(
            f'{label_name}: {inliers.shape[0]} inliers in {valid_ids.size} clusters | '
            f'{n_rejected} outliers rejected | '
            f'cluster sizes: {counts[np.isin(unique_ids, valid_ids)].tolist()}'
        )


# ── entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ObjectClusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
