#!/usr/bin/env python3
"""
sdf_fitting_node.py
-------------------
Runs per-cluster SDF fitting on STRUCTURE and OBJECT voxel clouds and
republishes with an additional `sdf` field.  The underlying sonar map
and Beta-Bernoulli probabilities are never modified.

For each cluster received from object_cluster_node this node:
  1. Groups voxels by cluster_id; skips clusters below min_cluster_voxels.
  2. Solves  (W + λL) φ = W d  where
       d_i = trunc · (1 − 2·p_i)           (signed distance guess)
       w_i = p_i · (α_i + β_i)             (observation-count weight)
       L   = graph Laplacian of the voxel neighbourhood graph
  3. Publishes the cluster with the optimised φ appended as `sdf`.
     φ ≈ 0 marks the reconstructed surface; φ < 0 is interior.
     Original Beta-Bernoulli probs are unchanged.

PARAMETERS
  structure_input         str    Default: structure_clusters
  object_input            str    Default: object_clusters
  structure_refined_topic str    Default: structure_refined
  object_refined_topic    str    Default: object_refined
  map_frame               str    Default: odom
  voxel_size              float  Must match sonar_map_ned [m].  Default: 0.05
  truncation_distance     float  SDF clamp distance [m].        Default: 0.15
  smoothness_weight       float  λ — Laplacian smoothness weight. Default: 0.5
  min_cluster_voxels      int    Default: 10

OUTPUT FIELDS (both topics)
  x, y, z, prob, intensity, semantic, cluster_id, alpha, beta, sdf
  prob — original Beta-Bernoulli occupancy probability, unchanged.
  sdf  — optimised signed distance estimate per voxel (φ ≈ 0 = surface).
"""

import numpy as np
import numpy.lib.recfunctions as rfn

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

from scipy.sparse import diags, csr_matrix
from scipy.sparse.linalg import cg
from scipy.spatial import cKDTree


# ── Defaults ───────────────────────────────────────────────────────────────────
STRUCTURE_INPUT         = 'structure_clusters'
OBJECT_INPUT            = 'object_clusters'
STRUCTURE_REFINED_TOPIC = 'structure_refined'
OBJECT_REFINED_TOPIC    = 'object_refined'
MAP_FRAME               = 'odom'
VOXEL_SIZE              = 0.05
TRUNCATION_DISTANCE     = 0.15
SMOOTHNESS_WEIGHT       = 0.5
MIN_CLUSTER_VOXELS      = 10


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

class SdfFittingNode(Node):

    def __init__(self):
        super().__init__('sdf_fitting_node')

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter('structure_input',         STRUCTURE_INPUT)
        self.declare_parameter('object_input',            OBJECT_INPUT)
        self.declare_parameter('structure_refined_topic', STRUCTURE_REFINED_TOPIC)
        self.declare_parameter('object_refined_topic',    OBJECT_REFINED_TOPIC)
        self.declare_parameter('map_frame',               MAP_FRAME)
        self.declare_parameter('voxel_size',              VOXEL_SIZE)
        self.declare_parameter('truncation_distance',     TRUNCATION_DISTANCE)
        self.declare_parameter('smoothness_weight',       SMOOTHNESS_WEIGHT)
        self.declare_parameter('min_cluster_voxels',      MIN_CLUSTER_VOXELS)

        def gp(n): return self.get_parameter(n).value

        self._map_frame  = str(gp('map_frame'))
        self._voxel_size = float(gp('voxel_size'))
        self._trunc      = float(gp('truncation_distance'))
        self._lambda     = float(gp('smoothness_weight'))
        self._min_voxels = int(gp('min_cluster_voxels'))

        struct_in  = str(gp('structure_input'))
        obj_in     = str(gp('object_input'))
        struct_ref = str(gp('structure_refined_topic'))
        obj_ref    = str(gp('object_refined_topic'))

        # ── output cloud fields ────────────────────────────────────────────
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
            PointField(name='sdf',        offset=36, datatype=PointField.FLOAT32, count=1),
        ]

        # ── publishers ────────────────────────────────────────────────────
        self._pub_struct_ref = self.create_publisher(PointCloud2, struct_ref, 10)
        self._pub_obj_ref    = self.create_publisher(PointCloud2, obj_ref,    10)

        # Latest-message buffers
        self._latest_struct: PointCloud2 | None = None
        self._latest_obj:    PointCloud2 | None = None
        self._processing_struct = False
        self._processing_obj    = False

        # ── subscribers (queue depth 1) ────────────────────────────────────
        self.create_subscription(
            PointCloud2, struct_in,
            lambda msg: self._enqueue(msg, 'structure'), 1)
        self.create_subscription(
            PointCloud2, obj_in,
            lambda msg: self._enqueue(msg, 'object'), 1)

        self.create_timer(0.5, self._drain)

        self.get_logger().info(
            f'SdfFittingNode ready\n'
            f'  structure in : {struct_in}  →  {struct_ref}\n'
            f'  object in    : {obj_in}  →  {obj_ref}\n'
            f'  min_voxels   : {self._min_voxels}'
        )

    # ── queueing ───────────────────────────────────────────────────────────────

    def _enqueue(self, msg: PointCloud2, label_name: str):
        if label_name == 'structure':
            self._latest_struct = msg
        else:
            self._latest_obj = msg

    def _drain(self):
        if self._latest_struct is not None and not self._processing_struct:
            msg = self._latest_struct
            self._latest_struct     = None
            self._processing_struct = True
            try:
                self._cb(msg, 'structure', self._pub_struct_ref)
            finally:
                self._processing_struct = False

        if self._latest_obj is not None and not self._processing_obj:
            msg = self._latest_obj
            self._latest_obj     = None
            self._processing_obj = True
            try:
                self._cb(msg, 'object', self._pub_obj_ref)
            finally:
                self._processing_obj = False

    # ── main callback ──────────────────────────────────────────────────────────

    def _cb(self, msg: PointCloud2, label_name: str, refined_pub):
        # col indices: 0=x 1=y 2=z 3=prob 4=intensity 5=semantic
        #              6=cluster_id 7=alpha 8=beta
        pts = _read_fields(
            msg, ('x', 'y', 'z', 'prob', 'intensity', 'semantic',
                  'cluster_id', 'alpha', 'beta'))
        if pts.shape[0] == 0:
            return

        cluster_ids = np.unique(pts[:, 6].astype(np.int32))
        kept      = []
        n_skipped = 0

        for cid in cluster_ids:
            cluster = pts[pts[:, 6].astype(np.int32) == cid]
            if cluster.shape[0] < self._min_voxels:
                n_skipped += 1
                continue

            phi = self._fit_sdf(
                xyz=cluster[:, :3],
                probs=cluster[:, 3],
                alpha=cluster[:, 7],
                beta=cluster[:, 8],
                cluster_id=int(cid),
                label_name=label_name,
            )
            kept.append(np.column_stack([cluster, phi.astype(np.float32)]))

        if not kept:
            return

        out = np.vstack(kept).astype(np.float32)
        header = Header()
        header.frame_id = self._map_frame
        header.stamp    = msg.header.stamp
        refined_pub.publish(pc2.create_cloud(header, self._fields_out, out))

        self.get_logger().info(
            f'{label_name.upper()}: published {out.shape[0]} voxels in '
            f'{len(kept)} clusters '
            f'({n_skipped} clusters below min_voxels={self._min_voxels} skipped)',
            throttle_duration_sec=2.0,
        )

    # ── SDF fitting ────────────────────────────────────────────────────────────

    def _fit_sdf(
            self,
            xyz:   np.ndarray,
            probs: np.ndarray,
            alpha: np.ndarray,
            beta:  np.ndarray,
            cluster_id: int,
            label_name: str,
    ) -> np.ndarray:
        """
        Solve  (W + λL) φ = W d

          d_i = trunc · (1 − 2·p_i)      signed-distance initial guess
          w_i = p_i · (α_i + β_i)        observation-count weight
          L   = graph Laplacian (voxel neighbourhood graph)

        Returns φ per voxel.  φ ≈ 0 → surface, φ < 0 → interior.
        Falls back to d on solver failure or isolated voxels.
        """
        N = xyz.shape[0]
        d = self._trunc * (1.0 - 2.0 * probs)
        w = probs * (alpha + beta)
        w = np.maximum(w, 1e-3)

        tree  = cKDTree(xyz)
        pairs = list(tree.query_pairs(r=1.5 * self._voxel_size))

        if not pairs:
            return d

        n_pairs = len(pairs)
        row = np.empty(2 * n_pairs + N, dtype=np.int32)
        col = np.empty(2 * n_pairs + N, dtype=np.int32)
        val = np.empty(2 * n_pairs + N, dtype=np.float64)
        deg = np.zeros(N, dtype=np.float64)

        for k, (i, j) in enumerate(pairs):
            row[2 * k]     = i;  col[2 * k]     = j;  val[2 * k]     = -1.0
            row[2 * k + 1] = j;  col[2 * k + 1] = i;  val[2 * k + 1] = -1.0
            deg[i] += 1.0
            deg[j] += 1.0

        row[2 * n_pairs:] = np.arange(N)
        col[2 * n_pairs:] = np.arange(N)
        val[2 * n_pairs:] = deg

        L = csr_matrix((val, (row, col)), shape=(N, N))
        W = diags(w)
        A = W + self._lambda * L
        b = np.asarray(W.dot(d)).flatten()

        phi, info = cg(A, b, rtol=1e-6, maxiter=500)
        if info != 0:
            return d
        return phi


# ── entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SdfFittingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
