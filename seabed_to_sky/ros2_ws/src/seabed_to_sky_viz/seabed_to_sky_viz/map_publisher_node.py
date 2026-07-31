#!/usr/bin/env python3
"""
seabed_to_sky_viz — publish saved mission lidar + sonar point clouds for Foxglove.

Reads the per-prune PCD chunks written during a mission (mission/lidar/*.pcd and
mission/sonar/*.pcd) and publishes them as LATCHED PointCloud2 topics:

  separately :  <ns>/lidar/cNN_<hhmmss>   each lidar chunk
                <ns>/sonar/cNN_<hhmmss>   each sonar chunk
  per sensor :  <ns>/lidar_all            all NON-excluded lidar chunks fused
                <ns>/sonar_all            all NON-excluded sonar chunks fused
  together   :  <fused_topic>             lidar + sonar fused (NON-excluded),
                                          'source' field = 0:lidar 1:sonar

The `exclude` list in the config selects which chunks NOT to show in the fused map
(and the per-sensor *_all topics): any chunk file whose name contains one of the
substrings (e.g. a timestamp "183549") is left out. The chunk still gets its own
per-chunk topic so you can inspect it. `exclude` is live: `ros2 param set
/seabed_to_sky_viz exclude "['183549','183750']"` rebuilds + republishes the
aggregated clouds with no relaunch.

In Foxglove: 3D panel, Fixed frame = <frame_id> (default camera_init), enable the
topics, and "Color by field" -> source / intensity / z.

NOTE: reads any PCD encoding (ascii / binary / binary_compressed) via open3d (pip).
"""
import glob
import os
import re

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2, PointField


def _latched():
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _read(path, voxel):
    """PCD (any encoding) -> (xyz Nx3 f32, intensity N f32), finite only, optional voxel."""
    import open3d as o3d
    a = o3d.t.io.read_point_cloud(path).point
    xyz = a['positions'].numpy().astype(np.float32)
    inten = (a['intensity'].numpy().reshape(-1).astype(np.float32)
             if 'intensity' in a else np.zeros(xyz.shape[0], np.float32))
    m = np.isfinite(xyz).all(1)
    xyz, inten = xyz[m], inten[m]
    if voxel > 0 and xyz.shape[0]:
        p = o3d.t.geometry.PointCloud()
        p.point['positions'] = o3d.core.Tensor(xyz)
        p.point['intensity'] = o3d.core.Tensor(inten.reshape(-1, 1))
        d = p.voxel_down_sample(float(voxel))
        xyz = d.point['positions'].numpy()
        inten = d.point['intensity'].numpy().reshape(-1)
    return xyz, inten


def _cloud(xyz, inten, frame, stamp, source=None):
    names = ['x', 'y', 'z', 'intensity'] + (['source'] if source is not None else [])
    nf = len(names)
    data = np.empty((xyz.shape[0], nf), np.float32)
    data[:, 0:3] = xyz
    data[:, 3] = inten
    if source is not None:
        data[:, 4] = source
    m = PointCloud2()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.height = 1
    m.width = xyz.shape[0]
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate(names)]
    m.is_bigendian = False
    m.point_step = 4 * nf
    m.row_step = 4 * nf * xyz.shape[0]
    m.is_dense = True
    m.data = data.tobytes()
    return m


class SeabedToSkyViz(Node):
    def __init__(self):
        super().__init__('seabed_to_sky_viz')
        p = self.declare_parameter
        self.lidar_dir = str(p('lidar_dir', '/home/rosdev/ros2_ws/mission/lidar').value)
        self.sonar_dir = str(p('sonar_dir', '/home/rosdev/ros2_ws/mission/sonar').value)
        self.frame = str(p('frame_id', 'camera_init').value)
        self.voxel = float(p('voxel_size', 0.1).value)
        self.publish_chunks = bool(p('publish_chunks', True).value)
        self.fused_topic = str(p('fused_topic', '/seabed_to_sky_map').value)
        self.ns = str(p('namespace', '/seabed_to_sky').value).rstrip('/')
        # Default a 1-element string list so rclpy infers STRING_ARRAY (an empty []
        # default is inferred as BYTE_ARRAY and conflicts). Empty entries are ignored.
        self.exclude = [s for s in (p('exclude', ['']).value or []) if s]

        self._stamp = self.get_clock().now().to_msg()
        self._chunk_pubs = []

        self.get_logger().info(
            f"loading chunks: lidar={self.lidar_dir} sonar={self.sonar_dir} "
            f"voxel={self.voxel:g} publish_chunks={self.publish_chunks}")
        self._lidar = self._load(self.lidar_dir, 'lidar')   # [(name, xyz, inten), ...]
        self._sonar = self._load(self.sonar_dir, 'sonar')

        self._lidar_all_pub = self.create_publisher(PointCloud2, f'{self.ns}/lidar_all', _latched())
        self._sonar_all_pub = self.create_publisher(PointCloud2, f'{self.ns}/sonar_all', _latched())
        self._fused_pub = self.create_publisher(PointCloud2, self.fused_topic, _latched())
        self._rebuild_aggregated()

        self.add_on_set_parameters_callback(self._on_set_params)
        self.get_logger().info(
            f"seabed_to_sky_viz ready: lidar={len(self._lidar)} sonar={len(self._sonar)} chunks | "
            f"excluded={self.exclude} | fused -> {self.fused_topic} | frame={self.frame}")

    def _load(self, d, kind):
        out = []
        files = sorted(glob.glob(os.path.join(d, '*.pcd')))
        for i, f in enumerate(files):
            try:
                xyz, inten = _read(f, self.voxel)
            except Exception as exc:                       # noqa: BLE001
                self.get_logger().warn(f"skip {os.path.basename(f)}: {exc!r}")
                continue
            name = os.path.basename(f)
            out.append((name, xyz, inten))
            if self.publish_chunks:
                mts = re.search(r'(\d{6})\.pcd$', name)
                tag = mts.group(1) if mts else f'{i:02d}'
                pub = self.create_publisher(PointCloud2, f'{self.ns}/{kind}/c{i:02d}_{tag}', _latched())
                pub.publish(_cloud(xyz, inten, self.frame, self._stamp))
                self._chunk_pubs.append(pub)
        return out

    def _excluded(self, name):
        return any(s and s in name for s in self.exclude)

    def _agg(self, chunks):
        keep = [(x, c) for (n, x, c) in chunks if x.shape[0] and not self._excluded(n)]
        if not keep:
            return np.zeros((0, 3), np.float32), np.zeros((0,), np.float32)
        return np.concatenate([x for x, _ in keep]), np.concatenate([c for _, c in keep])

    def _rebuild_aggregated(self):
        lx, li = self._agg(self._lidar)
        sx, si = self._agg(self._sonar)
        self._lidar_all_pub.publish(_cloud(lx, li, self.frame, self._stamp))
        self._sonar_all_pub.publish(_cloud(sx, si, self.frame, self._stamp))
        if lx.shape[0] and sx.shape[0]:
            ax, ai = np.concatenate([lx, sx]), np.concatenate([li, si])
        else:
            ax, ai = (lx, li) if lx.shape[0] else (sx, si)
        src = np.concatenate([np.zeros(lx.shape[0], np.float32),
                              np.ones(sx.shape[0], np.float32)])
        self._fused_pub.publish(_cloud(ax, ai, self.frame, self._stamp, source=src))
        self.get_logger().info(
            f"aggregated: lidar_all={lx.shape[0]:,}  sonar_all={sx.shape[0]:,}  "
            f"{self.fused_topic}={ax.shape[0]:,}  (excluded={self.exclude})")

    def _on_set_params(self, params):
        for prm in params:
            if prm.name == 'exclude':
                self.exclude = [s for s in (prm.value or []) if s]
                self._rebuild_aggregated()
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = SeabedToSkyViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
