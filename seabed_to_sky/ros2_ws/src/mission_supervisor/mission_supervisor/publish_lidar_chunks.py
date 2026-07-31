#!/usr/bin/env python3
"""
publish_lidar_chunks.py
=======================
Publish EACH saved lidar prune chunk as its own latched PointCloud2 topic
(/lidar/cNN_<hhmmss>) plus the concatenation on /lidar/all, so in Foxglove you can
toggle chunks on/off one by one and find the ones that don't belong (wrong session /
wrong frame / garbage). Each topic carries x,y,z,intensity.

  ros2 run mission_supervisor publish_lidar_chunks
  # Foxglove 3D panel: enable /lidar/all, then toggle each /lidar/cNN_* to find the bad one.

Latched (transient_local) so Foxglove receives each on subscribe. --voxel downsamples
each chunk for lighter viewing (shape is preserved). Prints index -> centroid so you
can map a misplaced topic back to its file.
"""
import argparse
import glob
import os
import re

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
           for i, n in enumerate(('x', 'y', 'z', 'intensity'))]


def _latched():
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _read(path, voxel):
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
        d = p.voxel_down_sample(voxel)
        xyz = d.point['positions'].numpy()
        inten = d.point['intensity'].numpy().reshape(-1)
    return xyz, inten


def _cloud(xyz, inten, frame, stamp):
    data = np.empty((xyz.shape[0], 4), np.float32)
    data[:, :3] = xyz
    data[:, 3] = inten
    m = PointCloud2()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.height = 1
    m.width = xyz.shape[0]
    m.fields = _FIELDS
    m.is_bigendian = False
    m.point_step = 16
    m.row_step = 16 * xyz.shape[0]
    m.is_dense = True
    m.data = data.tobytes()
    return m


class ChunkPublisher(Node):
    def __init__(self, lidar_dir, prefix, frame, voxel):
        super().__init__('lidar_chunk_publisher')
        files = sorted(glob.glob(os.path.join(lidar_dir, '*.pcd')))
        if not files:
            self.get_logger().error(f'no *.pcd in {lidar_dir}')
            return
        self._keep = []                       # hold pubs alive
        stamp = self.get_clock().now().to_msg()
        allx, alli = [], []
        self.get_logger().info(f'publishing {len(files)} lidar chunks under {prefix}/ (frame={frame})')
        for i, f in enumerate(files):
            xyz, inten = _read(f, voxel)
            mts = re.search(r'(\d{6})\.pcd$', os.path.basename(f))
            tag = mts.group(1) if mts else f'{i:02d}'
            topic = f'{prefix}/c{i:02d}_{tag}'
            pub = self.create_publisher(PointCloud2, topic, _latched())
            pub.publish(_cloud(xyz, inten, frame, stamp))
            self._keep.append(pub)
            c = xyz.mean(0) if xyz.shape[0] else np.zeros(3)
            self.get_logger().info(
                f'  {topic:<24} {xyz.shape[0]:>8,} pts  centroid=({c[0]:7.0f},{c[1]:7.0f},{c[2]:6.0f})')
            allx.append(xyz)
            alli.append(inten)
        AX, AI = np.concatenate(allx), np.concatenate(alli)
        ap = self.create_publisher(PointCloud2, f'{prefix}/all', _latched())
        ap.publish(_cloud(AX, AI, frame, stamp))
        self._keep.append(ap)
        self.get_logger().info(
            f'  {prefix}/all              {AX.shape[0]:>8,} pts (all chunks)\n'
            f'done: {len(self._keep)} latched topics. In Foxglove enable them and toggle '
            f'/lidar/cNN_* to find the intruders.')


def main(argv=None):
    pa = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    pa.add_argument('--lidar-dir', default='/home/rosdev/ros2_ws/mission/lidar')
    pa.add_argument('--prefix', default='/lidar')
    pa.add_argument('--frame', default='camera_init')
    pa.add_argument('--voxel', type=float, default=0.1, help='per-chunk downsample (m); 0=full res')
    a = pa.parse_args(argv)
    rclpy.init()
    node = ChunkPublisher(a.lidar_dir, a.prefix, a.frame, a.voxel)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
