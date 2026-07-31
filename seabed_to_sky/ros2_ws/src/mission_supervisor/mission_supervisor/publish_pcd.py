#!/usr/bin/env python3
"""
publish_pcd.py
==============
Publish a saved PCD (e.g. mission/fused/fused_all.pcd) as a LATCHED PointCloud2 so
Foxglove -- via the running foxglove_bridge -- can display it, even though the mission
nodes are stopped. The cloud carries x,y,z plus every scalar field found in the file
(intensity, and 'source' = 0:lidar / 1:sonar for the combined cloud), so in Foxglove's
3D panel you can set "Color mode -> Color field" to source / intensity / z and pick a
colormap (the "custom colors" workflow).

  ros2 run mission_supervisor publish_pcd --pcd mission/fused/fused_all.pcd
  # then in Foxglove: 3D panel -> enable /fused_map -> Color field = source (or intensity/z)

Latched (transient_local) so Foxglove gets it even if it connects after launch. Big
clouds are heavy over the bridge -- fuse with --voxel 0.2 first if it's sluggish.
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def _read_pcd(path):
    """-> (names list, data Nx F float32 C-contiguous). x,y,z first, then scalar fields."""
    import open3d as o3d
    pc = o3d.t.io.read_point_cloud(path)
    a = pc.point
    xyz = a['positions'].numpy().astype(np.float32)
    names = ['x', 'y', 'z']
    cols = [xyz[:, 0], xyz[:, 1], xyz[:, 2]]
    for name in ('intensity', 'source'):
        if name in a:
            names.append(name)
            cols.append(a[name].numpy().reshape(-1).astype(np.float32))
    data = np.ascontiguousarray(np.stack(cols, axis=1), dtype=np.float32)
    return names, data


class PcdPublisher(Node):
    def __init__(self, path, topic, frame, rate):
        super().__init__('pcd_publisher')
        names, data = _read_pcd(path)
        self._frame = frame
        self._n = data.shape[0]
        step = 4 * len(names)
        self._fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate(names)]
        self._step = step
        self._bytes = data.tobytes()
        # Latched: KEEP_LAST/1 + TRANSIENT_LOCAL so a late-joining Foxglove still receives it.
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PointCloud2, topic, qos)
        self.get_logger().info(
            f"publishing {self._n:,} pts on {topic} (frame={frame}) fields={names}")
        self._publish()
        if rate > 0:
            self.create_timer(1.0 / rate, self._publish)

    def _publish(self):
        msg = PointCloud2()
        msg.header.frame_id = self._frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = self._n
        msg.fields = self._fields
        msg.is_bigendian = False
        msg.point_step = self._step
        msg.row_step = self._step * self._n
        msg.is_dense = True
        msg.data = self._bytes
        self.pub.publish(msg)


def main(argv=None):
    pa = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    pa.add_argument('--pcd', required=True, help='path to the .pcd to publish')
    pa.add_argument('--topic', default='/fused_map')
    pa.add_argument('--frame', default='camera_init',
                    help="frame_id stamped on the cloud; set Foxglove's display frame to match")
    pa.add_argument('--rate', type=float, default=0.5,
                    help='republish Hz (latched, so 0 = publish once and rely on transient_local)')
    a = pa.parse_args(argv)
    rclpy.init()
    node = PcdPublisher(a.pcd, a.topic, a.frame, a.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
