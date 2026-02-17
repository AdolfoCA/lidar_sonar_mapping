#!/usr/bin/env python3
"""
Simple Sonar Accumulator Node
Subscribes to:
  - Transformed sonar point cloud (from sonar_scan_NED)
Publishes:
  - Accumulated point cloud map
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


class SonarMapNode(Node):
    
    def __init__(self):
        super().__init__('sonar_map_NED')
        
        # Create subscription
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            'sonar_map_cloud',  # From sonar_scan_NED
            self.cloud_callback,
            10
        )
        
        # Create publisher
        self.map_pub = self.create_publisher(
            PointCloud2,
            'sonar_map',  # Accumulated map
            10
        )
        
        # Timer to publish accumulated map every second
        self.create_timer(1.0, self.publish_map)
        
        # Accumulated points
        self.accumulated_points = None
        self.point_count = 0
        self.max_points = 10000000
        self.frame_id = 'odom'  # Will be updated from first cloud
        
        self.get_logger().info('Sonar Map NED node started')
        self.get_logger().info(f'Max points: {self.max_points}')

    def cloud_callback(self, msg: PointCloud2):
        """Accumulate incoming points"""
        
        # Get frame_id from first message
        if self.frame_id == 'odom' and msg.header.frame_id != 'odom':
            self.frame_id = msg.header.frame_id
            self.get_logger().info(f'Frame ID: {self.frame_id}')
        
        # Read points
        try:
            points = pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'))
            points = np.array(list(points))
        except Exception as e:
            self.get_logger().error(f'Error reading points: {e}')
            return
        
        if len(points) == 0:
            return
        
        # Accumulate
        if self.accumulated_points is None:
            self.accumulated_points = points.copy()
        else:
            self.accumulated_points = np.concatenate(
                (self.accumulated_points, points),
                axis=0
            )
        
        # Limit to max points
        if len(self.accumulated_points) > self.max_points:
            self.accumulated_points = self.accumulated_points[-self.max_points:]
            self.get_logger().info(f'Reached max points ({self.max_points}), keeping latest')
        
        self.point_count = len(self.accumulated_points)

    def publish_map(self):
        """Publish accumulated map once per second"""
        
        if self.accumulated_points is None or len(self.accumulated_points) == 0:
            return
        
        # Create message
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()
        
        output_msg = pc2.create_cloud(
            header,
            [
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
            ],
            self.accumulated_points
        )
        
        self.map_pub.publish(output_msg)
        self.get_logger().debug(f'Published map with {self.point_count} points')


def main(args=None):
    rclpy.init(args=args)
    node = SonarMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()