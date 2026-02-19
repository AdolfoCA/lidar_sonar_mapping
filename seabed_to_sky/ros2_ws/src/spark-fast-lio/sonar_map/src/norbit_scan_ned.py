#!/usr/bin/env python3
"""
Simple Sonar Map Node
Subscribes to:
  - /norbit/bathymetry/points, frame: norbit_link_sonar
  - Odometry (from FAST-LIO2, frame: odom -> os_imu)
Publishes:
  - Transformed point cloud in odom frame
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


class SonarScanNode(Node):
    
    def __init__(self):
        super().__init__('sonar_scan_ned')
        
        # Create subscriptions
        self.sonar_sub = self.create_subscription(
            PointCloud2,
            'sonar_cloud',  # From acoustic3d_edge, frame: norbit_link_sonar
            self.sonar_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry',  # From FAST-LIO2, frame: odom
            self.odom_callback,
            10
        )
        
        # Create publisher
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            'sonar_scan_ned',  # Output in odom frame
            10
        )
        
        # Store current odometry
        self.position = None
        self.orientation = None
        self.odom_frame_id = None
        
        # For logging
        self.first_odom = True
        self.first_cloud = True
        
        self.get_logger().info('Sonar Scan NED Node started')
        self.get_logger().info('Waiting for data from:')
        self.get_logger().info('  - /norbit/bathymetry/points (frame: norbit_link_sonar)')
        self.get_logger().info('  - odometry (frame: odom)')

    def odom_callback(self, msg: Odometry):
        """Store current pose from odometry"""
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.odom_frame_id = msg.header.frame_id  # This is the output frame
        
        if self.first_odom:
            self.get_logger().info(f'Odometry frame: {msg.header.frame_id} -> {msg.child_frame_id}')
            self.get_logger().info(f'Robot position: x={self.position.x:.2f}, y={self.position.y:.2f}, z={self.position.z:.2f}')
            self.first_odom = False

    def sonar_callback(self, msg: PointCloud2):
        """Transform sonar points using odometry"""
        
        if self.position is None or self.orientation is None:
            if self.first_cloud:
                self.get_logger().warn('No odometry data received yet. Waiting...')
                self.first_cloud = False
            return
        
        # Read points from sonar cloud
        try:
            points = pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'))
            points = np.array(list(points))
        except Exception as e:
            self.get_logger().error(f'Error reading points: {e}')
            return
        
        if len(points) == 0:
            return
        
        # Create transformation matrix from odometry
        # Odometry gives: odom -> os_imu transformation
        # Static transform: os_imu -> norbit_link_frame

        # Same orientation, sonar is 43 cm below IMU
        T_imu_sonar = np.eye(4)
        T_imu_sonar [2, 3] = -10  # z = -1 m (down in FLU)
        
        translation = np.array([
            self.position.x,
            self.position.y,
            self.position.z
        ])
        
        quat = np.array([
            self.orientation.x,
            self.orientation.y,
            self.orientation.z,
            self.orientation.w
        ])

        rotation_matrix = R.from_quat(quat).as_matrix()
        
        # Create 4x4 transformation matrix
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = translation
        
        # Transform points from sonar frame to odom frame
        xyz = np.column_stack((points['x'], points['y'], points['z']))
        ones = np.ones((xyz.shape[0], 1))
        xyz_homogeneous = np.hstack((xyz, ones))
        
        # Full chain: odom -> os_imu -> oculus
        T_odom_sonar = T @ T_imu_sonar

        # Transform points
        transformed_xyz = (T_odom_sonar @ xyz_homogeneous.T).T[:, :3]


        # Create output point cloud with same fields as input
        output_points = np.zeros(len(transformed_xyz), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32)
        ])
        
        output_points['x'] = transformed_xyz[:, 0]
        output_points['y'] = transformed_xyz[:, 1]
        output_points['z'] = transformed_xyz[:, 2]
        output_points['intensity'] = points['intensity']
        
        # Create output message
        header = Header()
        header.frame_id = self.odom_frame_id  # Output is in odometry frame
        header.stamp = msg.header.stamp
        
        output_msg = pc2.create_cloud(
            header,
            [
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
            ],
            output_points
        )
        
        self.cloud_pub.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SonarScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()