#!/usr/bin/env python3
"""
Save Map Server - Saves PCD files with initial pose from LiDAR odometry
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from pathlib import Path
from datetime import datetime
import json


class SaveMapServer(Node):
    
    def __init__(self):
        super().__init__('save_map')
        
        # Point cloud subscription
        self.sonar_sub = self.create_subscription(
            PointCloud2,
            '/sonar_map',
            self.sonar_callback,
            10
        )
        
        # LiDAR odometry from SPARK-FastLIO2
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry',
            self.odom_callback,
            10
        )
        
        self.save_service = self.create_service(
            Trigger,
            '/save_map',
            self.save_map_callback
        )
        
        # Data storage
        self.sonar_cloud = None
        self.initial_pose = None
        self.current_pose = None
        self.init_captured = False
        
        self.output_dir = Path.home() / 'ros2_ws' / 'saved_maps'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.get_logger().info('='*50)
        self.get_logger().info('Save Map Server started')
        self.get_logger().info('='*50)
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info('Listening for /odometry and /sonar_map')

    def odom_callback(self, msg: Odometry):
        """Store odometry - capture FIRST pose as initial"""
        pose = msg.pose.pose
        current_pose = {
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            'qx': pose.orientation.x,
            'qy': pose.orientation.y,
            'qz': pose.orientation.z,
            'qw': pose.orientation.w,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }
        
        # Capture initial pose only once (first message)
        if not self.init_captured:
            self.initial_pose = current_pose.copy()
            self.init_captured = True
            self.get_logger().info(
                f'✅ Initial pose captured: '
                f'[{current_pose["x"]:.3f}, {current_pose["y"]:.3f}, {current_pose["z"]:.3f}]'
            )
        
        self.current_pose = current_pose

    def sonar_callback(self, msg: PointCloud2):
        """Store latest sonar cloud"""
        try:
            points = pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'))
            self.sonar_cloud = np.array(list(points))
        except Exception as e:
            self.get_logger().error(f'Error reading sonar cloud: {e}')

    def save_map_callback(self, request, response):
        """Save sonar map, FAST-LIO2 KD-tree, and pose metadata"""
        
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            saved_files = []
            
            self.get_logger().info('='*50)
            self.get_logger().info('SAVING MAPS')
            self.get_logger().info('='*50)
            
            if self.initial_pose:
                self.get_logger().info(
                    f'Initial pose: [{self.initial_pose["x"]:.3f}, '
                    f'{self.initial_pose["y"]:.3f}, {self.initial_pose["z"]:.3f}]'
                )
            else:
                self.get_logger().warn('Initial pose NOT captured!')
            
            # Save pose metadata
            metadata = {
                'timestamp': timestamp,
                'initial_pose': self.initial_pose,
                'final_pose': self.current_pose,
            }
            
            metadata_file = self.output_dir / f'metadata_{timestamp}.json'
            with open(metadata_file, 'w') as f:
                json.dump(metadata, f, indent=2)
            saved_files.append(str(metadata_file))
            self.get_logger().info(f'✅ Saved metadata: {metadata_file}')
            
            # Save sonar map
            if self.sonar_cloud is not None and len(self.sonar_cloud) > 0:
                sonar_filename = self.output_dir / f'sonar_map_{timestamp}.pcd'
                self.save_pcd(sonar_filename, self.sonar_cloud)
                saved_files.append(str(sonar_filename))
                self.get_logger().info(f'✅ Saved sonar map: {sonar_filename} ({len(self.sonar_cloud)} points)')
            else:
                self.get_logger().warn('⚠️  No sonar cloud data')
            
            # Export FAST-LIO2 KD-tree
            kdtree_path = self.export_kdtree(timestamp)
            if kdtree_path:
                saved_files.append(kdtree_path)
                self.get_logger().info(f'✅ Exported LiDAR map: {kdtree_path}')
            else:
                self.get_logger().warn('⚠️  Could not export KD-tree')
            
            if saved_files:
                response.success = True
                response.message = f'Saved {len(saved_files)} file(s):\n' + '\n'.join(saved_files)
            else:
                response.success = False
                response.message = 'Failed to save maps'
                
            self.get_logger().info('='*50)
        
        except Exception as e:
            response.success = False
            response.message = f'Error: {str(e)}'
            self.get_logger().error(response.message)
        
        return response

    def export_kdtree(self, timestamp):
        """Export FAST-LIO2 KD-tree to PCD"""
        try:
            import time
            
            client = self.create_client(Trigger, '/export_kdtree_pcd')
            
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn('KD-tree export service not available')
                return None
            
            request = Trigger.Request()
            future = client.call_async(request)
            
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 10.0:
                time.sleep(0.1)
            
            if future.done():
                result = future.result()
                if result.success and 'to:' in result.message:
                    return result.message.split('to:')[1].strip()
                elif result.success:
                    return result.message
                    
        except Exception as e:
            self.get_logger().error(f'Error exporting KD-tree: {e}')
        
        return None

    def save_pcd(self, filename, points):
        """Save point cloud to PCD file - MATLAB compatible ASCII format"""
        xyz = np.column_stack((points['x'], points['y'], points['z']))
        intensity = points['intensity'] if 'intensity' in points.dtype.names else np.ones(len(points))
        
        with open(filename, 'w') as f:
            f.write('# .PCD v.7 - Point Cloud Data file format\n')
            f.write('VERSION .7\n')
            f.write('FIELDS x y z intensity\n')
            f.write('SIZE 4 4 4 4\n')
            f.write('TYPE F F F F\n')
            f.write('COUNT 1 1 1 1\n')
            f.write(f'WIDTH {len(xyz)}\n')
            f.write('HEIGHT 1\n')
            f.write(f'POINTS {len(xyz)}\n')
            f.write('DATA ascii\n')
            
            for i in range(len(xyz)):
                f.write(f'{xyz[i, 0]:.6f} {xyz[i, 1]:.6f} {xyz[i, 2]:.6f} {intensity[i]:.6f}\n')


def main(args=None):
    rclpy.init(args=args)
    node = SaveMapServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

## ros2 service call /service_name std_srvs/srv/Trigger {}