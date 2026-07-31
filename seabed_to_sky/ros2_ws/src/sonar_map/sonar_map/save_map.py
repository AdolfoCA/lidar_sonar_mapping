#!/usr/bin/env python3
"""
Save Map Server - Saves PCD files with initial pose from LiDAR odometry.

Saves, on a /save_map Trigger:
  * <topic>_<ts>.pcd — the map cloud from `map_topic` (default /sonar_map, the
    occupancy voxel map from sonar_map_ned) with EVERY float32 field the
    PointCloud2 carries, ASCII PCD so MATLAB can read each one. The file is
    named after the topic, so /sonar_map -> sonar_map_<ts>.pcd and
    /dirichlet_voxels -> dirichlet_voxels_<ts>.pcd.
    This node is deliberately FIELD-AGNOSTIC, which is what lets one node save
    either map: /sonar_map carries x/y/z/intensity, while the eq-24 pipeline's
    /dirichlet_voxels carries one posterior column per class (theta_<name>,
    e.g. theta_seabed ... theta_other) alongside x/y/z and the display fields
    (confidence, weight, ...) — a config-defined inventory. Hardcoding names
    here would silently drop columns whenever classes.yaml changes; instead the
    PCD header is built from the message's field list at save time.
  * the FAST-LIO2 LiDAR map (via the /export_kdtree_pcd service) — the LiDAR
    cloud to overlay against the sonar map in MATLAB.
  * metadata_<ts>.json — initial / final odometry pose.

In MATLAB the extra fields are read by parsing the ASCII DATA section directly
(pcread only returns x/y/z + intensity/colour); the FIELDS line names every
column in order.

To save a different cloud without a rebuild:
    ros2 run sonar_map save_map --ros-args -p map_topic:=/dirichlet_voxels
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
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
        
        # The map cloud to save — every field it carries is kept for the MATLAB
        # export (per-class theta_<name> columns included, whatever the class
        # set is). Default /sonar_map (sonar_map_ned's occupancy voxel map);
        # set map_topic to /dirichlet_voxels to save the eq-24 posterior cloud.
        self.map_topic = self.declare_parameter('map_topic', '/sonar_map').value
        # The saved file is named after the topic: /sonar_map -> sonar_map_<ts>.pcd.
        self.file_prefix = self.map_topic.strip('/').replace('/', '_') or 'map'
        # BEST_EFFORT on the SUBSCRIBER side is compatible with both publishers:
        # /dirichlet_voxels offers BEST_EFFORT (sensor-data QoS) and /sonar_map
        # offers RELIABLE, and a best-effort request is satisfied by either. A
        # RELIABLE request would silently never connect to /dirichlet_voxels.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_sub = self.create_subscription(
            PointCloud2,
            self.map_topic,
            self.map_callback,
            map_qos
        )

        # LiDAR odometry from SPARK-FastLIO2
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry',
            self.odom_callback,
            1
        )
        
        self.save_service = self.create_service(
            Trigger,
            '/save_map',
            self.save_map_callback
        )
        
        # Data storage
        self.map_cloud = None          # structured np array, all cloud fields
        self.map_field_names = None    # ordered field names from the message
        self.initial_pose = None
        self.current_pose = None
        self.init_captured = False

        self.output_dir = Path.home() / 'ros2_ws' / 'saved_maps' / 'PCD'
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.get_logger().info('='*50)
        self.get_logger().info('Save Map Server started')
        self.get_logger().info('='*50)
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info(
            f'Saving map topic : {self.map_topic} -> {self.file_prefix}_<ts>.pcd')
        self.get_logger().info(f'Listening for /odometry and {self.map_topic}')

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

    def map_callback(self, msg: PointCloud2):
        """Store the latest map cloud with ALL its fields.

        We read every field present on the message (no field_names subset) and
        remember their names IN MESSAGE ORDER, so the saved PCD carries whatever
        the publisher put there — x/y/z/intensity for /sonar_map, or x/y/z plus
        one theta_<class> column per classes.yaml entry (incl. theta_other) and
        the display fields for /dirichlet_voxels. Nothing is hardcoded here.
        """
        try:
            self.map_field_names = [f.name for f in msg.fields]
            pts = np.asarray(pc2.read_points(msg, skip_nans=False))
            self.map_cloud = pts
        except Exception as e:
            self.get_logger().error(f'Error reading map cloud from {self.map_topic}: {e}')

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
            
            # Save the map cloud with ALL fields
            if self.map_cloud is not None and len(self.map_cloud) > 0:
                map_filename = self.output_dir / f'{self.file_prefix}_{timestamp}.pcd'
                self.save_pcd(map_filename, self.map_cloud,
                              self.map_field_names)
                saved_files.append(str(map_filename))
                self.get_logger().info(
                    f'✅ Saved {self.map_topic}: {map_filename} '
                    f'({len(self.map_cloud)} points, '
                    f'fields={self.map_field_names})')
            else:
                self.get_logger().warn(
                    f'⚠️  No cloud data on {self.map_topic} '
                    f'(is the pipeline publishing?)')
            
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
        """Export FAST-LIO2 KD-tree to PCD via ros2 service CLI"""
        import subprocess
        try:
            result = subprocess.run(
                ['ros2', 'service', 'call', '/export_kdtree_pcd', 'std_srvs/srv/Trigger', '{}'],
                capture_output=True, text=True, timeout=10.0
            )
            self.get_logger().info(f'KD-tree service stdout: "{result.stdout}"')
            self.get_logger().info(f'KD-tree service stderr: "{result.stderr}"')
            
            if 'success=True' in result.stdout:
                # Parse path from response message
                if 'to:' in result.stdout:
                    # Extract path between 'to:' and the closing quote/newline
                    path = result.stdout.split('to:')[1].split("'")[0].strip()
                    return path
                return 'kdtree_exported'
        except subprocess.TimeoutExpired:
            self.get_logger().warn('KD-tree export timed out')
        except Exception as e:
            self.get_logger().error(f'Error exporting KD-tree: {e}')
            return None


    def save_pcd(self, filename, points, field_names=None):
        """Save a point cloud to an ASCII PCD file with ALL given fields.

        ``points`` is a structured numpy array (as returned by pc2.read_points);
        ``field_names`` is the ordered list of fields to write. The header is
        DYNAMIC — FIELDS/SIZE/TYPE/COUNT are generated from that list, so any
        class inventory (any number of theta_<name> columns) round-trips
        without touching this node. Every field is written as a float32 column
        so MATLAB can read each one; the FIELDS line names the columns in
        order, so a MATLAB reader can map column -> field.
        """
        if field_names is None:
            field_names = list(points.dtype.names)
        # Keep only fields actually present (defensive).
        field_names = [n for n in field_names if n in points.dtype.names]
        n = len(points)

        # Stack the requested fields into one (N, F) float array, in order.
        cols = np.column_stack(
            [np.asarray(points[name], dtype=np.float64) for name in field_names])

        n_fields = len(field_names)
        sizes = ' '.join(['4'] * n_fields)
        types = ' '.join(['F'] * n_fields)
        counts = ' '.join(['1'] * n_fields)

        with open(filename, 'w') as f:
            f.write('# .PCD v.7 - Point Cloud Data file format\n')
            f.write('VERSION .7\n')
            f.write('FIELDS ' + ' '.join(field_names) + '\n')
            f.write(f'SIZE {sizes}\n')
            f.write(f'TYPE {types}\n')
            f.write(f'COUNT {counts}\n')
            f.write(f'WIDTH {n}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {n}\n')
            f.write('DATA ascii\n')
            for i in range(n):
                f.write(' '.join(f'{v:.6f}' for v in cols[i]) + '\n')


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

## ros2 service call /save_map std_srvs/srv/Trigger {}