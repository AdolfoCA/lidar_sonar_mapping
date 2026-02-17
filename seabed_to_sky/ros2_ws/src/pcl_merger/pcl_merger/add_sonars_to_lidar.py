from geometry_msgs.msg import Vector3
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.parameter import Parameter
import numpy as np
import rclpy
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
from pcl_merger.transform_tools import create_global_transforms, compute_relative_transform_matrix
import pcl_merger.pcl2 as point_cloud2
from tf2_msgs.msg import TFMessage
from rclpy.node import Node


class PCLMerger(Node):

    # mimic of pointcloud fields from Ouster OS1
    _PC_FIELDS = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name="t", offset=20, datatype=PointField.UINT32, count=1),
        PointField(name="reflectivity", offset=24, datatype=PointField.UINT16, count=1),
        PointField(name="ring", offset=26, datatype=PointField.UINT16, count=1),
        PointField(name="ambient", offset=28, datatype=PointField.UINT16, count=1),
        PointField(name="range", offset=32, datatype=PointField.UINT32, count=1),
    ]

    def __init__(self):
        super().__init__("add_sonars_to_lidar")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("lidar_cloud_topic", Parameter.Type.STRING),
                ("transforms.lidar_cloud", Parameter.Type.STRING),
                ("sonar_cloud_horizontal_topic", Parameter.Type.STRING),
                ("transforms.sonar_cloud_horizontal", Parameter.Type.STRING),
                ("sonar_cloud_vertical_topic", Parameter.Type.STRING),
                ("transforms.sonar_cloud_vertical", Parameter.Type.STRING),
                ("sonar_cloud_patch_topic", Parameter.Type.STRING),
                ("transforms.sonar_cloud_patch", Parameter.Type.STRING),
                ("output_cloud_topic", Parameter.Type.STRING),
                ("target_frame", Parameter.Type.STRING),
                ("transforms_topic", Parameter.Type.STRING),
            ],
        )
        self._ts = 0  # nanoseconds
        self._dt = 0  # max delta time for each pointcloud message
        self._resolution = 1024  # Number of point for each ring of the lidar

        self._last_sonar_horizontal = None
        self._last_sonar_vertical = None
        self._last_sonar_patch = None
        self._max_ring = 128

        self._point_template = None

        self._LIDAR_CLOUD = self.get_parameter("lidar_cloud_topic").get_parameter_value().string_value
        self._LIDAR_CLOUD_TF = self.get_parameter("transforms.lidar_cloud").get_parameter_value().string_value
        self._SONAR_CLOUD_HORIZONTAL = (
            self.get_parameter("sonar_cloud_horizontal_topic").get_parameter_value().string_value
        )
        self._SONAR_CLOUD_HORIZONTAL_TF = (
            self.get_parameter("transforms.sonar_cloud_horizontal").get_parameter_value().string_value
        )
        self._SONAR_CLOUD_VERTICAL = self.get_parameter("sonar_cloud_vertical_topic").get_parameter_value().string_value
        self._SONAR_CLOUD_VERTICAL_TF = (
            self.get_parameter("transforms.sonar_cloud_vertical").get_parameter_value().string_value
        )
        self._SONAR_CLOUD_PATCH = self.get_parameter("sonar_cloud_patch_topic").get_parameter_value().string_value
        self._SONAR_CLOUD_PATCH_TF = (
            self.get_parameter("transforms.sonar_cloud_patch").get_parameter_value().string_value
        )
        self._CLOUD_OUT = self.get_parameter("output_cloud_topic").get_parameter_value().string_value
        self._TARGET_FRAME = self.get_parameter("target_frame").get_parameter_value().string_value
        self._TRANSFORMS = self.get_parameter("transforms_topic").get_parameter_value().string_value

        self._SONAR_HORIZONTAL_TO_LIDAR_TF = np.zeros((4, 4))
        self._SONAR_VERTICAL_TO_LIDAR_TF = np.zeros((4, 4))
        self._SONAR_PATCH_TO_LIDAR_TF = np.zeros((4, 4))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._cloud_pub = self.create_publisher(PointCloud2, self._CLOUD_OUT, 10)

        self.create_subscription(PointCloud2, self._LIDAR_CLOUD, self.lidar_callback, 10)
        self.create_subscription(PointCloud2, self._SONAR_CLOUD_HORIZONTAL, self.sonar_horizontal_callback, 10)
        self.create_subscription(PointCloud2, self._SONAR_CLOUD_VERTICAL, self.sonar_vertical_callback, 10)
        self.create_subscription(PointCloud2, self._SONAR_CLOUD_PATCH, self.sonar_patch_callback, 10)

        self.create_subscription(TFMessage, self._TRANSFORMS, self.transforms_callback, 10)
        self.get_logger().debug("Initialized PCLMerger node")

    def get_transform_matrix(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # Convert quaternion to a rotation matrix
            r = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
            rotation_matrix = r.as_matrix()

            # Create a 4x4 transformation matrix
            matrix = np.eye(4)
            matrix[:3, :3] = rotation_matrix
            matrix[:3, 3] = [translation.x, translation.y, translation.z]

            return matrix
        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            return None

    def sonar_horizontal_callback(self, msg):
        self.get_logger().debug("Received message on SONAR_CLOUD_HORIZONTAL edge")
        self._last_sonar_horizontal = msg

    def sonar_vertical_callback(self, msg):
        self.get_logger().debug("Received message on SONAR_CLOUD_VERTICAL edge")
        self._last_sonar_vertical = msg

    def sonar_patch_callback(self, msg):
        self.get_logger().debug("Received message on SONAR_CLOUD_PATCH")
        self._last_sonar_patch = msg

    def lidar_callback(self, lidar_cloud):
        """Callback for the LiDAR point cloud message. Adds sonar point clouds to the LiDAR point cloud message.
        Publishes the merged point cloud message.
        """
        # Make sure that the message is in the correct format
        lidar_cloud_xyz = point_cloud2.read_points(lidar_cloud, skip_nans=True)
        proper_dtype = point_cloud2.dtype_from_fields(self._PC_FIELDS, point_step=48)
        lidar_cloud_xyz = lidar_cloud_xyz.astype(proper_dtype)
        self._max_ring = np.max(lidar_cloud_xyz["ring"])
        merged_cloud = self._create_cloud(lidar_cloud_xyz)
        self._ts = lidar_cloud.header.stamp.nanosec + 1e9 * lidar_cloud.header.stamp.sec

        # Write to terminal
        self.get_logger().debug("Received message on LIDAR_CLOUD")

        if self._last_sonar_horizontal:
            merged_cloud = self._merge_clouds(merged_cloud, self._last_sonar_horizontal, ring=-1)
        if self._last_sonar_vertical:
            merged_cloud = self._merge_clouds(merged_cloud, self._last_sonar_vertical, ring=-2)
        if self._last_sonar_patch:
            merged_cloud = self._merge_clouds(merged_cloud, self._last_sonar_patch, ring=-3)
            self.get_logger().debug("Merged LiDAR and sonar patch cloud messages")

        self._cloud_pub.publish(merged_cloud)

        # Clear sonar point clouds
        self._last_sonar_horizontal = None
        self._last_sonar_vertical = None
        self._last_sonar_patch = None

    def transforms_callback(self, msg):
        self.get_logger().debug("Received message on TRANSFORMS")
        global_transforms = create_global_transforms(msg)
        self.get_logger().debug("Computed global transforms")
        try:
            self._SONAR_HORIZONTAL_TO_LIDAR_TF = self.get_transform_matrix(
                self._LIDAR_CLOUD_TF, self._SONAR_CLOUD_HORIZONTAL_TF
            )
            self._SONAR_VERTICAL_TO_LIDAR_TF = self.get_transform_matrix(
                self._LIDAR_CLOUD_TF, self._SONAR_CLOUD_VERTICAL_TF
            )
            self.get_logger().debug("Computed relative transforms successfully")
        except KeyError:
            self.get_logger().error(
                "Could not compute relative transforms. Check that all transforms are present in the message."
            )

    def _merge_clouds(
        self,
        cloud: PointCloud2,
        cloud_to_be_added: PointCloud2,
        ring: int = -1,
    ) -> PointCloud2:
        """
        Merge point cloud messages into a single point cloud message.

        Args:
            lidar_cloud (PointCloud2): LiDAR point cloud message
            sonar_cloud (PointCloud2): Sonar point cloud message

        Returns:
            PointCloud2: merged point cloud message
        """
        lidar_cloud_xyz = point_cloud2.read_points(cloud, skip_nans=True)
        sonar_cloud_xyz = point_cloud2.read_points(cloud_to_be_added, skip_nans=True)

        # determine what ring number to assign the points
        if ring == -1:
            ring = self._max_ring + 1
            tf = self._SONAR_HORIZONTAL_TO_LIDAR_TF
        elif ring == -2:
            ring = self._max_ring + 2
            tf = self._SONAR_VERTICAL_TO_LIDAR_TF
        elif ring == -3:
            ring = self._max_ring + 3
            tf = self._SONAR_HORIZONTAL_TO_LIDAR_TF
        self._point_template = np.zeros_like(lidar_cloud_xyz[0])

        # determine the time delta (take the maximum delta from LiDAR)
        self._dt = np.max(lidar_cloud_xyz["t"])

        # apply transform to xyz points
        sonar_cloud_xyz = self._apply_transform_loop(sonar_cloud_xyz, tf, ring)
        proper_dtype = point_cloud2.dtype_from_fields(self._PC_FIELDS, point_step=48)
        sonar_cloud_xyz = sonar_cloud_xyz.astype(proper_dtype)

        merged_cloud = np.concatenate((lidar_cloud_xyz, sonar_cloud_xyz), axis=0)
        merged_cloud = merged_cloud.astype(proper_dtype)

        cloud_msg = self._create_cloud(merged_cloud)
        return cloud_msg

    def _create_cloud(self, points: np.ndarray) -> PointCloud2:
        """
        Takes a numpy array of points and generates a PointCloud2 message.

        Args:
            points (np.ndarray): numpy array of points

        Returns:
            PointCloud2: PointCloud2 message
        """
        header = Header()
        header.stamp = Time(sec=int(self._ts // 1e9), nanosec=int(self._ts % 1e9))
        header.frame_id = self._TARGET_FRAME

        msg = point_cloud2.create_cloud(header, self._PC_FIELDS, points, point_step=48)
        msg.is_dense = True

        return msg

    def _apply_transform(self, cloud: np.ndarray, tf: np.ndarray, ring: int) -> np.ndarray:
        """
        Transforms a point cloud from its original frame to the target frame, provided by the config file.
        EXPERIMENTAL. NEEDS TESTING.
        Args:
            cloud (np.ndarray): numpy array of points
            tf (ndarray): transform to apply to the points
            ring (int): ring number to assign to the points

        Returns:
            np.ndarray: transformed point cloud
        """

        if not isinstance(cloud, np.ndarray):
            raise TypeError("Cloud must be a structured numpy ndarray.")
        if not all(field in cloud.dtype.names for field in ["x", "y", "z"]):
            raise ValueError("Structured array must have fields 'x', 'y', and 'z'.")

        points = np.stack((cloud["x"], cloud["y"], cloud["z"], np.ones(len(cloud))), axis=-1)  # Shape: (N, 4)
        points_transformed = (tf @ points.T).T  # Shape: (N, 4)
        cloud_transformed = np.zeros(len(cloud), dtype=self._point_template.dtype)
        cloud_transformed["x"] = points_transformed[:, 0]
        cloud_transformed["y"] = points_transformed[:, 1]
        cloud_transformed["z"] = points_transformed[:, 2]
        cloud_transformed["intensity"] = cloud["intensity"]
        cloud_transformed["t"] = np.ones(len(cloud)) * self._dt
        cloud_transformed["reflectivity"] = np.zeros(len(cloud))
        cloud_transformed["ring"] = np.arange(len(cloud)) // self._resolution + ring
        cloud_transformed["ambient"] = np.zeros(len(cloud))
        cloud_transformed["range"] = np.sqrt(
            cloud_transformed["x"] ** 2 + cloud_transformed["y"] ** 2 + cloud_transformed["z"] ** 2
        ).astype(np.uint32)

        return cloud_transformed

    def _apply_transform_loop(self, cloud: np.ndarray, tf: np.ndarray, ring: int) -> np.ndarray:
        """
        Transforms a point cloud from its original frame to the target frame, provided by the config file.

        Args:
            cloud (np.ndarray): numpy array of points
            tf (ndarray): transform to apply to the points
            ring (int): ring number to assign to the points

        Returns:
            np.ndarray: transformed point cloud
        """

        cloud_transformed = np.zeros_like(self._point_template, shape=(len(cloud),))
        for i, point in enumerate(cloud):
            # assign ring number to the point
            if i > self._resolution - 1:
                ring += 1

            cloud_transformed[i] = self._transform_point(point, tf, ring)
        return cloud_transformed

    def _transform_point(self, point: np.ndarray, tf: np.ndarray, ring: int) -> np.ndarray:
        """
        Transform a single point from its original frame to the target frame, provided by the config file.

        Args:
            point (np.ndarray): point to transform
            tf (ndarray): transform to apply to the point
            ring (int): ring number to assign to the point

        Returns:
            np.ndarray: transformed point
        """
        point_transformed = np.zeros_like(self._point_template)
        vector = Vector3(x=float(point[0]), y=float(point[1]), z=float(point[2]))

        vector_transformed = self._tf_vector(vector, tf)
        point_transformed["x"] = float(vector_transformed.x)
        point_transformed["y"] = float(vector_transformed.y)
        point_transformed["z"] = float(vector_transformed.z)
        point_transformed["intensity"] = point["intensity"]
        point_transformed["t"] = int(self._dt)
        point_transformed["reflectivity"] = 0
        point_transformed["ring"] = ring
        point_transformed["ambient"] = 0
        point_transformed["range"] = int(
            1000 * np.sqrt(vector_transformed.x**2 + vector_transformed.y**2 + vector_transformed.z**2)
        )

        return point_transformed

    def _tf_vector(self, vec: Vector3, tf: np.ndarray) -> Vector3:
        """
        Calculate the transformation of a vector using a Transform message.

        Args:
            vec (Vector3): vector to transform
            tf (ndarray): transform to apply to the vector

        Returns:
            Vector3: transformed vector
        """

        vec_homogeneous = np.array([vec.x, vec.y, vec.z, 1])
        vec_transformed = tf.dot(vec_homogeneous)[:3]

        return Vector3(x=vec_transformed[0], y=vec_transformed[1], z=vec_transformed[2])


def main(args=None):
    rclpy.init(args=args)
    pcl_merger = PCLMerger()
    rclpy.spin(pcl_merger)
    pcl_merger.destroy_node()
    rclpy.shutdown()
