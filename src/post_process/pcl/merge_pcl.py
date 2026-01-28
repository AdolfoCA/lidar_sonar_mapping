from geometry_msgs.msg import Vector3
from sensor_msgs.msg import PointCloud2, PointField
import pcl2 as point_cloud2
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import rosbag2_py
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_runtime_py.utilities import get_message

import numpy as np
import argparse
import yaml


class PCLMerger:
    
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

    def __init__(self, read_bag_path, write_bag_path, config_path):
        self._latest_cloud1 = None
        self._latest_cloud2 = None
        self._ts = 0  # nanoseconds
        self._ring = -1
        self._dt = 0  # max delta time for each pointcloud message

        self._latest_cloud1_flag = False
        self._latest_cloud2_flag = False

        self._point_template = None

        config = None
        with open(config_path, "r") as file:
            config = yaml.safe_load(file)

        if config is None:
            raise ValueError("transform data not found in config file")

        self._tf = np.array([
            config["transform"]["matrix"][0],
            config["transform"]["matrix"][1],
            config["transform"]["matrix"][2],
            config["transform"]["matrix"][3],
        ])

        self._LIDAR_CLOUD = config["ros"]["lidar_cloud_topic"]
        self._SONAR_CLOUD = config["ros"]["sonar_cloud_topic"]
        self._CLOUD_OUT = config["ros"]["output_cloud_topic"]
        self._TARGET_FRAME = config["ros"]["target_frame"]

        # initialize mcap readers and writers
        self._reader = rosbag2_py.SequentialReader()
        self._reader.open(
            rosbag2_py.StorageOptions(uri=read_bag_path, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )

        self._topic_types = self._reader.get_all_topics_and_types()
        if not any([topic.name == self._LIDAR_CLOUD for topic in self._topic_types]):
            raise ValueError(f"bag does not contain topic {self._LIDAR_CLOUD}")

        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(
            rosbag2_py.StorageOptions(uri=write_bag_path, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )

        self._writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=self._CLOUD_OUT,
                type="sensor_msgs/msg/PointCloud2",
                serialization_format="cdr",
            ),
        )

    def __del__(self):
        if self._writer:
            del self._writer
        if self._reader:
            del self._reader

    def merge(self):
        """
        Start the merging process.
        This function reads the input bag file and writes the merged pointclouds to the output bag file
        """
        while self._reader.has_next():
            topic, data, _ = self._reader.read_next()
            msg_type = get_message(self._typename(topic))
            msg = deserialize_message(data, msg_type)
            if topic == self._LIDAR_CLOUD:
                self._latest_cloud1 = msg
                self._latest_cloud1_flag = True
                self._ts = msg.header.stamp.nanosec + 1e9 * msg.header.stamp.sec

            elif topic == self._SONAR_CLOUD:
                self._latest_cloud2 = msg
                self._latest_cloud2_flag = True
                self._ts = msg.header.stamp.nanosec + 1e9 * msg.header.stamp.sec

            if self._latest_cloud1_flag and self._latest_cloud2_flag:
                # concateneate the two newest pointclouds when new ones are available
                cloud_out_msg = self._merge_clouds(
                    self._latest_cloud1, self._latest_cloud2
                )
                self._writer.write(
                    self._CLOUD_OUT, serialize_message(cloud_out_msg), int(self._ts)
                )
                self._latest_cloud1_flag = False
                self._latest_cloud2_flag = False

    def _merge_clouds(
        self, lidar_cloud: PointCloud2, sonar_cloud: PointCloud2
    ) -> PointCloud2:
        """
        Merge point cloud messages into a single point cloud message.

        Args:
            lidar_cloud (PointCloud2): LiDAR point cloud message
            sonar_cloud (PointCloud2): Sonar point cloud message

        Returns:
            PointCloud2: merged point cloud message
        """
        lidar_cloud_xyz = point_cloud2.read_points(lidar_cloud, skip_nans=True)
        sonar_cloud_xyz = point_cloud2.read_points(sonar_cloud, skip_nans=True)

        # determine what ring number to assign the points
        if self._ring == -1:
            self._ring = np.max(lidar_cloud_xyz["ring"]) + 1
            self._point_template = np.zeros_like(lidar_cloud_xyz[0])

        # determine the time delta (take the maximum delta from LiDAR)
        self._dt = np.max(lidar_cloud_xyz["t"])

        # apply transform to xyz points
        sonar_cloud_xyz = self._apply_transform(sonar_cloud_xyz, self._tf)
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

    def _apply_transform(self, cloud: np.ndarray, tf: np.ndarray) -> np.ndarray:
        """
        Transforms a point cloud from its original frame to the target frame, provided by the config file.

        Args:
            cloud (np.ndarray): numpy array of points
            tf (ndarray): transform to apply to the points

        Returns:
            np.ndarray: transformed point cloud
        """
        cloud_transformed = np.zeros_like(self._point_template, shape=(len(cloud),))
        for i, point in enumerate(cloud):
            cloud_transformed[i] = self._transform_point(point, tf)
        return cloud_transformed

    def _transform_point(self, point: np.ndarray, tf: np.ndarray) -> np.ndarray:
        """
        Transform a single point from its original frame to the target frame, provided by the config file.

        Args:
            point (np.ndarray): point to transform
            tf (ndarray): transform to apply to the point

        Returns:
            np.ndarray: transformed point
        """
        point_transformed = np.zeros_like(self._point_template)
        vector = Vector3(x=float(point[0]), y=float(point[1]), z=float(point[2]))

        vector_transformed = self._tf_vector(vector, tf)
        point_transformed["x"] = float(vector_transformed.x)
        point_transformed["y"] = float(vector_transformed.y)
        point_transformed["z"] = float(vector_transformed.z)
        point_transformed["intensity"] = point[3]
        point_transformed["t"] = int(self._dt)
        point_transformed["reflectivity"] = 0
        point_transformed["ring"] = self._ring
        point_transformed["ambient"] = 0
        point_transformed["range"] = int(
            1000
            * np.sqrt(
                vector_transformed.x**2
                + vector_transformed.y**2
                + vector_transformed.z**2
            )
        )

        return point_transformed

    def _typename(self, topic_name):
        """
        Helper function to get the type of a topic from the bag file.
        """
        for topic_type in self._topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="input bag to read from")
    parser.add_argument(
        "output", help="output directory to create and write to", default="output.mcap"
    )
    parser.add_argument("config", help="config file to read transform from")

    args = parser.parse_args()

    input_bag = args.input
    output_bag = args.output
    config_path = args.config

    pcl_merger = PCLMerger(input_bag, output_bag, config_path)
    pcl_merger.merge()
    del pcl_merger


if __name__ == "__main__":
    main()
