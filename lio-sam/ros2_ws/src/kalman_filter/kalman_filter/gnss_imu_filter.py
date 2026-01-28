import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation

from .filters.filter import GNSSIMUKalmanFilter
# from filters.ekf import GNSSIMUEKF
from threading import Lock

class GNSSIMUFilterNode(Node):

    GNSS_TOPIC = "/odometry/gps"
    IMU_TOPIC = "/imu/data/rep_tf"
    FILTER_TOPIC = "/odometry/filtered"
    FILTER_FRAME = "odom_enu"
    BASE_LINK_FRAME = "base_link"
    UPDATE_RATE = 100    # Hz
    # YAW_OFFSET = 0.0 * (np.pi / 180.0)

    def __init__(self) -> None:
        super().__init__("gnss_imu_filter_node")

        self._imu_msg = None

        self._publisher = self.create_publisher(Odometry, self.FILTER_TOPIC, 10)
        self._filter_path_publisher = self.create_publisher(Path, self.FILTER_TOPIC + "/path", 10)
        self._gnss_path_publisher = self.create_publisher(Path, self.GNSS_TOPIC + "/path", 10)
        self._imu_sub = self.create_subscription(Imu, self.IMU_TOPIC, self.imu_callback, 10)
        self._gnss_sub = self.create_subscription(Odometry, self.GNSS_TOPIC, self.gnss_callback, 10)
        self._kf_timer = self.create_timer(1 / self.UPDATE_RATE, self.kf_callback)
        self._path_timer = self.create_timer(1, self._raise_path_flag)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._kf = GNSSIMUKalmanFilter(rate=self.UPDATE_RATE)
        self._kf_lock = Lock()

        self.quat = None
        self.gnss_poses = []
        self.filter_poses = []
        self.poses_flag = True


    def imu_callback(self, msg: Imu):
        self._imu_msg = msg
        self._imu_flag = True

    def gnss_callback(self, msg: Odometry):
        # get heading angle from Quaternion
        self.quat = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        self.gnss_poses.append(msg.pose.pose)

        r = Rotation.from_quat(self.quat)
        euler = r.as_euler("zyx", degrees=False)
        heading = euler[0] - 1.571 # adjust the heading angle wrt. easting

        measurement = np.array([
            msg.pose.pose.position.x, 
            msg.pose.pose.position.y,
            heading
        ])

        R = np.zeros((len(measurement), len(measurement)))
        R[0, 0] = msg.pose.covariance[0]
        R[1, 1] = msg.pose.covariance[7]
        # R[2, 2] = 0.0 # we assume the heading is perfect

        with self._kf_lock:
            self._kf.update(m=measurement, R=R)
        
    def kf_callback(self):
        state = None
        cov = None

        if self._imu_msg is None:
            return
        
        u = np.array([
                self._imu_msg.linear_acceleration.x,
                self._imu_msg.linear_acceleration.y,
            ])

        with self._kf_lock:
            self._kf.predict(u)
            state, cov = self._kf.get_state()

        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.FILTER_FRAME
        msg.child_frame_id = self.BASE_LINK_FRAME

        msg.pose.pose.position.x = state[0]
        msg.pose.pose.position.y = state[1]

        if self.poses_flag:
            self.filter_poses.append(msg.pose.pose)
            self.poses_flag = False

        msg.pose.covariance[0] = cov[0,0]
        msg.pose.covariance[7] = cov[1,1]

        if self.quat is not None:
            msg.pose.pose.orientation.x = self.quat[0]
            msg.pose.pose.orientation.y = self.quat[1]
            msg.pose.pose.orientation.z = self.quat[2]
            msg.pose.pose.orientation.w = self.quat[3]

        self._publisher.publish(msg)
        self._tf_publish(msg)


    def _tf_publish(self, odom_msg: Odometry) -> None:
        tf_stamp_msg = TransformStamped()
        tf_stamp_msg.header.stamp = odom_msg.header.stamp
        tf_stamp_msg.header.frame_id = self.FILTER_FRAME
        tf_stamp_msg.child_frame_id = self.BASE_LINK_FRAME
        tf_stamp_msg.transform.translation.x = odom_msg.pose.pose.position.x
        tf_stamp_msg.transform.translation.y = odom_msg.pose.pose.position.y

        tf_stamp_msg.transform.rotation.x = odom_msg.pose.pose.orientation.x
        tf_stamp_msg.transform.rotation.y = odom_msg.pose.pose.orientation.y
        tf_stamp_msg.transform.rotation.z = odom_msg.pose.pose.orientation.z
        tf_stamp_msg.transform.rotation.w = odom_msg.pose.pose.orientation.w

        self._tf_broadcaster.sendTransform(tf_stamp_msg)

    def _raise_path_flag(self) -> None:
        self.poses_flag = True

        if len(self.filter_poses) == 0:
            return
        
        self.get_logger().info(f"Velocities: {self._kf.x[2:4]}")
        self.get_logger().info(f"Biases: {self._kf.x[4:6]}")
        self.get_logger().info(f"Residual: {self._kf.y}")
        msg = Path()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.FILTER_FRAME

        msg.poses = [PoseStamped(pose=pose, header=msg.header) for pose in self.filter_poses]
        self._filter_path_publisher.publish(msg)

        msg.poses = [PoseStamped(pose=pose, header=msg.header) for pose in self.gnss_poses]
        self._gnss_path_publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    gnss_imu_filter_node = GNSSIMUFilterNode()
    rclpy.spin(gnss_imu_filter_node)
    gnss_imu_filter_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


