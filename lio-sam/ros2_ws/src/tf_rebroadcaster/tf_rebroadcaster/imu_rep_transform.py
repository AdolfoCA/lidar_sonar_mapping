import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R

from tf_transformations import euler_from_quaternion


class ImuRepTransformNode(Node):
    IMU_FRAME = "imu"
    BASE_LINK_FRAME = "base_link"

    def __init__(self):
        super().__init__("imu_rep_transform_node")
        self.subscription = self.create_subscription(
            Imu, "/imu/data", self.imu_callback, 10
        )
        self.publisher = self.create_publisher(Imu, "/imu/data/rep_tf", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def imu_callback(self, msg):
        transformed_imu = self.transform_imu(msg)
        if transformed_imu is not None:
            self.publisher.publish(transformed_imu)

    def transform_imu(self, imu_msg):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.BASE_LINK_FRAME, self.IMU_FRAME, imu_msg.header.stamp
            )
        except Exception as e:
            self.get_logger().error(f"Could not find transform at lookup: {e}")
            return

        transformed_imu = Imu()
        transformed_imu.header = imu_msg.header
        transformed_imu.header.frame_id = self.BASE_LINK_FRAME
        
        # Transform the IMU orientation
        imu_orientation_transformed = self._quaternion_multiply(
            imu_msg.orientation, tf.transform.rotation
        )

        # Transform the IMU angular velocity
        imu_ang_vel_transformed = self._tf_vector(tf, imu_msg.angular_velocity)

        # Transform the IMU linear acceleration
        imu_lin_acc_transformed = self._tf_vector(tf, imu_msg.linear_acceleration)

        # Fill the transformed IMU message
        transformed_imu.orientation = imu_orientation_transformed
        transformed_imu.orientation_covariance = imu_msg.orientation_covariance
        transformed_imu.angular_velocity = imu_ang_vel_transformed
        transformed_imu.angular_velocity_covariance = (
            imu_msg.angular_velocity_covariance
        )
        transformed_imu.linear_acceleration = imu_lin_acc_transformed
        transformed_imu.linear_acceleration_covariance = (
            imu_msg.linear_acceleration_covariance
        )

        return transformed_imu

    def _quaternion_multiply(self, q0: Quaternion, q1: Quaternion) -> Quaternion:
        q0q1_w = q0.w * q1.w - q0.x * q1.x - q0.y * q1.y - q0.z * q1.z
        q0q1_x = q0.w * q1.x + q0.x * q1.w + q0.y * q1.z - q0.z * q1.y
        q0q1_y = q0.w * q1.y - q0.x * q1.z + q0.y * q1.w + q0.z * q1.x
        q0q1_z = q0.w * q1.z + q0.x * q1.y - q0.y * q1.x + q0.z * q1.w

        qfinal = Quaternion(x=q0q1_x, y=q0q1_y, z=q0q1_z, w=q0q1_w)

        return qfinal

    def _tf_vector(self, tf: TransformStamped, vec: Vector3) -> Vector3:
        rot = R.from_quat(
            [
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w,
            ]
        )
        rot_euler = rot.as_matrix()

        vec_arr = np.array([vec.x, vec.y, vec.z])
        vec_transformed = rot_euler.dot(vec_arr)

        return Vector3(x=vec_transformed[0], y=vec_transformed[1], z=vec_transformed[2])


def main(args=None):
    rclpy.init(args=args)
    imu_transform_node = ImuRepTransformNode()
    rclpy.spin(imu_transform_node)
    imu_transform_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
