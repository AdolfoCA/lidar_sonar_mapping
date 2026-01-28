import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

class GPSPathNode(Node):

    def __init__(self) -> None:
        super().__init__("gps_path_node")

        self.poses = []
        self.new_poses_flag = False

        self._subscription = self.create_subscription(
            Odometry, "/odometry/gps", self._odom_callback, 10
        )
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self._timer_callback)
        self._publisher = self.create_publisher(Path, "/odometry/gps/path", 10)

        self.i = 0

    def _odom_callback(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.poses.append(pose)
        self.new_poses_flag = True


    def _timer_callback(self) -> None:
        if not self.new_poses_flag:
            return
        
        path_msg = Path()
        path_msg.header = self.poses[-1].header
        path_msg.poses = self.poses

        self._publisher.publish(path_msg)
        self.new_poses_flag = False
        


def main(args=None):
    rclpy.init(args=args)
    gps_path_node = GPSPathNode()
    rclpy.spin(gps_path_node)
    gps_path_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
