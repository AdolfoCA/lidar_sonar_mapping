#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "spark_fast_lio.h"

// Use a MultiThreadedExecutor so that the sensor subscriptions (IMU, LiDAR)
// assigned to sensor_cb_group_ can receive data concurrently with the main
// processing timer. Without this, processLidarAndImu() blocks IMU delivery
// and causes repeated "Not enough IMU data" retries.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<spark_fast_lio::SPARKFastLIO2>(rclcpp::NodeOptions());

  // 2 threads: one for sensor callbacks, one for the processing timer.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
