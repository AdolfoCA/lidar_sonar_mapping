#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#define DEG2RAD(deg) ((deg) * M_PI / 180.0)
#define NODE_NAME "tf_static_publisher"

using namespace std::chrono_literals;


class StaticTFPublisher : public rclcpp::Node
{
public:
  StaticTFPublisher() : Node(NODE_NAME)
  {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory(NODE_NAME);
    std::string default_path = package_share_directory + "/config/transforms.yaml";
    this->declare_parameter<std::string>("transforms_path", default_path);

    std::string transforms_path;
    this->get_parameter("transforms_path", transforms_path);
    loadTransformsFromYAML(transforms_path);

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publishTransforms(); 
  }

private:
  std::vector<geometry_msgs::msg::TransformStamped> transforms_;

  void loadTransformsFromYAML(const std::string &yaml_file)
  {
    YAML::Node config;
    try {
      config = YAML::LoadFile(yaml_file);
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to parse transforms file '%s': %s", yaml_file.c_str(), e.what());
      return;
    }

    if (!config["transforms"] || !config["transforms"].IsSequence()) {
      RCLCPP_ERROR(this->get_logger(),
        "No 'transforms' sequence found in '%s'", yaml_file.c_str());
      return;
    }

    for (const auto &transform : config["transforms"])
    {
      try {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = transform["frame_id"].as<std::string>();
        tf.child_frame_id  = transform["child_frame_id"].as<std::string>();
        tf.transform.translation.x = transform["translation"]["x"].as<double>();
        tf.transform.translation.y = transform["translation"]["y"].as<double>();
        tf.transform.translation.z = transform["translation"]["z"].as<double>();

        double roll  = DEG2RAD(transform["rotation"]["x"].as<double>());
        double pitch = DEG2RAD(transform["rotation"]["y"].as<double>());
        double yaw   = DEG2RAD(transform["rotation"]["z"].as<double>());

        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);

        // Handle optional invert flag: swap parent/child and invert the transform
        bool invert = false;
        if (transform["invert"]) {
          invert = transform["invert"].as<bool>();
        }
        if (invert) {
          tf2::Transform t(quat, tf2::Vector3(
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z));
          tf2::Transform t_inv = t.inverse();
          tf.transform.translation.x = t_inv.getOrigin().x();
          tf.transform.translation.y = t_inv.getOrigin().y();
          tf.transform.translation.z = t_inv.getOrigin().z();
          tf.transform.rotation = tf2::toMsg(t_inv.getRotation());
          std::swap(tf.header.frame_id, tf.child_frame_id);
        } else {
          tf.transform.rotation = tf2::toMsg(quat);
        }

        transforms_.push_back(tf);
        RCLCPP_INFO(this->get_logger(), "Loaded transform: %s -> %s%s",
          tf.header.frame_id.c_str(), tf.child_frame_id.c_str(),
          invert ? " (inverted)" : "");
      } catch (const YAML::Exception &e) {
        RCLCPP_ERROR(this->get_logger(),
          "Skipping malformed transform entry: %s", e.what());
      }
    }

    RCLCPP_INFO(this->get_logger(),
      "Loaded %zu transform(s) from '%s'", transforms_.size(), yaml_file.c_str());
  }

  void publishTransforms()
  {
    if (transforms_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No transforms to publish");
      return;
    }
    // Stamp all transforms and publish in a single call to avoid partial publishes
    for (auto &tf : transforms_) {
      tf.header.stamp = this->now();
    }
    tf_static_broadcaster_->sendTransform(transforms_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StaticTFPublisher>();
  RCLCPP_INFO(rclcpp::get_logger("tf_static_publisher"), "Node started publishing static transforms");
  rclcpp::spin(node);
  RCLCPP_INFO(rclcpp::get_logger("tf_static_publisher"), "Node stopped");
  rclcpp::shutdown();
  return 0;
}