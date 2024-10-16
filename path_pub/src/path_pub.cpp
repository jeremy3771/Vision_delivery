#include <iostream>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

class PathPublisher : public rclcpp::Node
{
public:
  PathPublisher() : Node("path_publisher")
  {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_plan", 10);
    velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/path_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PathPublisher::publishPathAndVelocities, this));

    readPathFromYAML();
  }

private:
  void readPathFromYAML()
  {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("path_pub");
    std::string yaml_file_path = package_share_directory + "/config/path.yaml";

    YAML::Node yaml_file = YAML::LoadFile(yaml_file_path);
    auto path = yaml_file["path"];
    if (!path) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load path from YAML file: %s", yaml_file_path.c_str());
      return;
    }

    for (const auto & entry : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = entry["point"]["x"].as<double>();
      pose.pose.position.y = entry["point"]["y"].as<double>();
      pose.pose.position.z = entry["point"]["z"].as<double>();
      pose.pose.orientation.w = 1.0;

      double linear_x = entry["velocity"]["linear_x"].as<double>();
      path_.poses.push_back(pose);
      velocities_.push_back(linear_x);
    }

    path_.header.frame_id = "map";
    RCLCPP_INFO(this->get_logger(), "Path loaded with %zu points from %s", path_.poses.size(), yaml_file_path.c_str());
  }

  void publishPathAndVelocities()
  {
    if (path_.poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "No path available to publish.");
      return;
    }

    path_.header.stamp = this->get_clock()->now();
    path_pub_->publish(path_);

    std_msgs::msg::Float64MultiArray velocity_array_msg;
    velocity_array_msg.data = velocities_;

    RCLCPP_INFO(this->get_logger(), "Publishing velocities array with %zu elements", velocities_.size());
    velocity_pub_->publish(velocity_array_msg);
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path path_;
  std::vector<double> velocities_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

