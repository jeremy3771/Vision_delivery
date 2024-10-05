#ifndef VELOCITY_SMOOTHER__VELOCITY_SMOOTHER_HPP_
#define VELOCITY_SMOOTHER__VELOCITY_SMOOTHER_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class VelocitySmoother : public rclcpp::Node {
public:
  VelocitySmoother();
  ~VelocitySmoother();
private:
  void twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void smootherTimer();
  double applyConstraints(
    const double v_curr, const double v_cmd,
    const double accel, const double decel);

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Clock::SharedPtr clock_;
  geometry_msgs::msg::Twist::SharedPtr cmd_;
  geometry_msgs::msg::Twist last_cmd_;
  double smoothing_frequency_;
  bool stopped_{true};
  std::vector<double> max_velocities_;
  std::vector<double> min_velocities_;
  std::vector<double> max_accels_;
  std::vector<double> max_decels_;
  std::vector<double> deadband_velocities_;
  rclcpp::Duration velocity_timeout_{0, 0};
  rclcpp::Time last_command_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

#endif