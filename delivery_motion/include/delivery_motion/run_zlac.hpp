#ifndef RUN_ZLAC_HPP
#define RUN_ZLAC_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "zlac8015d.h"

class ZLAC_Twist : public rclcpp::Node {
public:
  ZLAC_Twist();
  ~ZLAC_Twist();
  void timer_cb();
  void twist_cb(const geometry_msgs::msg::Twist msg);
private:
  double wheelOffset1_, wheelOffset2_, axleWidth_, wheelDiameter_;
  static ZLAC front_mots; // Change Port Name
	static ZLAC rear_mots;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  double motor_rpm_[4] = {0.0, };
};

#endif