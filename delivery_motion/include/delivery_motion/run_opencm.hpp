#ifndef RUN_OPENCM_HPP
#define RUN_OPENCM_HPP

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#include <stdlib.h>
#include <stdio.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "delivery_motion/motor_driver.hpp"

#define PI 3.141592653589793

class DynamixelController : public rclcpp::Node, MotorCommand {
public:
    DynamixelController();
private:
    void twist_cb(const geometry_msgs::msg::Twist msg);
    void gimbal_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
    void timer_cb();
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr gimbal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int64_t yawPos_[4] = {2048, };
    int64_t gimbalPos_[4] = {2048, };
    double wheelOffset1_, wheelOffset2_, axleWidth_;
};

#endif
