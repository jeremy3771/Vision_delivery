#include "delivery_motion/run_opencm.hpp"

#include <iostream>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

DynamixelController::DynamixelController() : Node("dynamixel_controller") {
    declare_parameter("WO1", 0.285);
    declare_parameter("WO2", 0.285);
    declare_parameter("AW", 0.453);
    get_parameter("WO1", wheelOffset1_);
    get_parameter("WO2", wheelOffset2_);
    get_parameter("AW", axleWidth_);

    std::fill(yawPos_, yawPos_ + 4, 2048);
    std::fill(gimbalPos_, gimbalPos_ + 4, 2048);

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_smoothed", 10, std::bind(&DynamixelController::twist_cb ,this, _1));
    gimbal_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("gimbal_pos", 10, std::bind(&DynamixelController::gimbal_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(50ms, std::bind(&DynamixelController::timer_cb, this));
}

void DynamixelController::twist_cb(const geometry_msgs::msg::Twist msg) {
    double radius = std::abs(msg.linear.x / msg.angular.z);

    if (msg.angular.z < -0.001 && radius > 0.01) {
        yawPos_[0] = 2048 + (std::atan2(wheelOffset1_, radius + (axleWidth_ / 2)) * 2048 / PI);
        yawPos_[1] = 2048 + (std::atan2(wheelOffset1_, radius - (axleWidth_ / 2)) * 2048 / PI);
        yawPos_[2] = 2048 + (std::atan2(wheelOffset2_, radius + (axleWidth_ / 2)) * 2048 / PI);
        yawPos_[3] = 2048 + (std::atan2(wheelOffset2_, radius - (axleWidth_ / 2)) * 2048 / PI);
    }
    else if (msg.angular.z > 0.001 && radius > 0.01) {
        yawPos_[0] = 2048 - (std::atan2(wheelOffset1_, radius - (axleWidth_ / 2)) * 2048 / PI);
        yawPos_[1] = 2048 - (std::atan2(wheelOffset1_, radius + (axleWidth_ / 2)) * 2048 / PI);
        yawPos_[2] = 2048 - (std::atan2(wheelOffset2_, radius - (axleWidth_ / 2)) * 2048 / PI);
        yawPos_[3] = 2048 - (std::atan2(wheelOffset2_, radius + (axleWidth_ / 2)) * 2048 / PI);
    }
    else if (std::abs(msg.angular.z) > 0.001 && msg.linear.x < 0.001) { // rot. in place
        yawPos_[0] = 2048 + (std::atan2(wheelOffset1_, (axleWidth_ / 2)) * 2048 / PI);
        yawPos_[1] = 2048 - (std::atan2(wheelOffset1_, (axleWidth_ / 2)) * 2048 / PI);
        yawPos_[2] = 2048 + (std::atan2(wheelOffset2_, (axleWidth_ / 2)) * 2048 / PI);
        yawPos_[3] = 2048 - (std::atan2(wheelOffset2_, (axleWidth_ / 2)) * 2048 / PI);
    }
    else {
        yawPos_[0] = 2048;
        yawPos_[1] = 2048;
        yawPos_[2] = 2048;
        yawPos_[3] = 2048;
    }
}

void DynamixelController::gimbal_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    for (int i = 0; i < 4; i++) {
      gimbalPos_[i] = msg->data[i];
    }
}

void DynamixelController::timer_cb() {
    writeYawPosition(yawPos_);
    writeGimbalPosition(gimbalPos_);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelController>());
    rclcpp::shutdown();
    return 0;
}
