#include "delivery_motion/run_opencm.hpp"

#include <iostream>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

DynamixelController::DynamixelController() : Node("dynamixel_controller") {
    declare_parameter("WO1", 0.3);
    declare_parameter("WO2", 0.3);
    declare_parameter("AW", 0.643);
    get_parameter("WO1", wheelOffset1_);
    get_parameter("WO2", wheelOffset2_);
    get_parameter("AW", axleWidth_);

    std::fill(motPos_, motPos_ + 4, 2048);

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_smoothed", 10, std::bind(&DynamixelController::twist_cb ,this, _1));
    timer_ = this->create_wall_timer(50ms, std::bind(&DynamixelController::timer_cb, this));
}
void DynamixelController::twist_cb(const geometry_msgs::msg::Twist msg) {
    double radius = std::abs(msg.linear.x / msg.angular.z);

    if (msg.angular.z < -0.001 && radius > 0.01) {
        motPos_[0] = 2048 + (std::atan2(wheelOffset1_, radius + (axleWidth_ / 2)) * 2048 / PI);
        motPos_[1] = 2048 + (std::atan2(wheelOffset1_, radius - (axleWidth_ / 2)) * 2048 / PI);
        motPos_[2] = 2048 - (std::atan2(wheelOffset2_, radius + (axleWidth_ / 2)) * 2048 / PI);
        motPos_[3] = 2048 - (std::atan2(wheelOffset2_, radius - (axleWidth_ / 2)) * 2048 / PI);
    }
    else if (msg.angular.z > 0.001 && radius > 0.01) {
        motPos_[0] = 2048 - (std::atan2(wheelOffset1_, radius - (axleWidth_ / 2)) * 2048 / PI);
        motPos_[1] = 2048 - (std::atan2(wheelOffset1_, radius + (axleWidth_ / 2)) * 2048 / PI);
        motPos_[2] = 2048 + (std::atan2(wheelOffset2_, radius - (axleWidth_ / 2)) * 2048 / PI);
        motPos_[3] = 2048 + (std::atan2(wheelOffset2_, radius + (axleWidth_ / 2)) * 2048 / PI);
    }
    else if (std::abs(msg.angular.z) > 0.001 && msg.linear.x < 0.001) { // rot. in place
        motPos_[0] = 2048 + (std::atan2(wheelOffset1_, (axleWidth_ / 2)) * 2048 / PI);
        motPos_[1] = 2048 - (std::atan2(wheelOffset1_, (axleWidth_ / 2)) * 2048 / PI);
        motPos_[2] = 2048 - (std::atan2(wheelOffset2_, (axleWidth_ / 2)) * 2048 / PI);
        motPos_[3] = 2048 + (std::atan2(wheelOffset2_, (axleWidth_ / 2)) * 2048 / PI);
    }
    else {
        motPos_[0] = 2048;
        motPos_[1] = 2048;
        motPos_[2] = 2048;
        motPos_[3] = 2048;
    }
}
void DynamixelController::timer_cb() {
    writePosition(motPos_);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelController>());
    rclcpp::shutdown();
    return 0;
}
