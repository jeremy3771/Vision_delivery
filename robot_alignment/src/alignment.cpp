#include <chrono>
#include <thread>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define PI 3.141592

using namespace std::chrono_literals;
using std::placeholders::_1;

class Robot_Alignment : public rclcpp::Node {
public:
    Robot_Alignment() : Node("Robot_Alignment") {
        timer_ = this->create_wall_timer(
            500ms, std::bind(&Robot_Alignment::timer_cb, this));
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::QoS(1).best_effort(), std::bind(&Robot_Alignment::scan_cb, this, _1));
    }

private:
    void timer_cb() {
        printf("Front Dist: %.3f, Right: %.3f, Angle: %.3f\n", front_dist_, right_dist_, angle_);
    }
    void scan_cb(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
		 front_dist_ = msg->ranges[179];
		 right_dist_ = msg->ranges[89];

		 // Change right and left
		 float right_60deg = msg->ranges[149];
		 float right_60deg_x = right_60deg * std::cos(60 * (PI / 180));
		 float right_60deg_y = right_60deg * std::sin(60 * (PI / 180));
		 
		 float dx = right_60deg_x - right_dist_;
		 float dy = right_60deg_y;
		 
		 angle_ = std::atan2(dy, dx) * (180 / PI) - 90;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    float range_ahead_;
    float front_dist_, left_dist_, right_dist_;
    float angle_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot_Alignment>());
    rclcpp::shutdown();
    return 0;
}
