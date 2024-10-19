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
            100ms, std::bind(&Robot_Alignment::timer_cb, this));
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::QoS(1).best_effort(), std::bind(&Robot_Alignment::scan_cb, this, _1));
    }

private:
    void timer_cb() {
        printf("Front Dist: %f, Left: %f, Right: %f, Angle: %f\n", front_dist_, left_dist_, right_dist_, angle_);
    }
    void scan_cb(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
		 front_dist_ = msg->ranges[0];
		 left_dist_ = msg->ranges[89];
		 right_dist_ = msg->ranges[269];
		 
		 float right_45deg = msg->ranges[314];
		 float right_45deg_x = right_45deg * std::cos(45 * (PI / 180));
		 float right_45deg_y = right_45deg * std::sin(45 * (PI / 180));
		 
		 float dx = right_45deg_x - right_dist_;
		 float dy = right_45deg_y;
		 angle_ = std::atan2(dy, dx);
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
