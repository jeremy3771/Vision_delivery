#include "regulated_pure_pursuit/delivery_autonomous.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

delivery_autonomous::delivery_autonomous() : Node("delivery_autonomous") {
  vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
  controller_ = std::make_shared<RegulatedPurePursuitController>();
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto RPP_node = std::make_shared<RegulatedPurePursuitController>();
    auto node = std::make_shared<delivery_autonomous>();
    rclcpp::spin(RPP_node);
    rclcpp::shutdown();

    return 0;
}