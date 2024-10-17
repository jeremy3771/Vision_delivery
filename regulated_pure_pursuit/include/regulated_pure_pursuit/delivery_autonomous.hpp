#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "regulated_pure_pursuit/regulated_pure_pursuit.hpp"

class delivery_autonomous : public rclcpp::Node {
public:
  delivery_autonomous();
  ~delivery_autonomous() = default;
private:
  std::shared_ptr<RegulatedPurePursuitController> controller_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr path_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
};