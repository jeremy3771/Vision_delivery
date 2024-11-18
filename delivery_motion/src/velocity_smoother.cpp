#include "delivery_motion/velocity_smoother.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

VelocitySmoother::VelocitySmoother() : Node("velocity_smoother") {
  declare_parameter("smoothing_frequency", 20.0);
  declare_parameter("max_velocity", std::vector<double>{2.0, 0.0, 2.5});
  declare_parameter("min_velocity", std::vector<double>{-2.0, 0.0, -2.5});
  declare_parameter("max_accel", std::vector<double>{0.5, 0.0, 3.0});
  declare_parameter("max_decel", std::vector<double>{-0.5, 0.0, -3.0});
  declare_parameter("deadband_velocity", std::vector<double>{0.0, 0.0, 0.0});
  declare_parameter("velocity_timeout", 1.0);

  double velocity_timeout_dbl;
  get_parameter("smoothing_frequency", smoothing_frequency_);
  get_parameter("max_velocity", max_velocities_);
  get_parameter("min_velocity", min_velocities_);
  get_parameter("max_accel", max_accels_);
  get_parameter("max_decel", max_decels_);
  get_parameter("deadband_velocity", deadband_velocities_);
  get_parameter("velocity_timeout", velocity_timeout_dbl);
  velocity_timeout_ = rclcpp::Duration::from_seconds(velocity_timeout_dbl);

  double timer_duration_ms = 1000.0 / smoothing_frequency_;
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(timer_duration_ms)),
    std::bind(&VelocitySmoother::smootherTimer, this));
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1).best_effort(), std::bind(&VelocitySmoother::twist_cb, this, _1));
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_smoothed", 1);
}

VelocitySmoother::~VelocitySmoother() {
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = 0;
  cmd_vel->linear.y = 0;
  cmd_vel->angular.z = 0;
  publisher_->publish(std::move(cmd_vel));
}

void VelocitySmoother::twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_ = msg;
  last_command_time_ = now();
}

double VelocitySmoother::applyConstraints(
    const double v_curr, const double v_cmd,
    const double accel, const double decel) {
  double dv = v_cmd - v_curr;

  double v_component_max;
  double v_component_min;

  if (abs(v_cmd) >= abs(v_curr) && v_curr * v_cmd >= 0.0) {
    v_component_max = accel / smoothing_frequency_;
    v_component_min = -accel / smoothing_frequency_;
  } else {
    v_component_max = -decel / smoothing_frequency_;
    v_component_min = decel / smoothing_frequency_;
  }

  return v_curr + std::clamp(dv, v_component_min, v_component_max);
}

void VelocitySmoother::smootherTimer() {
  if (!cmd_) {
    return;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();

  if (now() - last_command_time_ > velocity_timeout_) {
    if (last_cmd_ == geometry_msgs::msg::Twist() || stopped_) {
      stopped_ = true;
      return;
    }
    *cmd_ = geometry_msgs::msg::Twist();
  }

  stopped_ = false;

  geometry_msgs::msg::Twist current_ = last_cmd_;

  cmd_->linear.x = std::clamp(
    cmd_->linear.x, min_velocities_[0], max_velocities_[0]);
  cmd_->linear.y = std::clamp(
    cmd_->linear.y, min_velocities_[1], max_velocities_[1]);
  cmd_->angular.z = std::clamp(
    cmd_->angular.z, min_velocities_[2], max_velocities_[2]);

  cmd_vel->linear.x = applyConstraints(
    current_.linear.x, cmd_->linear.x, max_accels_[0], max_decels_[0]);
  cmd_vel->linear.y = applyConstraints(
    current_.linear.y, cmd_->linear.y, max_accels_[1], max_decels_[1]);
  cmd_vel->angular.z = applyConstraints(
    current_.angular.z, cmd_->angular.z, max_accels_[2], max_decels_[2]);
  last_cmd_ = *cmd_vel;

  cmd_vel->linear.x =
    fabs(cmd_vel->linear.x) < deadband_velocities_[0] ? 0.0 : cmd_vel->linear.x;
  cmd_vel->linear.y =
    fabs(cmd_vel->linear.y) < deadband_velocities_[1] ? 0.0 : cmd_vel->linear.y;
  cmd_vel->angular.z =
    fabs(cmd_vel->angular.z) < deadband_velocities_[2] ? 0.0 : cmd_vel->angular.z;

  publisher_->publish(std::move(cmd_vel));
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocitySmoother>());
  rclcpp::shutdown();
  return 0;
}