#include "delivery_motion/run_zlac.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define PI 3.1415926535897932

ZLAC_Twist::ZLAC_Twist() : Node("ZLAC"), front_mots("/dev/ttyUSB0", 115200, 0x01), rear_mots("/dev/ttyUSB1", 115200, 0x01) {
  declare_parameter("WO1", 0.285);
  declare_parameter("WO2", 0.285);
  declare_parameter("AW", 0.453);
  declare_parameter("WD", 0.13);
  get_parameter("WO1", wheelOffset1_);
  get_parameter("WO2", wheelOffset2_);
  get_parameter("AW", axleWidth_);
  get_parameter("WD", wheelDiameter_);

  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_smoothed", 1, std::bind(&ZLAC_Twist::twist_cb, this, _1));
  timer_ = this->create_wall_timer(50ms, std::bind(&ZLAC_Twist::timer_cb, this));
}

ZLAC_Twist::~ZLAC_Twist() {
  front_mots.set_double_rpm(0, 0);
  rear_mots.set_double_rpm(0, 0);
  front_mots.terminate();
  rear_mots.terminate();
}

void ZLAC_Twist::timer_cb() {
  try {
    front_mots.set_double_rpm(motor_rpm_[0], motor_rpm_[1]);
    rear_mots.set_double_rpm(motor_rpm_[2], motor_rpm_[3]);
  }
  catch (const std::exception& e) {
    std::cerr << typeid(e).name() << std::endl;
    std::cerr << e.what() << std::endl;
  }
}

void ZLAC_Twist::twist_cb(const geometry_msgs::msg::Twist msg) {
  double v = msg.linear.x;
  double omega = msg.angular.z;
  double radius = std::abs(v / omega);
  double wheel_omega[4];

  wheel_omega[0] = v * sqrt(pow((radius - axleWidth_ / 2), 2) + pow((wheelOffset1_ / 2), 2)) / radius / (wheelDiameter_ / 2);
  wheel_omega[1] = v * sqrt(pow((radius + axleWidth_ / 2), 2) + pow((wheelOffset1_ / 2), 2)) / radius / (wheelDiameter_ / 2);
  wheel_omega[2] = v * sqrt(pow((radius - axleWidth_ / 2), 2) + pow((wheelOffset2_ / 2), 2)) / radius / (wheelDiameter_ / 2);
  wheel_omega[3] = v * sqrt(pow((radius + axleWidth_ / 2), 2) + pow((wheelOffset2_ / 2), 2)) / radius / (wheelDiameter_ / 2);

  if (std::abs(v) < 0.01 && std::abs(omega) > 0.01) { // rot. in place
    motor_rpm_[0] = sqrt(pow(wheelOffset1_, 2) + pow(axleWidth_ / 2, 2)) * omega / (wheelDiameter_ / 2) * 30 / PI;
    motor_rpm_[1] = -1 * motor_rpm_[0];
    motor_rpm_[2] = sqrt(pow(wheelOffset2_, 2) + pow(axleWidth_ / 2, 2)) * omega / (wheelDiameter_ / 2) * 30 / PI;
    motor_rpm_[3] = -1 * motor_rpm_[2];
  }
  else if (omega < -0.01) {
    motor_rpm_[0] = wheel_omega[0] * 30 / PI;
    motor_rpm_[1] = wheel_omega[1] * 30 / PI;
    motor_rpm_[2] = -1 * wheel_omega[3] * 30 / PI;
    motor_rpm_[3] = -1 * wheel_omega[2] * 30 / PI;
  }
  else if (omega > 0.01) {
    motor_rpm_[0] = wheel_omega[1] * 30 / PI;
    motor_rpm_[1] = wheel_omega[0] * 30 / PI;
    motor_rpm_[2] = -1 * wheel_omega[2] * 30 / PI;
    motor_rpm_[3] = -1 * wheel_omega[3] * 30 / PI;
  }
  else {
    double rpm = (60 * v) / (wheelDiameter_ * PI);
    motor_rpm_[0] = rpm;
    motor_rpm_[1] = rpm;
    motor_rpm_[2] = -1 * rpm;
    motor_rpm_[3] = -1 * rpm;
  }
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZLAC_Twist>());
  rclcpp::shutdown();
  return 0;
}
