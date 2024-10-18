#include <memory>
#include <string>
#include <limits>
#include <vector>
#include "goal_checker/goal_checker.hpp"
#include "angles/angles.h"

using std::placeholders::_1;

/**
 * @brief Get a geometry_msgs Quaternion from a yaw angle
 * @param angle Yaw angle to generate a quaternion from
 * @return geometry_msgs Quaternion
 */
inline geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);  // void returning function
  return tf2::toMsg(q);
}

SimpleGoalChecker::SimpleGoalChecker()
: xy_goal_tolerance_(0.25),
  yaw_goal_tolerance_(0.25),
  xy_goal_tolerance_sq_(0.0625)
{
}

bool SimpleGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  double dx = query_pose.position.x - goal_pose.position.x,
    dy = query_pose.position.y - goal_pose.position.y;
  if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
    return false;
  }
  double dyaw = angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation),
    tf2::getYaw(goal_pose.orientation));
  return fabs(dyaw) < yaw_goal_tolerance_;
}

bool SimpleGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation = orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}