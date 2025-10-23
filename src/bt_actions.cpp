#include "patrol_behavior_tree/bt_actions.hpp"
#include <cmath>

namespace patrol_bt {

// MoveForward implementation
MoveForward::MoveForward(const std::string& name, const BT::NodeConfig& config,
                         rclcpp::Node::SharedPtr node)
  : StatefulActionNode(name, config), node_(node), distance_traveled_(0.0) {
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

BT::NodeStatus MoveForward::onStart() {
  if (!getInput("distance", target_distance_)) {
    throw BT::RuntimeError("Missing required input [distance]");
  }
  
  distance_traveled_ = 0.0;
  start_time_ = node_->get_clock()->now();
  
  RCLCPP_INFO(node_->get_logger(), "Moving forward %.2f meters", target_distance_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveForward::onRunning() {
  const double speed = 0.2;
  auto current_time = node_->get_clock()->now();
  double elapsed = (current_time - start_time_).seconds();
  distance_traveled_ = speed * elapsed;
  
  auto cmd = geometry_msgs::msg::Twist();
  cmd.linear.x = speed;
  cmd_vel_pub_->publish(cmd);
  
  if (distance_traveled_ >= target_distance_) {
    cmd.linear.x = 0.0;
    cmd_vel_pub_->publish(cmd);
    RCLCPP_INFO(node_->get_logger(), "Completed forward motion: %.2f m", distance_traveled_);
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::RUNNING;
}

void MoveForward::onHalted() {
  auto cmd = geometry_msgs::msg::Twist();
  cmd.linear.x = 0.0;
  cmd_vel_pub_->publish(cmd);
  RCLCPP_INFO(node_->get_logger(), "Forward motion halted");
}

// RotateRobot implementation
RotateRobot::RotateRobot(const std::string& name, const BT::NodeConfig& config,
                         rclcpp::Node::SharedPtr node)
  : StatefulActionNode(name, config), node_(node), angle_rotated_(0.0) {
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

BT::NodeStatus RotateRobot::onStart() {
  if (!getInput("angle", target_angle_)) {
    throw BT::RuntimeError("Missing required input [angle]");
  }
  
  target_angle_ = target_angle_ * M_PI / 180.0;
  angle_rotated_ = 0.0;
  start_time_ = node_->get_clock()->now();
  
  RCLCPP_INFO(node_->get_logger(), "Rotating %.2f degrees", target_angle_ * 180.0 / M_PI);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RotateRobot::onRunning() {
  const double angular_speed = 0.5;
  auto current_time = node_->get_clock()->now();
  double elapsed = (current_time - start_time_).seconds();
  angle_rotated_ = angular_speed * elapsed;
  
  auto cmd = geometry_msgs::msg::Twist();
  cmd.angular.z = angular_speed;
  cmd_vel_pub_->publish(cmd);
  
  if (angle_rotated_ >= std::abs(target_angle_)) {
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
    RCLCPP_INFO(node_->get_logger(), "Completed rotation: %.2f deg", 
                angle_rotated_ * 180.0 / M_PI);
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::RUNNING;
}

void RotateRobot::onHalted() {
  auto cmd = geometry_msgs::msg::Twist();
  cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
  RCLCPP_INFO(node_->get_logger(), "Rotation halted");
}

// IsObstacleClose implementation
IsObstacleClose::IsObstacleClose(const std::string& name, const BT::NodeConfig& config,
                                 rclcpp::Node::SharedPtr node)
  : ConditionNode(name, config), node_(node), min_distance_(10.0) {
  
  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&IsObstacleClose::laserCallback, this, std::placeholders::_1));
}

void IsObstacleClose::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  min_distance_ = 10.0;
  for (const auto& range : msg->ranges) {
    if (std::isfinite(range) && range < min_distance_) {
      min_distance_ = range;
    }
  }
}

BT::NodeStatus IsObstacleClose::tick() {
  if (!getInput("distance", threshold_distance_)) {
    threshold_distance_ = 0.5;
  }
  
  if (min_distance_ < threshold_distance_) {
    RCLCPP_WARN(node_->get_logger(), "Obstacle at %.2f m", min_distance_);
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::FAILURE;
}

} // namespace patrol_bt