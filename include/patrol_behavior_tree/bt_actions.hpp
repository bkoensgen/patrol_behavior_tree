#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace patrol_bt {

class MoveForward : public BT::StatefulActionNode {
public:
    MoveForward(const std::string& name, const BT::NodeConfig& config,
                rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("distance") };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    double target_distance_;
    double distance_traveled_;
    rclcpp::Time start_time_;
};

class RotateRobot : public BT::StatefulActionNode {
public:
    RotateRobot(const std::string& name, const BT::NodeConfig& config,
                rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("angle") };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    double target_angle_;
    double angle_rotated_;
    rclcpp::Time start_time_;
};

class IsObstacleClose : public BT::SyncActionNode {
public:
    IsObstacleClose(const std::string& name, const BT::NodeConfig& config,
                    rclcpp::Node::SharedPtr node);
    
    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("distance") };
    }

    BT::NodeStatus tick() override;
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    double min_distance_;
    double threshold_distance_;
};

} // namespace patrol_bt
