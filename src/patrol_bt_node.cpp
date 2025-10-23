#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "patrol_behavior_tree/bt_actions.hpp"

class PatrolBTNode : public rclcpp::Node {
public:
  PatrolBTNode() : Node("patrol_bt_node") {
    BT::BehaviorTreeFactory factory;
    
    auto node_ptr = shared_from_this();
    
    factory.registerNodeType<patrol_bt::MoveForward>("MoveForward", node_ptr);
    factory.registerNodeType<patrol_bt::RotateRobot>("RotateRobot", node_ptr);
    factory.registerNodeType<patrol_bt::IsObstacleClose>("IsObstacleClose", node_ptr);
    
    std::string pkg_path = ament_index_cpp::get_package_share_directory("patrol_behavior_tree");
    std::string xml_path = pkg_path + "/behavior_trees/patrol_tree.xml";
    
    tree_ = factory.createTreeFromFile(xml_path);
    logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
    
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PatrolBTNode::tickTree, this));
    
    RCLCPP_INFO(get_logger(), "Patrol behavior tree initialized");
  }

private:
  void tickTree() {
    auto status = tree_.tickOnce();
    
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(get_logger(), "Behavior tree completed");
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(get_logger(), "Behavior tree failed");
    }
  }
  
  BT::Tree tree_;
  std::unique_ptr<BT::StdCoutLogger> logger_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolBTNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}