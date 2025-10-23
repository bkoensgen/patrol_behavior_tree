#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "patrol_behavior_tree/bt_actions.hpp"

class BTActionsTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
  }
  
  void TearDown() override {
    rclcpp::shutdown();
  }
  
  rclcpp::Node::SharedPtr node_;
};

TEST_F(BTActionsTest, MoveForwardInitializes) {
  BT::NodeConfig config;
  patrol_bt::MoveForward action("test", config, node_);
  SUCCEED();
}

TEST_F(BTActionsTest, IsObstacleCloseInitializes) {
  BT::NodeConfig config;
  patrol_bt::IsObstacleClose condition("test", config, node_);
  SUCCEED();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}