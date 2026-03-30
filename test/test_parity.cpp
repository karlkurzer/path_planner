#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "planner.h"

using namespace HybridAStar;

class PlannerParityTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("planner_parity_test");
    exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);

    planner_ = std::make_unique<Planner>(node_);
    planner_->initializeLookups();
  }

  void TearDown() override {
    exec_->cancel();
    exec_->remove_node(node_);

    planner_.reset();
    node_.reset();
    exec_.reset();

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::unique_ptr<Planner> planner_;
};

TEST_F(PlannerParityTest, SimpleParityCheck) {
  auto grid_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  grid_msg->info.width = 15;
  grid_msg->info.height = 15;
  grid_msg->info.resolution = 1.0;
  grid_msg->data.assign(225, 0); // 15x15 empty grid

  auto start_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  start_msg->pose.pose.position.x = 2.0;
  start_msg->pose.pose.position.y = 2.0;
  start_msg->pose.pose.orientation.w = 1.0;

  auto goal_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
  goal_msg->pose.position.x = 10.0;
  goal_msg->pose.position.y = 10.0;
  goal_msg->pose.orientation.w = 1.0;

  planner_->setMap(grid_msg);
  planner_->setStart(start_msg);
  planner_->setGoal(goal_msg);

  SUCCEED();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
