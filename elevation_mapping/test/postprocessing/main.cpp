#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("elevation_mapping");
  ros::start();  // To make use of ROS time in output macros.
  testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
