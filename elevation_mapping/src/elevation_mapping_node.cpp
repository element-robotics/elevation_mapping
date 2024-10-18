/*
 * elevation_map_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "rclcpp/rclcpp.hpp"
#include "elevation_mapping/ElevationMapping.hpp"

int main(int argc, char** argv) {
  std::cout << "Starting elevation_mapping_node..." << std::endl;
  rclcpp::init(argc, argv);
  std::cout << "rclcpp initialized." << std::endl;
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  std::cout << "Executor created." << std::endl;
  auto elevation_mapping_node = std::make_shared<elevation_mapping::ElevationMapping>();
  std::cout << "ElevationMapping node created." << std::endl;
  elevation_mapping_node->configure();
  std::cout << "Node configured." << std::endl;
  executor.add_node(elevation_mapping_node);
  std::cout << "Node added to executor." << std::endl;
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
