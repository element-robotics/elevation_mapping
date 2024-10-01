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
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  auto elevation_node = std::make_shared<elevation_mapping::ElevationMapping>();
  executor.add_node(elevation_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
