/*
 *  InputSourceManager.cpp
 *
 *  Created on: Oct 02, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/input_sources/InputSourceManager.hpp"
#include "elevation_mapping/ElevationMapping.hpp"

namespace elevation_mapping {

InputSourceManager::InputSourceManager(const rclcpp::Node::SharedPtr node) : node_(node) {}

bool InputSourceManager::configureFromRos(const std::string& inputSourcesNamespace) {
  node_->declare_parameter<std::vector<std::string>>(inputSourcesNamespace + ".sources");
  std::vector<std::string> inputSources;
  if (!node_->get_parameter(inputSourcesNamespace + ".sources", inputSources) || inputSources.empty()) {
      RCLCPP_WARN(node_->get_logger(), 
          "No input sources specified. No inputs will be configured.");
      return false;
    }
  return configure(inputSourcesNamespace, inputSources);
}

bool InputSourceManager::configure(const std::string& inputSourcesNamespace, const std::vector<std::string>& inputSources) {

  bool successfulConfiguration = true;
  // TODO: Why not an unordered_set?
  std::set<std::string> subscribedTopics;
  //TODO: Proper error checking for parameters here
  SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{node_->declare_parameter<std::string>("robot_base_frame_id", "/base_link"),
                                                                      node_->get_parameter("map_frame_id").as_string()};
  // Configure all input sources in the list.
  for (const auto& inputSource : inputSources) {
    Input source{(node_)};

    const bool configured{source.configure( inputSource, inputSourcesNamespace, generalSensorProcessorConfig)};
    if (!configured) {
      successfulConfiguration = false;
      continue;
    }

    if (!source.isEnabled()) {
      continue;
    }

    const std::string subscribedTopic{source.getSubscribedTopic()};
    const bool topicIsUnique{subscribedTopics.insert(subscribedTopic).second};

    if (topicIsUnique) {
      sources_.push_back(std::move(source));
    } else {
      RCLCPP_WARN(node_->get_logger(), 
          "The input sources specification tried to subscribe to %s "
          "multiple times. Only subscribing once.",
          subscribedTopic.c_str());
      successfulConfiguration = false;
    }
  }

  return successfulConfiguration;
}

int InputSourceManager::getNumberOfSources() {
  return static_cast<int>(sources_.size());
}

}  // namespace elevation_mapping