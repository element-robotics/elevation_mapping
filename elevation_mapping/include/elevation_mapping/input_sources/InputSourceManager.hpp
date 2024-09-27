/*
 *  InputSourceManager.hpp
 *
 *  Created on: Oct 02, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include "elevation_mapping/input_sources/Input.hpp"

#include <XmlRpc.h>
#include "rclcpp/rclcpp.hpp"

namespace elevation_mapping {
class ElevationMapping;  // Forward declare to avoid cyclic import dependency.

/**
 * @brief An input source manager reads a list of input sources from the configuration and connects them to the appropriate callback of
 * elevation mapping.
 */
class InputSourceManager {
 public:
  /**
   * @brief Constructor.
   * @param nodeHandle Used to resolve the namespace and setup the subscribers.
   */
  explicit InputSourceManager(const rclcpp::Node& nodeHandle);

  /**
   * @brief Configure the input sources from a configuration stored on the
   * parameter server under inputSourcesNamespace.
   * @param inputSourcesNamespace The namespace of the subscribers list to load.
   * @return True if configuring was successful.
   */
  bool configureFromRos(const std::string& inputSourcesNamespace);

  /**
   * @brief Configure the input sources.
   * This will configure all managed input sources.
   * @param config The list of input source parameters.
   * @param sourceConfigurationName The name of the input source configuration.
   * @return True if configuring was successful.
   */
  bool configure(const XmlRpc::XmlRpcValue& config, const std::string& sourceConfigurationName);

  /**
   * @brief Registers the corresponding callback in the elevationMap.
   * @param map The map we want to link the input sources to.
   * @param callbacks pairs of callback type strings and their corresponding
   * callback. E.g: std::make_pair("pointcloud",
   * &ElevationMap::pointCloudCallback), std::make_pair("depthimage",
   * &ElevationMap::depthImageCallback)
   * @tparam MsgT The message types of the callbacks
   * @return True if registering was successful.
   */
  template <typename... MsgT>
  bool registerCallbacks(ElevationMapping& map, std::pair<const char*, Input::CallbackT<MsgT>>... callbacks);

  /**
   * @return The number of successfully configured input sources.
   */
  int getNumberOfSources();

 protected:
  //! A list of input sources.
  std::vector<Input> sources_;

  //! Node handle to load.
  rclcpp::Node nodeHandle_;
};

// Template definitions

template <typename... MsgT>
bool InputSourceManager::registerCallbacks(ElevationMapping& map, std::pair<const char*, Input::CallbackT<MsgT>>... callbacks) {
  if (sources_.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("ElevationMapping"), "Not registering any callbacks, no input sources given. Did you configure the InputSourceManager?");
    return true;
  }
  for (Input& source : sources_) {
    bool callbackRegistered = false;
    for (auto& callback : {callbacks...}) {
      if (source.getType() == callback.first) {
        source.registerCallback(map, callback.second);
        callbackRegistered = true;
      }
    }
    if (not callbackRegistered) {
      RCLCPP_WARN(rclcpp::get_logger("ElevationMapping"), "The configuration contains input sources of an unknown type: %s", source.getType().c_str());
      RCLCPP_WARN(rclcpp::get_logger("ElevationMapping"), "Available types are:");
      for (auto& callback : {callbacks...}) {
        RCLCPP_WARN(rclcpp::get_logger("ElevationMapping"), "- %s", callback.first);
      }
      return false;
    }
  }
  return true;
}

}  // namespace elevation_mapping
