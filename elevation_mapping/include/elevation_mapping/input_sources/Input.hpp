/*
 *  Input.hpp
 *
 *  Created on: Oct 06, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <string>

#include "elevation_mapping/ThreadSafeDataWrapper.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

namespace elevation_mapping {
class ElevationMapping;  // Forward declare to avoid cyclic import dependency.

/**
 * @brief An Input feeds data to ElevationMapping callbacks. E.g it holds a subscription to an sensor source and registered an appropriate
 * ElevationMapping callback.
 */
class Input {
 public:
  template <typename MsgT>
  using CallbackT = void (ElevationMapping::*)(const std::shared_ptr<const MsgT>&, bool, const SensorProcessorBase::Ptr&);

  /**
   * @brief Constructor.
   * @param node Reference to the nodeHandle of the manager. Used to subscribe
   * to inputs.
   */
  explicit Input(rclcpp::Node::SharedPtr node);

  /**
   * Whether the input source is enabled or not.
   */
  bool isEnabled() const {
    const Parameters parameters{parameters_.getData()};
    return parameters.isEnabled_;
  }

  /**
   * @brief Configure the input source.
   * @param name Name of this input source.
   * @param inputSourcesNamespace Namespace under which the input source configuration is located.
   * @param generalSensorProcessorParameters Parameters shared by all sensor processors.
   * @return True if configuring was successful.
   */
  bool configure(std::string name, const std::string& inputSourcesNamespace,
                 const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters);

  /**
   * @brief Registers the corresponding callback in the elevationMap.
   * @param callback The callback to use for incoming data.
   * @tparam MsgT The message types of the callback.
   */
  template <typename MsgT>
  void registerCallback(CallbackT<MsgT> callback);

  /**
   * @return The topic (as absolute path, with renames) that this input
   * subscribes to.
   */
  std::string getSubscribedTopic() const;

  /**
   * @return The type of this input source.
   */
  std::string getType() {
    const Parameters parameters{parameters_.getData()};
    return parameters.type_;
  }

 private:
  /**
   * @brief Configures the used sensor processor from the given parameters.
   * @param name The name of this input source
   * @param parameters The parameters of this input source
   * @param generalSensorProcessorParameters  General parameters needed for the sensor processor that are not specific to this sensor
   * processor.
   * @return True if successful.
   */
  bool configureSensorProcessor(std::string name, const XmlRpc::XmlRpcValue& parameters,
                                const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters);

  // ROS connection.
  rclcpp::SubscriptionBase::SharedPtr subscriber_;
  rclcpp::Node::SharedPtr node_;

  //! Sensor processor
  SensorProcessorBase::Ptr sensorProcessor_;

  // Parameters.
  struct Parameters {
    std::string name_;
    std::string type_;
    bool isEnabled_{true};
    uint32_t queueSize_{0};
    std::string topic_;
    bool publishOnUpdate_{true};
  };
  ThreadSafeDataWrapper<Parameters> parameters_;
};

template <typename MsgT>
void Input::registerCallback(ElevationMapping& map, CallbackT<MsgT> callback) {
  const Parameters parameters{parameters_.getData()};
  subscriber_ = node_->create_subscription<MsgT>(
      parameters.topic_, parameters.queueSize_,
      [callback, &map, this](const std::shared_ptr<const MsgT> msg) {
        (map.*callback)(msg, parameters.publishOnUpdate_, sensorProcessor_);
      });
  RCLCPP_INFO(rclcpp::get_logger("ElevationMapping"), "Subscribing to %s: %s, queue_size: %i.", parameters.type_.c_str(), parameters.topic_.c_str(), parameters.queueSize_);
}

}  // namespace elevation_mapping