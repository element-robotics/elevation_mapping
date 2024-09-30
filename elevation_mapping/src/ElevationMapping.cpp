/*
 * ElevationMapping.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <cmath>
#include <memory>
#include <string>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rcpputils/asserts.hpp>

#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/ElevationMapping.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

namespace elevation_mapping {

ElevationMapping::ElevationMapping(rclcpp::Node::SharedPtr node)
    : node_(node),
      inputSources_(node_),
      transformListener_(transformBuffer_),
      map_(node),
      robotMotionMapUpdater_(node),
      receivedFirstMatchingPointcloudAndPose_(false) {
#ifndef NDEBUG
  // Print a warning if built in debug.
  RCLCPP_WARN(rclcpp->get_logger("ElevationMapping"), "CMake Build Type is 'Debug'. Change to 'Release' for better performance.");
#endif

  RCLCPP_INFO(rclcpp->get_logger("ElevationMapping"), "Elevation mapping node started.");

  readParameters();
  setupSubscribers();
  setupServices();
  setupTimers();

  initialize();

  RCLCPP_INFO(node->get_logger("ElevationMapping"), "Successfully launched node.");
}

void ElevationMapping::setupSubscribers() {  // Handle deprecated point_cloud_topic and input_sources configuration.
  auto [parameters, parameterGuard]{parameters_.getDataToWrite()};
  const bool configuredInputSources = inputSources_.configureFromRos("input_sources");
  const bool hasDeprecatedPointcloudTopic = node_.hasParam("point_cloud_topic");
  if (hasDeprecatedPointcloudTopic) {
    RCLCPP_WARN(node->get_logger("ElevationMapping"), "Parameter 'point_cloud_topic' is deprecated, please use 'input_sources' instead.");
  }
  if (!configuredInputSources && hasDeprecatedPointcloudTopic) {
    pointCloudSubscriber_ = node_.subscribe<sensor_msgs::msg::PointCloud2>(
        parameters.pointCloudTopic_, 1, [&](const auto& msg) { pointCloudCallback(msg, true, sensorProcessor_); });
  }
  if (configuredInputSources) {
    inputSources_.registerCallbacks(*this, make_pair("pointcloud", &ElevationMapping::pointCloudCallback));
  }

  if (!parameters.robotPoseTopic_.empty()) {
    robotPoseSubscriber_.subscribe(node_, parameters.robotPoseTopic_, 1);
    robotPoseCache_.connectInput(robotPoseSubscriber_);
    robotPoseCache_.setCacheSize(parameters.robotPoseCacheSize_);
  } else {
    parameters.ignoreRobotMotionUpdates_ = true;
  }
}

void ElevationMapping::setupServices() {
  const Parameters parameters{parameters_.getData()};
  // Multi-threading for fusion.
  ros::AdvertiseServiceOptions advertiseServiceOptionsForTriggerFusion = ros::AdvertiseServiceOptions::create<std_srvs::srv::Empty>(
      "trigger_fusion", [&](auto& req, auto& res) { return fuseEntireMapServiceCallback(req, res); }, ros::VoidConstPtr(),
      &fusionServiceQueue_);
  fusionTriggerService_ = node_.advertiseService(advertiseServiceOptionsForTriggerFusion);

  ros::AdvertiseServiceOptions advertiseServiceOptionsForGetFusedSubmap = ros::AdvertiseServiceOptions::create<grid_map_msgs::srv::GetGridMap>(
      "get_submap", [&](auto& req, auto& res) { return getFusedSubmapServiceCallback(req, res); }, ros::VoidConstPtr(),
      &fusionServiceQueue_);
  fusedSubmapService_ = node_.advertiseService(advertiseServiceOptionsForGetFusedSubmap);

  ros::AdvertiseServiceOptions advertiseServiceOptionsForGetRawSubmap = ros::AdvertiseServiceOptions::create<grid_map_msgs::srv::GetGridMap>(
      "get_raw_submap", [&](auto& req, auto& res) { return getRawSubmapServiceCallback(req, res); }, ros::VoidConstPtr(),
      &fusionServiceQueue_);
  rawSubmapService_ = node_.advertiseService(advertiseServiceOptionsForGetRawSubmap);

  clearMapService_ = node_.advertiseService("clear_map", &ElevationMapping::clearMapServiceCallback, this);
  enableUpdatesService_ = node_.advertiseService("enable_updates", &ElevationMapping::enableUpdatesServiceCallback, this);
  disableUpdatesService_ = node_.advertiseService("disable_updates", &ElevationMapping::disableUpdatesServiceCallback, this);
  maskedReplaceService_ = node_.advertiseService("masked_replace", &ElevationMapping::maskedReplaceServiceCallback, this);
  saveMapService_ = node_.advertiseService("save_map", &ElevationMapping::saveMapServiceCallback, this);
  loadMapService_ = node_.advertiseService("load_map", &ElevationMapping::loadMapServiceCallback, this);
  reloadParametersService_ = node_.advertiseService("reload_parameters", &ElevationMapping::reloadParametersServiceCallback, this);
}

void ElevationMapping::setupTimers() {
  const Parameters parameters{parameters_.getData()};
  ElevationMap::Parameters mapParameters{map_.parameters_.getData()};
  mapUpdateTimer_ = node_.createTimer(parameters.maxNoUpdateDuration_, &ElevationMapping::mapUpdateTimerCallback, this, true, false);

  if (!parameters.fusedMapPublishTimerDuration_.isZero()) {
    rclcpp::TimerOptions timerOptions = rclcpp::TimerOptions(
        parameters.fusedMapPublishTimerDuration_, [this](auto&& PH1) { publishFusedMapCallback(PH1); }, &fusionServiceQueue_, false, false);
    fusedMapPublishTimer_ = node_.createTimer(timerOptions);
  }

  // Multi-threading for visibility cleanup. Visibility clean-up does not help when continuous clean-up is enabled.
  if (mapParameters.enableVisibilityCleanup_ && !parameters.visibilityCleanupTimerDuration_.isZero() &&
      !mapParameters.enableContinuousCleanup_) {
    rclcpp::TimerOptions timerOptions = rclcpp::TimerOptions(
        parameters.visibilityCleanupTimerDuration_, [this](auto&& PH1) { visibilityCleanupCallback(PH1); }, &visibilityCleanupQueue_, false,
        false);
    visibilityCleanupTimer_ = node_.createTimer(timerOptions);
  }
}

ElevationMapping::~ElevationMapping() {
  // Shutdown all services.

  {  // Fusion Service Queue
    rawSubmapService_.shutdown();
    fusionTriggerService_.shutdown();
    fusedSubmapService_.shutdown();
    fusedMapPublishTimer_.stop();

    fusionServiceQueue_.disable();
    fusionServiceQueue_.clear();
  }

  {  // Visibility cleanup queue
    visibilityCleanupTimer_.stop();

    visibilityCleanupQueue_.disable();
    visibilityCleanupQueue_.clear();
  }

  node_.shutdown();

  // Join threads.
  if (fusionServiceThread_.joinable()) {
    fusionServiceThread_.join();
  }
  if (visibilityCleanupThread_.joinable()) {
    visibilityCleanupThread_.join();
  }
}

// TODO: Seperate this function out to a declareParameters function and a readParameters function. Setup a set_parameters_callback.
bool ElevationMapping::readParameters(bool reload) {
  // Using getDataToWrite gets a write-lock on the parameters thereby blocking all reading threads for the whole scope of this
  // readParameters, which is desired.
  auto [parameters, parametersGuard] = parameters_.getDataToWrite();
  auto [mapParameters, mapParametersGuard] = map_.parameters_.getDataToWrite();

  parameters.pointCloudTopic_ = node_->declare_parameter<std::string>("point_cloud_topic", "/points");
  parameters.robotPoseTopic_ = node_->declare_parameter<std::string>("robot_pose_with_covariance_topic", "/pose");
  parameters.trackPointFrameId_ = node_->declare_parameter<std::string>("track_point_frame_id", "/robot");
  parameters.trackPoint_.x() = node_->declare_parameter<double>("track_point_x", 0.0);
  parameters.trackPoint_.y() = node_->declare_parameter<double>("track_point_y", 0.0);
  parameters.trackPoint_.z() = node_->declare_parameter<double>("track_point_z", 0.0);

  parameters.robotPoseCacheSize_ = node_->declare_parameter<int>("robot_pose_cache_size", 200);
  
  // TODO: Add ROS2 version of this
  // ROS_ASSERT(parameters.robotPoseCacheSize_ >= 0);
  rcpputils::assert_true(parameters.robotPoseCacheSize_ >= 0);

  double minUpdateRate{2.0};
  minUpdateRate = node_->declare_parameter<double>("min_update_rate", minUpdateRate);
  if (minUpdateRate == 0.0) {
    parameters.maxNoUpdateDuration_ = rclcpp::Duration(0.0);
    RCLCPP_WARN(node_->get_logger("ElevationMapping"), "Rate for publishing the map is zero.");
  } else {
    parameters.maxNoUpdateDuration_ = rclcpp::Duration::from_seconds(1.0 / minUpdateRate, 0.0);
  }
  //TODO: what is the point of this
  rcpputils::assert_true(parameters.maxNoUpdateDuration_ > rclcpp::Duration(0.0));

  double timeTolerance{0.0};
  timeTolerance = node_->declare_parameter<double>("time_tolerance", timeTolerance);
  parameters.timeTolerance_ = rclcpp::Duration::from_seconds(timeTolerance);

  double fusedMapPublishingRate{1.0};
  fusedMapPublishingRate = node_->declare_parameter<double>("fused_map_publishing_rate", fusedMapPublishingRate);
  if (fusedMapPublishingRate == 0.0) {
    parameters.fusedMapPublishTimerDuration_ = rclcpp::Duration(0.0);
    RCLCPP_WARN(node_->get_logger("ElevationMapping"), 
        "Rate for publishing the fused map is zero. The fused elevation map will not be published unless the service `triggerFusion` is "
        "called.");
  } else if (std::isinf(fusedMapPublishingRate)) {
    parameters.isContinuouslyFusing_ = true;
    parameters.fusedMapPublishTimerDuration_ = rclcpp::Duration(0.0);
  } else {
    parameters.fusedMapPublishTimerDuration_ = rclcpp::Duration::from_seconds(1.0 / fusedMapPublishingRate);
  }

  double visibilityCleanupRate{1.0};
  visibilityCleanupRate = node_->declare_parameter<double>("visibility_cleanup_rate", visibilityCleanupRate);
  if (visibilityCleanupRate == 0.0) {
    parameters.visibilityCleanupTimerDuration_ = rclcpp::Duration(0.0);
    RCLCPP_WARN(node->get_logger("ElevationMapping"), "Rate for visibility cleanup is zero and therefore disabled.");
  } else {
    parameters.visibilityCleanupTimerDuration_ = rclcpp::Duration::from_seconds(1.0 / visibilityCleanupRate);
    mapParameters.visibilityCleanupDuration_ = 1.0 / visibilityCleanupRate;
  }

  // ElevationMap parameters. TODO Move this to the elevation map class.
  parameters.mapFrameId_ = node_->declare_parameter<std::string>("map_frame_id", "/map");

  grid_map::Length length;
  grid_map::Position position;
  double resolution{0.01};
  length(0) = node_->declare_parameter<double>("length_in_x", 1.5);
  length(1) = node_->declare_parameter<double>("length_in_y", 1.5);
  position.x() = node_->declare_parameter<double>("position_x", 0.0);
  position.y() = node_->declare_parameter<double>("position_y", 0.0);
  resolution = node_->declare_parameter<double>("resolution", resolution);

  // TODO: Use set_parameter_callback instead of this
  if (!reload) {
    map_.setFrameId(parameters.mapFrameId_);
    map_.setGeometry(length, resolution, position);
  }

  mapParameters.minVariance_ = node_->declare_parameter<double>("min_variance", pow(0.003, 2));
  mapParameters.maxVariance_ = node_->declare_parameter<double>("max_variance", pow(0.03, 2));
  mapParameters.mahalanobisDistanceThreshold_ = node_->declare_parameter<double>("mahalanobis_distance_threshold", 2.5);
  mapParameters.multiHeightNoise_ = node_->declare_parameter<double>("multi_height_noise", pow(0.003, 2));
  mapParameters.minHorizontalVariance_ = node_->declare_parameter<double>("min_horizontal_variance", pow(resolution / 2.0, 2));  // two-sigma
  mapParameters.maxHorizontalVariance_ = node_->declare_parameter<double>("max_horizontal_variance", 0.5);
  mapParameters.underlyingMapTopic_ = node_->declare_parameter<std::string>("underlying_map_topic", std::string());
  mapParameters.enableVisibilityCleanup_ = node_->declare_parameter<bool>("enable_visibility_cleanup", true);
  mapParameters.enableContinuousCleanup_ = node_->declare_parameter<bool>("enable_continuous_cleanup", false);
  mapParameters.scanningDuration_ = node_->declare_parameter<double>("scanning_duration", 1.0);
  mapParameters.increaseHeightAlpha_ = node_->declare_parameter<double>("increase_height_alpha", 0.0);
  parameters.maskedReplaceServiceMaskLayerName_ = node_->declare_parameter<std::string>("masked_replace_service_mask_layer_name", std::string("mask"));

  // Settings for initializing elevation map
  parameters.initializeElevationMap_ = node_->declare_parameter<bool>("initialize_elevation_map", false);
  parameters.initializationMethod_ = node_->declare_parameter<int>("initialization_method", 0);
  parameters.lengthInXInitSubmap_ = node_->declare_parameter<double>("length_in_x_init_submap", 1.2);
  parameters.lengthInYInitSubmap_ = node_->declare_parameter<double>("length_in_y_init_submap", 1.8);
  parameters.initSubmapHeightOffset_ = node_->declare_parameter<double>("init_submap_height_offset", 0.0);
  parameters.initSubmapVariance_ = node_->declare_parameter<double>("init_submap_variance", 0.01);
  parameters.targetFrameInitSubmap_ = node_->declare_parameter<std::string>("target_frame_init_submap", std::string("/footprint"));

  // SensorProcessor parameters. Deprecated, use the sensorProcessor from within input sources instead!
  std::string sensorType = node_->declare_parameter<std::string>("sensor_processor/type", "structured_light");

  SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{node_->declare_parameter<std::string>("robot_base_frame_id", "/robot"),
                                                                      parameters.mapFrameId_};
  if (sensorType == "structured_light") {
    sensorProcessor_ = std::make_unique<StructuredLightSensorProcessor>(node_, generalSensorProcessorConfig);
  } else if (sensorType == "stereo") {
    sensorProcessor_ = std::make_unique<StereoSensorProcessor>(node_, generalSensorProcessorConfig);
  } else if (sensorType == "laser") {
    sensorProcessor_ = std::make_unique<LaserSensorProcessor>(node_, generalSensorProcessorConfig);
  } else if (sensorType == "perfect") {
    sensorProcessor_ = std::make_unique<PerfectSensorProcessor>(node_, generalSensorProcessorConfig);
  } else {
    RCLCPP_ERROR(node_->get_logger("ElevationMapping"), "The sensor type %s is not available.", sensorType.c_str());
  }
  // TODO: modify these functions to use node based parameters
  if (!sensorProcessor_->readParameters()) {
    return false;
  }
  if (!robotMotionMapUpdater_.readParameters()) {
    return false;
  }

  return true;
}

bool ElevationMapping::initialize() {
  RCLCPP_INFO(node->get_logger("ElevationMapping"), "Elevation mapping node initializing ... ");
  fusionServiceThread_ = boost::thread(&ElevationMapping::runFusionServiceThread, this);
  rclcpp::Duration(1.0).sleep();  // Need this to get the TF caches fill up.
  resetMapUpdateTimer();
  fusedMapPublishTimer_.start();
  visibilityCleanupThread_ = boost::thread([this] { visibilityCleanupThread(); });
  visibilityCleanupTimer_.start();
  initializeElevationMap();
  return true;
}

void ElevationMapping::runFusionServiceThread() {
  rclcpp::Rate loopRate(20);

  while (node_.ok()) {
    fusionServiceQueue_.callAvailable();

    // Sleep until the next execution.
    loopRate.sleep();
  }
}

void ElevationMapping::visibilityCleanupThread() {
  rclcpp::Rate loopRate(20);

  while (node_.ok()) {
    visibilityCleanupQueue_.callAvailable();

    // Sleep until the next execution.
    loopRate.sleep();
  }
}

void ElevationMapping::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointCloudMsg, bool publishPointCloud,
                                          const SensorProcessorBase::Ptr& sensorProcessor_) {
  const Parameters parameters{parameters_.getData()};
  RCLCPP_DEBUG(node->get_logger("ElevationMapping"), "Processing data from: %s", pointCloudMsg->header.frame_id.c_str());
  if (!parameters.updatesEnabled_) {
    ROS_WARN_THROTTLE(10, "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    if (publishPointCloud) {
      map_.setTimestamp(rclcpp::Time::now());
      map_.postprocessAndPublishRawElevationMap();
    }
    return;
  }

  // Check if point cloud has corresponding robot pose at the beginning
  if (!receivedFirstMatchingPointcloudAndPose_) {
    const double oldestPoseTime = robotPoseCache_.getOldestTime().toSec();
    const double currentPointCloudTime = pointCloudMsg->header.stamp.toSec();

    if (currentPointCloudTime < oldestPoseTime) {
      ROS_WARN_THROTTLE(5, "No corresponding point cloud and pose are found. Waiting for first match. (Warning message is throttled, 5s.)");
      return;
    } else {
      RCLCPP_INFO(node->get_logger("ElevationMapping"), "First corresponding point cloud and pose found, elevation mapping started. ");
      receivedFirstMatchingPointcloudAndPose_ = true;
    }
  }

  stopMapUpdateTimer();

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud.
  // TODO(max): Double check with http://wiki.ros.org/hydro/Migration
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pointCloudMsg, pcl_pc);

  PointCloudType::Ptr pointCloud(new PointCloudType);
  pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
  lastPointCloudUpdateTime_.fromNSec(1000 * pointCloud->header.stamp);

  RCLCPP_DEBUG(node->get_logger("ElevationMapping"), "ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));

  // Get robot pose covariance matrix at timestamp of point cloud.
  Eigen::Matrix<double, 6, 6> robotPoseCovariance;
  robotPoseCovariance.setZero();
  if (!parameters.ignoreRobotMotionUpdates_) {
    std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped const> poseMessage =
        robotPoseCache_.getElemBeforeTime(lastPointCloudUpdateTime_);
    if (!poseMessage) {
      // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
      if (robotPoseCache_.getOldestTime().toSec() > lastPointCloudUpdateTime_.toSec()) {
        RCLCPP_ERROR(node->get_logger("ElevationMapping"), "The oldest pose available is at %f, requested pose at %f", robotPoseCache_.getOldestTime().toSec(),
                  lastPointCloudUpdateTime_.toSec());
      } else {
        RCLCPP_ERROR(node->get_logger("ElevationMapping"), "Could not get pose information from robot for time %f. Buffer empty?", lastPointCloudUpdateTime_.toSec());
      }
      return;
    }
    robotPoseCovariance = Eigen::Map<const Eigen::MatrixXd>(poseMessage->pose.covariance.data(), 6, 6);
  }

  // Process point cloud.
  PointCloudType::Ptr pointCloudProcessed(new PointCloudType);
  Eigen::VectorXf measurementVariances;
  if (!sensorProcessor_->process(pointCloud, robotPoseCovariance, pointCloudProcessed, measurementVariances,
                                 pointCloudMsg->header.frame_id)) {
    if (!sensorProcessor_->isTfAvailableInBuffer()) {
      ROS_INFO_THROTTLE(10, "Waiting for tf transformation to be available. (Message is throttled, 10s.)");
      return;
    }
    ROS_ERROR_THROTTLE(10, "Point cloud could not be processed. (Throttled 10s)");
    resetMapUpdateTimer();
    return;
  }

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  // Update map location.
  updateMapLocation();

  // Update map from motion prediction.
  if (!updatePrediction(lastPointCloudUpdateTime_)) {
    RCLCPP_ERROR(node->get_logger("ElevationMapping"), "Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  ElevationMap::Parameters mapParameters{map_.parameters_.getData()};

  // Clear the map if continuous clean-up was enabled.
  if (mapParameters.enableContinuousCleanup_) {
    RCLCPP_DEBUG(node->get_logger("ElevationMapping"), "Clearing elevation map before adding new point cloud.");
    map_.clear();
  }

  // Add point cloud to elevation map.
  if (!map_.add(pointCloudProcessed, measurementVariances, lastPointCloudUpdateTime_,
                Eigen::Affine3d(sensorProcessor_->transformationSensorToMap_))) {
    RCLCPP_ERROR(node->get_logger("ElevationMapping"), "Adding point cloud to elevation map failed.");
    resetMapUpdateTimer();
    return;
  }

  if (publishPointCloud) {
    // Publish elevation map.
    map_.postprocessAndPublishRawElevationMap();
    if (isFusingEnabled()) {
      map_.fuseAll();
      map_.publishFusedElevationMap();
    }
  }

  resetMapUpdateTimer();
}

void ElevationMapping::mapUpdateTimerCallback(const rclcpp::TimerEvent& /*unused*/) {
  const Parameters parameters{parameters_.getData()};
  if (!parameters.updatesEnabled_) {
    ROS_WARN_THROTTLE(10, "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    map_.setTimestamp(rclcpp::Time::now());
    map_.postprocessAndPublishRawElevationMap();
    return;
  }

  rclcpp::Time time = rclcpp::Time::now();
  if ((lastPointCloudUpdateTime_ - time) <=
      parameters.maxNoUpdateDuration_) {  // there were updates from sensordata, no need to force an update.
    return;
  }
  ROS_WARN_THROTTLE(5, "Elevation map is updated without data from the sensor. (Warning message is throttled, 5s.)");

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  stopMapUpdateTimer();

  // Update map from motion prediction.
  if (!updatePrediction(time)) {
    RCLCPP_ERROR(node->get_logger("ElevationMapping"), "Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Publish elevation map.
  map_.postprocessAndPublishRawElevationMap();
  if (isFusingEnabled()) {
    map_.fuseAll();
    map_.publishFusedElevationMap();
  }

  resetMapUpdateTimer();
}

void ElevationMapping::publishFusedMapCallback(const rclcpp::TimerEvent& /*unused*/) {
  if (!map_.hasFusedMapSubscribers()) {
    return;
  }
  RCLCPP_DEBUG(node->get_logger("ElevationMapping"), "Elevation map is fused and published from timer.");
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
}

void ElevationMapping::visibilityCleanupCallback(const rclcpp::TimerEvent& /*unused*/) {
  RCLCPP_DEBUG(node->get_logger("ElevationMapping"), "Elevation map is running visibility cleanup.");
  // Copy constructors for thread-safety.
  map_.visibilityCleanup(rclcpp::Time(lastPointCloudUpdateTime_));
}

bool ElevationMapping::fuseEntireMapServiceCallback(std_srvs::srv::Empty::Request& /*unused*/, std_srvs::srv::Empty::Response& /*unused*/) {
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
  return true;
}

bool ElevationMapping::isFusingEnabled() {
  const Parameters parameters{parameters_.getData()};
  return parameters.isContinuouslyFusing_ && map_.hasFusedMapSubscribers();
}

bool ElevationMapping::updatePrediction(const rclcpp::Time& time) {
  const Parameters parameters{parameters_.getData()};
  if (parameters.ignoreRobotMotionUpdates_) {
    return true;
  }

  RCLCPP_DEBUG(node->get_logger("ElevationMapping"), "Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().toSec());

  if (time + parameters.timeTolerance_ < map_.getTimeOfLastUpdate()) {
    RCLCPP_ERROR(node->get_logger("ElevationMapping"), "Requested update with time stamp %f, but time of last update was %f.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
    return false;
  } else if (time < map_.getTimeOfLastUpdate()) {
    RCLCPP_DEBUG(node->get_logger("ElevationMapping"), "Requested update with time stamp %f, but time of last update was %f. Ignoring update.", time.toSec(),
              map_.getTimeOfLastUpdate().toSec());
    return true;
  }

  // Get robot pose at requested time.
  std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(time);
  if (!poseMessage) {
    // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
    if (robotPoseCache_.getOldestTime().toSec() > lastPointCloudUpdateTime_.toSec()) {
      RCLCPP_ERROR(node->get_logger("ElevationMapping"), "The oldest pose available is at %f, requested pose at %f", robotPoseCache_.getOldestTime().toSec(),
                lastPointCloudUpdateTime_.toSec());
    } else {
      RCLCPP_ERROR(node->get_logger("ElevationMapping"), "Could not get pose information from robot for time %f. Buffer empty?", lastPointCloudUpdateTime_.toSec());
    }
    return false;
  }

  kindr::HomTransformQuatD robotPose;
  kindr_ros::convertFromRosGeometryMsg(poseMessage->pose.pose, robotPose);
  // Covariance is stored in row-major in ROS: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
  Eigen::Matrix<double, 6, 6> robotPoseCovariance =
      Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(poseMessage->pose.covariance.data(), 6, 6);

  // Compute map variance update from motion prediction.
  robotMotionMapUpdater_.update(map_, robotPose, robotPoseCovariance, time);

  return true;
}

bool ElevationMapping::updateMapLocation() {
  const Parameters parameters{parameters_.getData()};
  RCLCPP_DEBUG(node->get_logger("ElevationMapping"), "Elevation map is checked for relocalization.");

  geometry_msgs::msg::PointStamped trackPoint;
  trackPoint.header.frame_id = parameters.trackPointFrameId_;
  trackPoint.header.stamp = rclcpp::Time(0);
  kindr_ros::convertToRosGeometryMsg(parameters.trackPoint_, trackPoint.point);
  geometry_msgs::msg::PointStamped trackPointTransformed;

  try {
    trackPointTransformed = transformBuffer_.transform(trackPoint, map_.getFrameId());
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(node->get_logger("ElevationMapping"), "%s", ex.what());
    return false;
  }

  kindr::Position3D position3d;
  kindr_ros::convertFromRosGeometryMsg(trackPointTransformed.point, position3d);
  grid_map::Position position = position3d.vector().head(2);
  map_.move(position);
  return true;
}

bool ElevationMapping::getFusedSubmapServiceCallback(grid_map_msgs::srv::GetGridMap::Request& request,
                                                     grid_map_msgs::srv::GetGridMap::Response& response) {
  grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
  grid_map::Length requestedSubmapLength(request.length_x, request.length_y);
  RCLCPP_DEBUG(node->get_logger("ElevationMapping"), "Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(), requestedSubmapPosition.y(),
            requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseArea(requestedSubmapPosition, requestedSubmapLength);

  bool isSuccess{false};
  grid_map::Index index;
  grid_map::GridMap subMap = map_.getFusedGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  scopedLock.unlock();

  if (request.layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response.map);
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request.layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response.map);
  }

  RCLCPP_DEBUG(node->get_logger("ElevationMapping"), "Elevation submap responded with timestamp %f.", map_.getTimeOfLastFusion().toSec());
  return isSuccess;
}

bool ElevationMapping::getRawSubmapServiceCallback(grid_map_msgs::srv::GetGridMap::Request& request,
                                                   grid_map_msgs::srv::GetGridMap::Response& response) {
  grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
  grid_map::Length requestedSubmapLength(request.length_x, request.length_y);
  RCLCPP_DEBUG(node->get_logger("ElevationMapping"), "Elevation raw submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(),
            requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  bool isSuccess{false};
  grid_map::Index index;
  grid_map::GridMap subMap = map_.getRawGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  scopedLock.unlock();

  if (request.layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response.map);
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request.layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response.map);
  }
  return isSuccess;
}

bool ElevationMapping::disableUpdatesServiceCallback(std_srvs::srv::Empty::Request& /*request*/, std_srvs::srv::Empty::Response& /*response*/) {
  RCLCPP_INFO(node->get_logger("ElevationMapping"), "Disabling updates.");
  auto [parameters, parameterGuard]{parameters_.getDataToWrite()};
  parameters.updatesEnabled_ = false;
  return true;
}

bool ElevationMapping::enableUpdatesServiceCallback(std_srvs::srv::Empty::Request& /*request*/, std_srvs::srv::Empty::Response& /*response*/) {
  RCLCPP_INFO(node->get_logger("ElevationMapping"), "Enabling updates.");
  auto [parameters, parameterGuard]{parameters_.getDataToWrite()};
  parameters.updatesEnabled_ = true;
  return true;
}

bool ElevationMapping::initializeElevationMap() {
  const Parameters parameters{parameters_.getData()};
  if (parameters.initializeElevationMap_) {
    if (static_cast<elevation_mapping::InitializationMethods>(parameters.initializationMethod_) ==
        elevation_mapping::InitializationMethods::PlanarFloorInitializer) {
      geometry_msgs::msg::TransformStamped transform_msg;
      tf2::Stamped<tf2::Transform> transform;

      // Listen to transform between mapFrameId_ and targetFrameInitSubmap_ and use z value for initialization
      try {
        transform_msg = transformBuffer_.lookupTransform(parameters.mapFrameId_, parameters.targetFrameInitSubmap_, rclcpp::Time(0), rclcpp::Duration(5.0));
        tf2::fromMsg(transform_msg, transform);

        ROS_DEBUG_STREAM("Initializing with x: " << transform.getOrigin().x() << " y: " << transform.getOrigin().y()
                                                 << " z: " << transform.getOrigin().z());

        const grid_map::Position positionRobot(transform.getOrigin().x(), transform.getOrigin().y());

        // Move map before we apply the height values. This prevents unwanted behavior from intermediate move() calls in
        // updateMapLocation().
        map_.move(positionRobot);

        map_.setRawSubmapHeight(positionRobot, transform.getOrigin().z() + parameters.initSubmapHeightOffset_,
                                parameters.initSubmapVariance_, parameters.lengthInXInitSubmap_, parameters.lengthInYInitSubmap_);
        return true;
      } catch (tf2::TransformException& ex) {
        RCLCPP_DEBUG(node->get_logger("ElevationMapping"), "%s", ex.what());
        RCLCPP_WARN(node->get_logger("ElevationMapping"), "Could not initialize elevation map with constant height. (This warning can be ignored if TF tree is not available.)");
        return false;
      }
    }
  }
  return true;
}

bool ElevationMapping::clearMapServiceCallback(std_srvs::srv::Empty::Request& /*request*/, std_srvs::srv::Empty::Response& /*response*/) {
  RCLCPP_INFO(node->get_logger("ElevationMapping"), "Clearing map...");
  bool success = map_.clear();
  success &= initializeElevationMap();
  RCLCPP_INFO(node->get_logger("ElevationMapping"), "Map cleared.");

  return success;
}

bool ElevationMapping::maskedReplaceServiceCallback(grid_map_msgs::srv::SetGridMap::Request& request,
                                                    grid_map_msgs::srv::SetGridMap::Response& /*response*/) {
  const Parameters parameters{parameters_.getData()};
  RCLCPP_INFO(node->get_logger("ElevationMapping"), "Masked replacing of map.");
  grid_map::GridMap sourceMap;
  grid_map::GridMapRosConverter::fromMessage(request.map, sourceMap);

  // Use the supplied mask or do not use a mask
  grid_map::Matrix mask;
  if (sourceMap.exists(parameters.maskedReplaceServiceMaskLayerName_)) {
    mask = sourceMap[parameters.maskedReplaceServiceMaskLayerName_];
  } else {
    mask = Eigen::MatrixXf::Ones(sourceMap.getSize()(0), sourceMap.getSize()(1));
  }

  boost::recursive_mutex::scoped_lock scopedLockRawData(map_.getRawDataMutex());

  // Loop over all layers that should be set
  for (auto sourceLayerIterator = sourceMap.getLayers().begin(); sourceLayerIterator != sourceMap.getLayers().end();
       sourceLayerIterator++) {
    // skip "mask" layer
    if (*sourceLayerIterator == parameters.maskedReplaceServiceMaskLayerName_) {
      continue;
    }
    grid_map::Matrix& sourceLayer = sourceMap[*sourceLayerIterator];
    // Check if the layer exists in the elevation map
    if (map_.getRawGridMap().exists(*sourceLayerIterator)) {
      grid_map::Matrix& destinationLayer = map_.getRawGridMap()[*sourceLayerIterator];
      for (grid_map::GridMapIterator destinationIterator(map_.getRawGridMap()); !destinationIterator.isPastEnd(); ++destinationIterator) {
        // Use the position to find corresponding indices in source and destination
        const grid_map::Index destinationIndex(*destinationIterator);
        grid_map::Position position;
        map_.getRawGridMap().getPosition(*destinationIterator, position);

        if (!sourceMap.isInside(position)) {
          continue;
        }

        grid_map::Index sourceIndex;
        sourceMap.getIndex(position, sourceIndex);
        // If the mask allows it, set the value from source to destination
        if (!std::isnan(mask(sourceIndex(0), sourceIndex(1)))) {
          destinationLayer(destinationIndex(0), destinationIndex(1)) = sourceLayer(sourceIndex(0), sourceIndex(1));
        }
      }
    } else {
      RCLCPP_ERROR(node->get_logger("ElevationMapping"), "Masked replace service: Layer %s does not exist!", sourceLayerIterator->c_str());
    }
  }

  return true;
}

bool ElevationMapping::saveMapServiceCallback(grid_map_msgs::srv::ProcessFile::Request& request,
                                              grid_map_msgs::srv::ProcessFile::Response& response) {
  RCLCPP_INFO(node->get_logger("ElevationMapping"), "Saving map to file.");
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  std::string topic = node_.getNamespace() + "/elevation_map";
  if (!request.topic_name.empty()) {
    topic = node_.getNamespace() + "/" + request.topic_name;
  }
  response.success = static_cast<unsigned char>(grid_map::GridMapRosConverter::saveToBag(map_.getFusedGridMap(), request.file_path, topic));
  response.success = static_cast<unsigned char>(
      (grid_map::GridMapRosConverter::saveToBag(map_.getRawGridMap(), request.file_path + "_raw", topic + "_raw")) &&
      static_cast<bool>(response.success));
  return static_cast<bool>(response.success);
}

bool ElevationMapping::loadMapServiceCallback(grid_map_msgs::srv::ProcessFile::Request& request,
                                              grid_map_msgs::srv::ProcessFile::Response& response) {
  RCLCPP_WARN(node->get_logger("ElevationMapping"), "Loading from bag file.");
  boost::recursive_mutex::scoped_lock scopedLockFused(map_.getFusedDataMutex());
  boost::recursive_mutex::scoped_lock scopedLockRaw(map_.getRawDataMutex());

  std::string topic = node_.getNamespace();
  if (!request.topic_name.empty()) {
    topic += "/" + request.topic_name;
  } else {
    topic += "/elevation_map";
  }

  response.success =
      static_cast<unsigned char>(grid_map::GridMapRosConverter::loadFromBag(request.file_path, topic, map_.getFusedGridMap()));
  response.success = static_cast<unsigned char>(
      grid_map::GridMapRosConverter::loadFromBag(request.file_path + "_raw", topic + "_raw", map_.getRawGridMap()) &&
      static_cast<bool>(response.success));

  // Update timestamp for visualization in ROS
  map_.setTimestamp(rclcpp::Time::now());
  map_.postprocessAndPublishRawElevationMap();
  return static_cast<bool>(response.success);
}

bool ElevationMapping::reloadParametersServiceCallback(std_srvs::srv::Trigger::Request& /*request*/, std_srvs::srv::Trigger::Response& response) {
  Parameters fallbackParameters{parameters_.getData()};

  const bool success{readParameters(true)};
  response.success = success;

  if (success) {
    response.message = "Successfully reloaded parameters!";
  } else {
    parameters_.setData(fallbackParameters);
    response.message = "Reloading parameters failed, reverted parameters to previous state!";
  }

  return true;
}

void ElevationMapping::resetMapUpdateTimer() {
  const Parameters parameters{parameters_.getData()};
  mapUpdateTimer_.stop();
  rclcpp::Duration periodSinceLastUpdate = rclcpp::Time::now() - map_.getTimeOfLastUpdate();
  if (periodSinceLastUpdate > parameters.maxNoUpdateDuration_) {
    periodSinceLastUpdate.fromSec(0.0);
  }
  mapUpdateTimer_.setPeriod(parameters.maxNoUpdateDuration_ - periodSinceLastUpdate);
  mapUpdateTimer_.start();
}

void ElevationMapping::stopMapUpdateTimer() {
  mapUpdateTimer_.stop();
}

}  // namespace elevation_mapping
