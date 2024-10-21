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

ElevationMapping::ElevationMapping()
    : Node("elevation_mapping"),
      // inputSources_(shared_from_this()),
      receivedFirstMatchingPointcloudAndPose_(false),
      lastPointCloudUpdateTime_(0, 0, RCL_ROS_TIME) {
#ifndef NDEBUG
  // Print a warning if built in debug.
  RCLCPP_WARN(this->get_logger(), "CMake Build Type is 'Debug'. Change to 'Release' for better performance.");
#endif

  RCLCPP_INFO(this->get_logger(), "Elevation mapping node started.");

  // #TODO: Does this need to be on the heap?
}

void ElevationMapping::configure(){

  map_ = std::make_shared<ElevationMap>(shared_from_this());
  RCLCPP_INFO(this->get_logger(), "Elevation map created.");
  robotMotionMapUpdater_ = std::make_shared<RobotMotionMapUpdater>(shared_from_this());
  RCLCPP_INFO(this->get_logger(), "Robot motion map updater created.");

  transformBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  RCLCPP_INFO(this->get_logger(), "Transform buffer created.");
  transformListener_ = std::make_shared<tf2_ros::TransformListener>(*transformBuffer_);
  RCLCPP_INFO(this->get_logger(), "Transform listener created.");

  // create the CallbackGroups
  fusionServiceCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  visibilityCleanupCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  RCLCPP_INFO(this->get_logger(), "Callback groups created.");

  readParameters();
  RCLCPP_INFO(this->get_logger(), "Parameters read.");
  setupSubscribers();
  RCLCPP_INFO(this->get_logger(), "Subscribers setup.");
  setupServices();
  RCLCPP_INFO(this->get_logger(), "Services setup.");
  setupTimers();
  RCLCPP_INFO(this->get_logger(), "Timers setup.");
  initialize();
  RCLCPP_INFO(this->get_logger(), "Successfully configured node.");
}

void ElevationMapping::setupSubscribers() {  // Handle deprecated point_cloud_topic and input_sources configuration.
  auto [parameters, parameterGuard]{parameters_.getDataToWrite()};
  // TODO: Fix inputSources for ROS2
  // const bool configuredInputSources = inputSources_.configureFromRos("input_sources");
  // if (configuredInputSources) {
  //   inputSources_.registerCallbacks<sensor_msgs::msg::PointCloud2>(make_pair("pointcloud", &ElevationMapping::pointCloudCallback));
  // }

  pointCloudSubscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      parameters.pointCloudTopic_, rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pointCloudCallback(msg, true);
      });

  if (!parameters.robotPoseTopic_.empty()) {
    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
    qos.depth = 1;
    robotPoseSubscriber_.subscribe(this->shared_from_this(), parameters.robotPoseTopic_, qos);
    robotPoseCache_.connectInput(robotPoseSubscriber_);
    robotPoseCache_.setCacheSize(parameters.robotPoseCacheSize_);
  } else {
    parameters.ignoreRobotMotionUpdates_ = true;
  }
}

void ElevationMapping::setupServices() {
  const Parameters parameters{parameters_.getData()};
  // Multi-threading for fusion.
  fusionTriggerService_ = this->create_service<std_srvs::srv::Empty>(
      "trigger_fusion", 
      std::bind(&ElevationMapping::fuseEntireMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      fusionServiceCallbackGroup_);

  fusedSubmapService_ = this->create_service<grid_map_msgs::srv::GetGridMap>(
      "get_submap", 
      std::bind(&ElevationMapping::getFusedSubmapServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      fusionServiceCallbackGroup_);

  rawSubmapService_ = this->create_service<grid_map_msgs::srv::GetGridMap>(
      "get_raw_submap", 
      std::bind(&ElevationMapping::getRawSubmapServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      fusionServiceCallbackGroup_);

  clearMapService_ = this->create_service<std_srvs::srv::Empty>(
      "clear_map", std::bind(&ElevationMapping::clearMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  enableUpdatesService_ = this->create_service<std_srvs::srv::Empty>(
      "enable_updates", std::bind(&ElevationMapping::enableUpdatesServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  disableUpdatesService_ = this->create_service<std_srvs::srv::Empty>(
      "disable_updates", std::bind(&ElevationMapping::disableUpdatesServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  maskedReplaceService_ = this->create_service<grid_map_msgs::srv::SetGridMap>(
      "masked_replace", std::bind(&ElevationMapping::maskedReplaceServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  saveMapService_ = this->create_service<grid_map_msgs::srv::ProcessFile>(
      "save_map", std::bind(&ElevationMapping::saveMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  loadMapService_ = this->create_service<grid_map_msgs::srv::ProcessFile>(
      "load_map", std::bind(&ElevationMapping::loadMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  reloadParametersService_ = this->create_service<std_srvs::srv::Trigger>(
      "reload_parameters", std::bind(&ElevationMapping::reloadParametersServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void ElevationMapping::setupTimers() {
  
  const Parameters parameters{parameters_.getData()};
  ElevationMap::Parameters mapParameters{map_->parameters_.getData()};
  mapUpdateTimer_ = this->create_wall_timer(parameters.maxNoUpdateDuration_.to_chrono<std::chrono::milliseconds>(), 
                                             std::bind(&ElevationMapping::mapUpdateTimerCallback, this));

  // TODO set up for ROS2 with callback groups and executors.
  if (parameters.fusedMapPublishTimerDuration_ != std::chrono::milliseconds(0)) {
    fusedMapPublishTimer_ = this->create_wall_timer(parameters.fusedMapPublishTimerDuration_,
                                                    std::bind(&ElevationMapping::publishFusedMapCallback, this),
                                                    fusionServiceCallbackGroup_);
  }

  // Multi-threading for visibility cleanup. Visibility clean-up does not help when continuous clean-up is enabled.
  if (mapParameters.enableVisibilityCleanup_ && (parameters.visibilityCleanupTimerDuration_ != std::chrono::milliseconds(0)) &&
      !mapParameters.enableContinuousCleanup_) {
    visibilityCleanupTimer_ = this->create_wall_timer(parameters.visibilityCleanupTimerDuration_, 
                                                      std::bind(&ElevationMapping::visibilityCleanupCallback, this),
                                                      visibilityCleanupCallbackGroup_);
  }
}

// TODO: Seperate this function out to a declareParameters function and a readParameters function. Setup a set_parameters_callback.
bool ElevationMapping::readParameters(bool reload) {
  // Using getDataToWrite gets a write-lock on the parameters thereby blocking all reading threads for the whole scope of this
  // readParameters, which is desired.
  auto [parameters, parametersGuard] = parameters_.getDataToWrite();
  RCLCPP_INFO(this->get_logger(), "Reading parameters ...");
  auto [mapParameters, mapParametersGuard] = map_->parameters_.getDataToWrite();
  RCLCPP_INFO(this->get_logger(), "Reading map parameters ...");

  parameters.pointCloudTopic_ = this->declare_parameter<std::string>("point_cloud_topic", "/points");
  parameters.robotPoseTopic_ = this->declare_parameter<std::string>("robot_pose_with_covariance_topic", "/pose");
  parameters.trackPointFrameId_ = this->declare_parameter<std::string>("track_point_frame_id", "/robot");
  parameters.trackPoint_.x() = this->declare_parameter<double>("track_point_x", 0.0);
  parameters.trackPoint_.y() = this->declare_parameter<double>("track_point_y", 0.0);
  parameters.trackPoint_.z() = this->declare_parameter<double>("track_point_z", 0.0);

  parameters.robotPoseCacheSize_ = this->declare_parameter<int>("robot_pose_cache_size", 200);
  
  // TODO: Add ROS2 version of this
  // ROS_ASSERT(parameters.robotPoseCacheSize_ >= 0);
  rcpputils::assert_true(parameters.robotPoseCacheSize_ >= 0);

  double minUpdateRate{2.0};
  minUpdateRate = this->declare_parameter<double>("min_update_rate", minUpdateRate);
  if (minUpdateRate == 0.0) {
    parameters.maxNoUpdateDuration_ = rclcpp::Duration::from_seconds(0.0);
    RCLCPP_WARN(this->get_logger(), "Rate for publishing the map is zero.");
  } else {
    parameters.maxNoUpdateDuration_ = rclcpp::Duration::from_seconds(1.0 / minUpdateRate);
  }
  //TODO: what is the point of this
  rcpputils::assert_true(parameters.maxNoUpdateDuration_ > rclcpp::Duration::from_seconds(0.0));

  double timeTolerance{0.0};
  timeTolerance = this->declare_parameter<double>("time_tolerance", timeTolerance);
  parameters.timeTolerance_ = rclcpp::Duration::from_seconds(timeTolerance);

  double fusedMapPublishingRate{1.0};
  fusedMapPublishingRate = this->declare_parameter<double>("fused_map_publishing_rate", fusedMapPublishingRate);
  if (fusedMapPublishingRate == 0.0) {
    parameters.fusedMapPublishTimerDuration_ = std::chrono::milliseconds(0);
    RCLCPP_WARN(this->get_logger(), 
        "Rate for publishing the fused map is zero. The fused elevation map will not be published unless the service `triggerFusion` is "
        "called.");
  } else if (std::isinf(fusedMapPublishingRate)) {
    parameters.isContinuouslyFusing_ = true;
    parameters.fusedMapPublishTimerDuration_ = std::chrono::milliseconds(0);
  } else {
    parameters.fusedMapPublishTimerDuration_ = std::chrono::milliseconds(static_cast<int>(1000.0 / fusedMapPublishingRate));
  }

  double visibilityCleanupRate{1.0};
  visibilityCleanupRate = this->declare_parameter<double>("visibility_cleanup_rate", visibilityCleanupRate);
  if (visibilityCleanupRate == 0.0) {
    parameters.visibilityCleanupTimerDuration_ = std::chrono::milliseconds(0);
    RCLCPP_WARN(this->get_logger(), "Rate for visibility cleanup is zero and therefore disabled.");
    } else {
    parameters.visibilityCleanupTimerDuration_ = std::chrono::milliseconds(static_cast<int>(1000.0 / visibilityCleanupRate));
    mapParameters.visibilityCleanupDuration_ = 1.0 / visibilityCleanupRate;
  }

  // ElevationMap parameters. TODO Move this to the elevation map class.
  parameters.mapFrameId_ = this->declare_parameter<std::string>("map_frame_id", "/map");

  grid_map::Length length;
  grid_map::Position position;
  double resolution{0.01};
  length(0) = this->declare_parameter<double>("length_in_x", 1.5);
  length(1) = this->declare_parameter<double>("length_in_y", 1.5);
  position.x() = this->declare_parameter<double>("position_x", 0.0);
  position.y() = this->declare_parameter<double>("position_y", 0.0);
  resolution = this->declare_parameter<double>("resolution", resolution);

  // TODO: Use set_parameter_callback instead of this
  if (!reload) {
    map_->setFrameId(parameters.mapFrameId_);
    map_->setGeometry(length, resolution, position);
  }

  mapParameters.minVariance_ = this->declare_parameter<double>("min_variance", pow(0.003, 2));
  mapParameters.maxVariance_ = this->declare_parameter<double>("max_variance", pow(0.03, 2));
  mapParameters.mahalanobisDistanceThreshold_ = this->declare_parameter<double>("mahalanobis_distance_threshold", 2.5);
  mapParameters.multiHeightNoise_ = this->declare_parameter<double>("multi_height_noise", pow(0.003, 2));
  mapParameters.minHorizontalVariance_ = this->declare_parameter<double>("min_horizontal_variance", pow(resolution / 2.0, 2));  // two-sigma
  mapParameters.maxHorizontalVariance_ = this->declare_parameter<double>("max_horizontal_variance", 0.5);
  mapParameters.underlyingMapTopic_ = this->declare_parameter<std::string>("underlying_map_topic", std::string());
  mapParameters.enableVisibilityCleanup_ = this->declare_parameter<bool>("enable_visibility_cleanup", true);
  mapParameters.enableContinuousCleanup_ = this->declare_parameter<bool>("enable_continuous_cleanup", false);
  mapParameters.scanningDuration_ = this->declare_parameter<double>("scanning_duration", 1.0);
  mapParameters.increaseHeightAlpha_ = this->declare_parameter<double>("increase_height_alpha", 0.0);
  parameters.maskedReplaceServiceMaskLayerName_ = this->declare_parameter<std::string>("masked_replace_service_mask_layer_name", std::string("mask"));

  // Settings for initializing elevation map
  parameters.initializeElevationMap_ = this->declare_parameter<bool>("initialize_elevation_map", false);
  parameters.initializationMethod_ = this->declare_parameter<int>("initialization_method", 0);
  parameters.lengthInXInitSubmap_ = this->declare_parameter<double>("length_in_x_init_submap", 1.2);
  parameters.lengthInYInitSubmap_ = this->declare_parameter<double>("length_in_y_init_submap", 1.8);
  parameters.initSubmapHeightOffset_ = this->declare_parameter<double>("init_submap_height_offset", 0.0);
  parameters.initSubmapVariance_ = this->declare_parameter<double>("init_submap_variance", 0.01);
  parameters.targetFrameInitSubmap_ = this->declare_parameter<std::string>("target_frame_init_submap", std::string("/footprint"));

  // // SensorProcessor parameters. Deprecated, use the sensorProcessor from within input sources instead!
  std::string sensorType = this->declare_parameter<std::string>("sensor_processor.type", "structured_light");

  SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{this->declare_parameter<std::string>("robot_base_frame_id", "/robot"),
                                                                      parameters.mapFrameId_};
  if (sensorType == "structured_light") {
    sensorProcessor_ = std::make_unique<StructuredLightSensorProcessor>(shared_from_this(), generalSensorProcessorConfig);
  } else if (sensorType == "stereo") {
    sensorProcessor_ = std::make_unique<StereoSensorProcessor>(shared_from_this(), generalSensorProcessorConfig);
  } else if (sensorType == "laser") {
    sensorProcessor_ = std::make_unique<LaserSensorProcessor>(shared_from_this(), generalSensorProcessorConfig);
  } else if (sensorType == "perfect") {
    sensorProcessor_ = std::make_unique<PerfectSensorProcessor>(shared_from_this(), generalSensorProcessorConfig);
  } else {
    RCLCPP_ERROR(this->get_logger(), "The sensor type %s is not available.", sensorType.c_str());
  }

  // if (!sensorProcessor_->readParameters()) {
  //   return false;
  // }
  // if (!robotMotionMapUpdater_->readParameters()) {
  //   return false;
  // }

  return true;
}

bool ElevationMapping::initialize() {
  RCLCPP_INFO(this->get_logger(), "Elevation mapping node initializing ... ");
  rclcpp::sleep_for(std::chrono::seconds(1));  // Need this to get the TF caches fill up.
  resetMapUpdateTimer();
  initializeElevationMap();
  return true;
}

void ElevationMapping::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg, bool publishPointCloud) {
  const Parameters parameters{parameters_.getData()};
  RCLCPP_DEBUG(this->get_logger(), "Processing data from: %s", pointCloudMsg->header.frame_id.c_str());
  if (!parameters.updatesEnabled_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()), 10000, "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    if (publishPointCloud) {
      map_->setTimestamp(this->now());
      map_->postprocessAndPublishRawElevationMap();
    }
    return;
  }

  // Check if point cloud has corresponding robot pose at the beginning
  if (!receivedFirstMatchingPointcloudAndPose_) {
    const double oldestPoseTime = robotPoseCache_.getOldestTime().seconds();
    const double currentPointCloudTime = pointCloudMsg->header.stamp.sec;

    if (currentPointCloudTime < oldestPoseTime) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()), 5000, "No corresponding point cloud and pose are found. Waiting for first match. (Warning message is throttled, 5s.)");
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "First corresponding point cloud and pose found, elevation mapping started. ");
      receivedFirstMatchingPointcloudAndPose_ = true;
    }
  }

  cancelMapUpdateTimer();

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud.
  // TODO(max): Double check with http://wiki.ros.org/hydro/Migration
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pointCloudMsg, pcl_pc);

  PointCloudType::Ptr pointCloud(new PointCloudType);
  pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
  lastPointCloudUpdateTime_ = rclcpp::Time(1000 * pointCloud->header.stamp, RCL_ROS_TIME);

  RCLCPP_DEBUG(this->get_logger(), "ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));

  // Get robot pose covariance matrix at timestamp of point cloud.
  Eigen::Matrix<double, 6, 6> robotPoseCovariance;
  robotPoseCovariance.setZero();
  if (!parameters.ignoreRobotMotionUpdates_) {
    std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped const> poseMessage =
        robotPoseCache_.getElemBeforeTime(lastPointCloudUpdateTime_);
    if (!poseMessage) {
      // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
      if (robotPoseCache_.getOldestTime().seconds() > lastPointCloudUpdateTime_.seconds()) {
        RCLCPP_ERROR(this->get_logger(), "The oldest pose available is at %f, requested pose at %f", robotPoseCache_.getOldestTime().seconds(),
                  lastPointCloudUpdateTime_.seconds());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Could not get pose information from robot for time %f. Buffer empty?", lastPointCloudUpdateTime_.seconds());
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
      RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), 10000, "Waiting for tf transformation to be available. (Message is throttled, 10s.)");
      return;
    }
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *(this->get_clock()), 10000, "Point cloud could not be processed. (Throttled 10s)");
    resetMapUpdateTimer();
    return;
  }

  boost::recursive_mutex::scoped_lock scopedLock(map_->getRawDataMutex());

  // Update map location.
  updateMapLocation();

  // Update map from motion prediction.
  if (!updatePrediction(lastPointCloudUpdateTime_)) {
    RCLCPP_ERROR(this->get_logger(), "Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  ElevationMap::Parameters mapParameters{map_->parameters_.getData()};

  // Clear the map if continuous clean-up was enabled.
  if (mapParameters.enableContinuousCleanup_) {
    RCLCPP_DEBUG(this->get_logger(), "Clearing elevation map before adding new point cloud.");
    map_->clear();
  }

  // Add point cloud to elevation map.
  if (!map_->add(pointCloudProcessed, measurementVariances, lastPointCloudUpdateTime_,
                Eigen::Affine3d(sensorProcessor_->transformationSensorToMap_))) {
    RCLCPP_ERROR(this->get_logger(), "Adding point cloud to elevation map failed.");
    resetMapUpdateTimer();
    return;
  }

  if (publishPointCloud) {
    // Publish elevation map.
    map_->postprocessAndPublishRawElevationMap();
    if (isFusingEnabled()) {
      map_->fuseAll();
      map_->publishFusedElevationMap();
    }
  }

  resetMapUpdateTimer();
}

void ElevationMapping::mapUpdateTimerCallback() {
  const Parameters parameters{parameters_.getData()};
  if (!parameters.updatesEnabled_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()), 10000, "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    map_->setTimestamp(this->now());
    map_->postprocessAndPublishRawElevationMap();
    return;
  }

  rclcpp::Time time = this->now();
  RCLCPP_DEBUG(this->get_logger(), "Now time clock type: %d", time.get_clock_type());
  RCLCPP_DEBUG(this->get_logger(), "lastPointCloudUpdateTime_ time clock type: %d", lastPointCloudUpdateTime_.get_clock_type());
  if ((lastPointCloudUpdateTime_ - time) <=
      parameters.maxNoUpdateDuration_) {  // there were updates from sensordata, no need to force an update.
    return;
  }
  RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()), 5000, "Elevation map is updated without data from the sensor. (Warning message is throttled, 5s.)");

  //TODO: convert to std::recursive_mutex and std::lock_guard
  boost::recursive_mutex::scoped_lock scopedLock(map_->getRawDataMutex());

  cancelMapUpdateTimer();

  // Update map from motion prediction.
  if (!updatePrediction(time)) {
    RCLCPP_ERROR(this->get_logger(), "Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Publish elevation map.
  map_->postprocessAndPublishRawElevationMap();
  if (isFusingEnabled()) {
    map_->fuseAll();
    map_->publishFusedElevationMap();
  }
  // TODO: as previous TODO.
  resetMapUpdateTimer();
}

void ElevationMapping::publishFusedMapCallback() {
  if (!map_->hasFusedMapSubscribers()) {
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "Elevation map is fused and published from timer.");
  boost::recursive_mutex::scoped_lock scopedLock(map_->getFusedDataMutex());
  map_->fuseAll();
  map_->publishFusedElevationMap();
}

void ElevationMapping::visibilityCleanupCallback() {
  RCLCPP_DEBUG(this->get_logger(), "Elevation map is running visibility cleanup.");
  // Copy constructors for thread-safety.
  map_->visibilityCleanup(lastPointCloudUpdateTime_);
}

void ElevationMapping::fuseEntireMapServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  //TODO: convert to std::recursive_mutex and std::lock_guard
  boost::recursive_mutex::scoped_lock scopedLock(map_->getFusedDataMutex());
  map_->fuseAll();
  map_->publishFusedElevationMap();
}

bool ElevationMapping::isFusingEnabled() {
  const Parameters parameters{parameters_.getData()};
  return parameters.isContinuouslyFusing_ && map_->hasFusedMapSubscribers();
}

bool ElevationMapping::updatePrediction(const rclcpp::Time& time) {
  const Parameters parameters{parameters_.getData()};
  if (parameters.ignoreRobotMotionUpdates_) {
    return true;
  }

  RCLCPP_DEBUG(this->get_logger(), "Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().seconds());

  if (time + parameters.timeTolerance_ < map_->getTimeOfLastUpdate()) {
    RCLCPP_ERROR(this->get_logger(), "Requested update with time stamp %f, but time of last update was %f.", time.seconds(), map_->getTimeOfLastUpdate().seconds());
    return false;
  } else if (time < map_->getTimeOfLastUpdate()) {
    RCLCPP_DEBUG(this->get_logger(), "Requested update with time stamp %f, but time of last update was %f. Ignoring update.", time.seconds(),
              map_->getTimeOfLastUpdate().seconds());
    return true;
  }

  // Get robot pose at requested time.
  std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(time);
  if (!poseMessage) {
    // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
    if (robotPoseCache_.getOldestTime().seconds() > lastPointCloudUpdateTime_.seconds()) {
      RCLCPP_ERROR(this->get_logger(), "The oldest pose available is at %f, requested pose at %f", robotPoseCache_.getOldestTime().seconds(),
                lastPointCloudUpdateTime_.seconds());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Could not get pose information from robot for time %f. Buffer empty?", lastPointCloudUpdateTime_.seconds());
    }
    return false;
  }

  kindr::HomTransformQuatD robotPose;
  kindr_ros::convertFromRosGeometryMsg(poseMessage->pose.pose, robotPose);
  // Covariance is stored in row-major in ROS: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
  Eigen::Matrix<double, 6, 6> robotPoseCovariance =
      Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(poseMessage->pose.covariance.data(), 6, 6);

  // Compute map variance update from motion prediction.
  robotMotionMapUpdater_->update(map_, robotPose, robotPoseCovariance, time);

  return true;
}

bool ElevationMapping::updateMapLocation() {
  const Parameters parameters{parameters_.getData()};
  RCLCPP_DEBUG(this->get_logger(), "Elevation map is checked for relocalization.");

  geometry_msgs::msg::PointStamped trackPoint;
  trackPoint.header.frame_id = parameters.trackPointFrameId_;
  trackPoint.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  kindr_ros::convertToRosGeometryMsg(parameters.trackPoint_, trackPoint.point);
  geometry_msgs::msg::PointStamped trackPointTransformed;

  try {
    trackPointTransformed = transformBuffer_->transform(trackPoint, map_->getFrameId());
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return false;
  }

  kindr::Position3D position3d;
  kindr_ros::convertFromRosGeometryMsg(trackPointTransformed.point, position3d);
  grid_map::Position position = position3d.vector().head(2);
  map_->move(position);
  return true;
}

void ElevationMapping::getFusedSubmapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                                     std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response) {
  grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
  grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
  RCLCPP_DEBUG(this->get_logger(), "Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(), requestedSubmapPosition.y(),
            requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_->getFusedDataMutex());
  map_->fuseArea(requestedSubmapPosition, requestedSubmapLength);

  bool isSuccess;
  grid_map::GridMap subMap = map_->getFusedGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
  scopedLock.unlock();

  if (request->layers.empty()) {
    grid_map_msgs::msg::GridMap::SharedPtr mapMessage = grid_map::GridMapRosConverter::toMessage(subMap);
    response->map = *mapMessage;
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request->layers) {
      layers.push_back(layer);
    }
    grid_map_msgs::msg::GridMap::SharedPtr mapMessage = grid_map::GridMapRosConverter::toMessage(subMap, layers);
    response->map = *mapMessage;
  }

  RCLCPP_DEBUG(this->get_logger(), "Elevation submap responded with timestamp %f.", map_->getTimeOfLastFusion().seconds());
}

void ElevationMapping::getRawSubmapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                                   std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response) {
  grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
  grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
  RCLCPP_DEBUG(this->get_logger(), "Elevation raw submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(),
            requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_->getRawDataMutex());

  bool isSuccess;
  grid_map::GridMap subMap = map_->getRawGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
  scopedLock.unlock();

  if (request->layers.empty()) {
    grid_map_msgs::msg::GridMap::SharedPtr mapMessage = grid_map::GridMapRosConverter::toMessage(subMap);
    response->map = *mapMessage;
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request->layers) {
      layers.push_back(layer);
    }
    grid_map_msgs::msg::GridMap::SharedPtr mapMessage = grid_map::GridMapRosConverter::toMessage(subMap, layers);
    response->map = *mapMessage;
  }
}

void ElevationMapping::disableUpdatesServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                                     std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Disabling updates.");
  auto [parameters, parameterGuard]{parameters_.getDataToWrite()};
  parameters.updatesEnabled_ = false;
}

void ElevationMapping::enableUpdatesServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, 
                                                    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Enabling updates.");
  auto [parameters, parameterGuard]{parameters_.getDataToWrite()};
  parameters.updatesEnabled_ = true;
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
        transform_msg = transformBuffer_->lookupTransform(parameters.mapFrameId_, parameters.targetFrameInitSubmap_, rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(5.0));
        tf2::fromMsg(transform_msg, transform);

        RCLCPP_DEBUG(this->get_logger(), "Initializing with x: %f y: %f z: %f", 
               transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

        const grid_map::Position positionRobot(transform.getOrigin().x(), transform.getOrigin().y());

        // Move map before we apply the height values. This prevents unwanted behavior from intermediate move() calls in
        // updateMapLocation().
        map_->move(positionRobot);

        map_->setRawSubmapHeight(positionRobot, transform.getOrigin().z() + parameters.initSubmapHeightOffset_,
                                parameters.initSubmapVariance_, parameters.lengthInXInitSubmap_, parameters.lengthInYInitSubmap_);
        return true;
      } catch (tf2::TransformException& ex) {
        RCLCPP_DEBUG(this->get_logger(), "%s", ex.what());
        RCLCPP_WARN(this->get_logger(), "Could not initialize elevation map with constant height. (This warning can be ignored if TF tree is not available.)");
        return false;
      }
    }
  }
  return true;
}

void ElevationMapping::clearMapServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, 
                                               std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Clearing map...");
  map_->clear();
  initializeElevationMap();
  RCLCPP_INFO(this->get_logger(), "Map cleared.");
}

void ElevationMapping::maskedReplaceServiceCallback(const std::shared_ptr<grid_map_msgs::srv::SetGridMap::Request> request,
                                                   std::shared_ptr<grid_map_msgs::srv::SetGridMap::Response> /*response*/) {
  const Parameters parameters{parameters_.getData()};
  RCLCPP_INFO(this->get_logger(), "Masked replacing of map.");
  grid_map::GridMap sourceMap;
  grid_map::GridMapRosConverter::fromMessage(request->map, sourceMap);

  // Use the supplied mask or do not use a mask
  grid_map::Matrix mask;
  if (sourceMap.exists(parameters.maskedReplaceServiceMaskLayerName_)) {
    mask = sourceMap[parameters.maskedReplaceServiceMaskLayerName_];
  } else {
    mask = Eigen::MatrixXf::Ones(sourceMap.getSize()(0), sourceMap.getSize()(1));
  }

  boost::recursive_mutex::scoped_lock scopedLockRawData(map_->getRawDataMutex());

  // Loop over all layers that should be set
  for (auto sourceLayerIterator = sourceMap.getLayers().begin(); sourceLayerIterator != sourceMap.getLayers().end();
       sourceLayerIterator++) {
    // skip "mask" layer
    if (*sourceLayerIterator == parameters.maskedReplaceServiceMaskLayerName_) {
      continue;
    }
    grid_map::Matrix& sourceLayer = sourceMap[*sourceLayerIterator];
    // Check if the layer exists in the elevation map
    if (map_->getRawGridMap().exists(*sourceLayerIterator)) {
      grid_map::Matrix& destinationLayer = map_->getRawGridMap()[*sourceLayerIterator];
      for (grid_map::GridMapIterator destinationIterator(map_->getRawGridMap()); !destinationIterator.isPastEnd(); ++destinationIterator) {
        // Use the position to find corresponding indices in source and destination
        const grid_map::Index destinationIndex(*destinationIterator);
        grid_map::Position position;
        map_->getRawGridMap().getPosition(*destinationIterator, position);

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
      RCLCPP_ERROR(this->get_logger(), "Masked replace service: Layer %s does not exist!", sourceLayerIterator->c_str());
    }
  }
}

void ElevationMapping::saveMapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
                                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Saving map to file.");
  boost::recursive_mutex::scoped_lock scopedLock(map_->getFusedDataMutex());
  map_->fuseAll();
  std::string topic = std::string(this->get_namespace()) + "/elevation_map";
  if (!request->topic_name.empty()) {
    topic = std::string(this->get_namespace()) + "/" + request->topic_name;
  }
  response->success = static_cast<unsigned char>(grid_map::GridMapRosConverter::saveToBag(map_->getFusedGridMap(), request->file_path, topic));
  response->success = static_cast<unsigned char>(
      (grid_map::GridMapRosConverter::saveToBag(map_->getRawGridMap(), request->file_path + "_raw", topic + "_raw")) &&
      static_cast<bool>(response->success));
}

void ElevationMapping::loadMapServiceCallback(std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
                                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  RCLCPP_WARN(this->get_logger(), "Loading from bag file.");
  boost::recursive_mutex::scoped_lock scopedLockFused(map_->getFusedDataMutex());
  boost::recursive_mutex::scoped_lock scopedLockRaw(map_->getRawDataMutex());

  std::string topic = this->get_namespace();
  if (!request->topic_name.empty()) {
    topic += "/" + request->topic_name;
  } else {
    topic += "/elevation_map";
  }

  response->success =
      static_cast<unsigned char>(grid_map::GridMapRosConverter::loadFromBag(request->file_path, topic, map_->getFusedGridMap()));
  response->success = static_cast<unsigned char>(
      grid_map::GridMapRosConverter::loadFromBag(request->file_path + "_raw", topic + "_raw", map_->getRawGridMap()) &&
      static_cast<bool>(response->success));

  // Update timestamp for visualization in ROS
  map_->setTimestamp(this->now());
  map_->postprocessAndPublishRawElevationMap();
}

// TODO: We can probably replace this with a set_parameters_callback instread.
void ElevationMapping::reloadParametersServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, 
                                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  Parameters fallbackParameters{parameters_.getData()};

  const bool success{readParameters(true)};
  response->success = success;

  if (success) {
    response->message = "Successfully reloaded parameters!";
  } else {
    parameters_.setData(fallbackParameters);
    response->message = "Reloading parameters failed, reverted parameters to previous state!";
  }
}

void ElevationMapping::resetMapUpdateTimer() {
  const Parameters parameters{parameters_.getData()};
  mapUpdateTimer_->cancel();
  rclcpp::Time currentTime = this->now();
  rclcpp::Time lastUpdateTime = map_->getTimeOfLastUpdate();
  rclcpp::Duration periodSinceLastUpdate = currentTime - lastUpdateTime;
  if (periodSinceLastUpdate > parameters.maxNoUpdateDuration_) {
    periodSinceLastUpdate = rclcpp::Duration::from_seconds(0.0);
  }

  auto remainingTime = (parameters.maxNoUpdateDuration_ - periodSinceLastUpdate).to_chrono<std::chrono::milliseconds>();
  // Look at contributing to ROS2 to allow for a reset of the timer time.
  mapUpdateTimer_ = this->create_wall_timer(remainingTime, std::bind(&ElevationMapping::mapUpdateTimerCallback, this));
}

void ElevationMapping::cancelMapUpdateTimer() {
  mapUpdateTimer_->cancel();
}

}  // namespace elevation_mapping
