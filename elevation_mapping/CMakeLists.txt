cmake_minimum_required(VERSION 3.5.1)
project(elevation_mapping)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(filters REQUIRED)
find_package(kindr REQUIRED)
find_package(kindr_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

# Clang tools support
find_package(cmake_clang_tools QUIET)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE RelWithDebInfo)


###########
# Library #
###########

# Our custom PCL type with precompiled template instantiations
add_library(${PROJECT_NAME}_pcl_types
  src/PointXYZRGBConfidenceRatio.cpp
)

add_library(${PROJECT_NAME}_library
  src/ElevationMapping.cpp
  src/ElevationMap.cpp
  # src/input_sources/Input.cpp
  # src/input_sources/InputSourceManager.cpp
  src/postprocessing/PostprocessorPool.cpp
  src/postprocessing/PostprocessingWorker.cpp
  src/postprocessing/PostprocessingPipelineFunctor.cpp
  src/RobotMotionMapUpdater.cpp
  src/sensor_processors/SensorProcessorBase.cpp
  src/sensor_processors/StructuredLightSensorProcessor.cpp
  src/sensor_processors/StereoSensorProcessor.cpp
  src/sensor_processors/LaserSensorProcessor.cpp
  src/sensor_processors/PerfectSensorProcessor.cpp
)

##############
# Executable #
##############

add_executable(${PROJECT_NAME}_node
  src/elevation_mapping_node.cpp
)

###############
# Include Dirs #
###############

# Include directories for the libraries
target_include_directories(${PROJECT_NAME}_pcl_types
  PUBLIC
    include
    ${Boost_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME}_library
  PUBLIC
    include
    include/elevation_mapping
    ${Boost_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${tf2_eigen_INCLUDE_DIRS}
    ${grid_map_core_INCLUDE_DIRS}
    ${grid_map_ros_INCLUDE_DIRS}
    ${kindr_INCLUDE_DIRS}
    ${kindr_ros_INCLUDE_DIRS}
)

# Include directories for the executable
target_include_directories(${PROJECT_NAME}_node
  PRIVATE
    include
    ${Boost_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${tf2_eigen_INCLUDE_DIRS}
)

###############
# Dependencies #
###############

# Specify dependencies for the PCL types library
ament_target_dependencies(${PROJECT_NAME}_pcl_types
  rclcpp
  geometry_msgs
  grid_map_ros
  std_srvs
  message_filters
  grid_map_msgs
  tf2_ros
  filters
  tf2
  tf2_geometry_msgs
  tf2_eigen
  sensor_msgs
  grid_map_core
  pcl_conversions
  pcl_ros
)

# Specify dependencies for the main library
ament_target_dependencies(${PROJECT_NAME}_library
  rclcpp
  geometry_msgs
  grid_map_ros
  std_srvs
  message_filters
  grid_map_msgs
  tf2_ros
  filters
  tf2
  tf2_geometry_msgs
  tf2_eigen
  sensor_msgs
  grid_map_core
  pcl_conversions
  kindr
  kindr_ros
  pcl_ros
)

#Link with libraries
target_link_libraries(${PROJECT_NAME}_library
  ${PROJECT_NAME}_pcl_types
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Link the libraries with the executable
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_library
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}_pcl_types
    ${PROJECT_NAME}_library
    ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME}_library)
ament_export_dependencies(
  Boost
  Eigen3
  geometry_msgs
  grid_map_ros
  grid_map_core
  grid_map_msgs
  message_filters
  rclcpp
  sensor_msgs
  std_srvs
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_eigen
  filters
  pcl_conversions
  kindr
  kindr_ros
  pcl_ros
)
ament_package()

##########
## Test ##
##########

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
  
#   ament_add_gtest(test_input_sources test/input_sources/test_input_sources.cpp)
#   target_link_libraries(test_input_sources ${PROJECT_NAME}_library)
  
#   ament_add_gtest(test_postprocessor test/postprocessing/PostprocessorTest.cpp)
#   target_link_libraries(test_postprocessor ${PROJECT_NAME}_library)

#   ament_add_gtest(test_thread_safe_parameter_wrapper test/ThreadSafeParameterWrapperTest.cpp)
#   target_link_libraries(test_thread_safe_parameter_wrapper ${PROJECT_NAME}_library)
  
#   if(cmake_clang_tools_FOUND)
#     add_default_clang_tooling()
#   endif()
# endif()
