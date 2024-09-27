cmake_minimum_required(VERSION 3.5.1)
project(elevation_mapping_demos)
find_package(ament_cmake REQUIRED)
find_package(elevation_mapping REQUIRED)
find_package(grid_map_rviz_plugin REQUIRED)
find_package(rviz REQUIRED)

## Find catkin macros and libraries

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

ament_export_dependencies(elevation_mapping grid_map_rviz_plugin rviz)
ament_package()

#############
## Install ##
#############

install(DIRECTORY config doc launch rviz sample_data
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/tf_to_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

##############
##   Test   ##
##############
if(BUILD_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
    roslaunch
    )

  ###################
  ## Code_coverage ##
  ###################
  find_package(cmake_code_coverage QUIET)

  target_link_libraries(test_${PROJECT_NAME}
    ${catkin_LIBRARIES}
    gtest_main
  )
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        test_${PROJECT_NAME}
    )
  endif()
  ament_add_gtest(test_${PROJECT_NAME}
    test/empty_test.cpp
  
)

  target_include_directories(test_${PROJECT_NAME}
    PRIVATE
      include
    SYSTEM PUBLIC
      ${catkin_INCLUDE_DIRS}
  )
  roslaunch_add_file_check(launch)
endif()