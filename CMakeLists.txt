cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME lidar_pose_localization)
project(${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
  message(FATAL_ERROR "Using an unsupported compiler! Compilation has only been tested with GCC 4.9 and above")
else()
  add_compile_options(-std=c++14)
endif()
  
set(LIB_NAME localization)
set(CMAKE_BUILD_TYPE Release)

set(SO_VERSION_MAJOR 2)
set(SO_VERSION_MINOR 2)
set(SO_VERSION_PATCH 0)
set(SO_VERSION ${SO_VERSION_MAJOR}.${SO_VERSION_MINOR}.${SO_VERSION_PATCH})

configure_file(
  "${PROJECT_SOURCE_DIR}/version.h.in"
  "${PROJECT_SOURCE_DIR}/include/common/version.h"
)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  roscpp
  nav_msgs
  geometry_msgs
  roslib
  pcl_ros
  pcl_conversions
  tf
  tf_conversions
  eigen_conversions
  )

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(PCL 1.8 REQUIRED common visualization)
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_definitions(-fPIC)

find_package(yaml-cpp REQUIRED)

include_directories( 
  include
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )

# message(STATUS "Using PCL version ${PCL_VERSION}")
# find_package(libpointmatcher REQUIRED EXACT 1.3.1)
# include_directories(${libpointmatcher_INCLUDE_DIRS})
include_directories(lib/include)
link_directories(${PROJECT_SOURCE_DIR}/lib)

## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
  )

add_definitions(-DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG)
############
# Libraries#
############
add_library(multi_lidar_aes_encriptor
  src/yaml/aes.cpp
  src/yaml/aes_encryptor.cpp)

add_library(multi_lidar_yaml_parser
  src/yaml/yaml_parser.cpp)
target_link_libraries(multi_lidar_yaml_parser 
multi_lidar_aes_encriptor
  yaml-cpp
  ) 

add_library(multi_lidar_pointcloud_viewer
src/common/point_cloud_viewer.cpp
)
target_link_libraries(multi_lidar_pointcloud_viewer
${PCL_LIBRARIES}
)

add_library(multi_lidar_pointmatcher_ros
  src/pointmatcher/point_cloud.cpp
  src/pointmatcher/transform.cpp)

target_link_libraries(multi_lidar_pointmatcher_ros
  ${catkin_LIBRARIES}
  pointmatcher 
)

add_library(multi_lidar_map_server
  src/map_server/celled_map.cpp
  src/map_server/map_server.cpp
  src/sensor/gps_msg_process.cpp
  )

target_link_libraries(multi_lidar_map_server
multi_lidar_yaml_parser
multi_lidar_pointcloud_viewer
  )

add_library(multi_lidar_icp_localization
  src/icp_localization/icp_point_filter.cpp
  src/icp_localization/icp_sampler.cpp
  src/icp_localization/icp_pre_matcher.cpp
  src/icp_localization/icp_matcher.cpp
  src/icp_localization/icp_evaluator.cpp
  src/icp_localization/icp_localization.cpp
  )

target_link_libraries(multi_lidar_icp_localization
multi_lidar_yaml_parser
multi_lidar_map_server
  )

add_library(multi_lidar_sensor_fusion
  src/msf/observer/lidar_es_observer.cpp
  src/msf/core/sensor_fusion.cpp
  src/common/logger.cpp
  )

target_link_libraries(multi_lidar_sensor_fusion
  ${catkin_LIBRARIES}
  multi_lidar_yaml_parser
  multi_lidar_icp_localization
  )

add_library(multi_lidar_rs_localization
src/rs_localization/rs_localization_ros.cpp
src/rs_localization/rs_localization.cpp
src/rs_localization/rs_localization_impl.cpp
)
target_link_libraries(multi_lidar_rs_localization 
multi_lidar_yaml_parser
multi_lidar_sensor_fusion
) 

set_target_properties(multi_lidar_rs_localization PROPERTIES VERSION ${SO_VERSION}
                                                 SOVERSION ${SO_VERSION_MAJOR})


add_library(multi_lidar_rviz_display
src/visualization/rviz_display.cpp
src/sensor/gps_msg_process.cpp)
target_link_libraries(multi_lidar_rviz_display
${catkin_LIBRARIES}
)

#############
#Executables#
#############
add_executable(lidar_pose_localization
  src/rs_localization/rs_localization_ros_node.cpp
  src/visualization/rviz_display.cpp
  )
target_link_libraries(lidar_pose_localization
  ${catkin_LIBRARIES}
  multi_lidar_yaml_parser
  multi_lidar_rs_localization
  multi_lidar_pointmatcher_ros
  multi_lidar_map_server
  multi_lidar_rviz_display
  )

#####################
## Install library ##
#####################
# set(CMAKE_INSTALL_PREFIX /home/data/code_data/git_lab/lidars_calibration_pcd/src/rs_calibration/lidar_pose_localization)
# set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)
# set(INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include)

# add_custom_target(install-localization-release COMMAND 
#                   ${CMAKE_COMMAND} -DCOMPONENT=release -P ${CMAKE_BINARY_DIR}/cmake_install.cmake)

# set(RELEASE_LIBS 
#     multi_lidar_aes_encriptor
#     multi_lidar_yaml_parser
#     multi_lidar_sensor_fusion
#     multi_lidar_icp_localization
#     multi_lidar_rs_localization
#     multi_lidar_pointmatcher_ros
#     multi_lidar_pointcloud_viewer
#     multi_lidar_map_server
#     multi_lidar_rviz_display
# )

# INSTALL(TARGETS ${RELEASE_LIBS}
#         DESTINATION ${INSTALL_LIB_DIR}
#         COMPONENT release)

#############
## Install ##
#############
# install(DIRECTORY cfg/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)

# install(DIRECTORY user_config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/user_config)

# install(DIRECTORY model/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/model)

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   PATTERN "test" EXCLUDE
#   )

# install(DIRECTORY data/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
#   PATTERN "*.rsmap" EXCLUDE
#   )
# install(FILES CHANGELOG.rst
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   )

#   install(DIRECTORY doc/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
#   PATTERN "img" EXCLUDE
#   )

# install(DIRECTORY rviz/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
#   )

# install(TARGETS rs_localization_ros # map_process black_box
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


