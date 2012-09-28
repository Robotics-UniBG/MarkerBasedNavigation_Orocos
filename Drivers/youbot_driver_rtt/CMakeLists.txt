cmake_minimum_required(VERSION 2.4.6)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(youbot_driver_rtt)


#
# Do setup in case of ros package, If ROS_ROOT is set, it is
# recommended to use RTT/OCL through the ros packages.
#
set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package( rtt )
  set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/../install )
  # rosbuild_genmsg()
  # rosbuild_include(rtt_rosnode GenerateRTTtypekit)
  # ros_generate_rtt_typekit(youbot_driver_rtt)
endif()

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

orocos_component(${PROJECT_NAME} src/youbot_driver_component.cpp src/youbot_base_service.cpp src/youbot_arm_service.cpp src/youbot_helpers.cpp)

# target_link_libraries(${PROJECT_NAME} rtt-youbot_driver_rtt-typekit-${OROCOS_TARGET})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-Wall")

orocos_plugin(youbot-plugin src/youbot_plugin.cpp)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

orocos_generate_package()