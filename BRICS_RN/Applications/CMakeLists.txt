cmake_minimum_required(VERSION 2.6.3)

project(Applications)

set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package( rtt )
  rosbuild_find_ros_package( ocl )
  set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )
  set( OCL_HINTS HINTS ${ocl_PACKAGE_PATH}/install )
endif()

find_package(Orocos-RTT REQUIRED rtt-marshalling ${RTT_HINTS})

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)



