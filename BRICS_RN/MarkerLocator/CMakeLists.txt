cmake_minimum_required(VERSION 2.6.3)

project(Marker_Local_Traker_ARTKP)

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
find_package(OpenCV REQUIRED)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

set(MARKER_LOCATOR_CPP src/MarkerLocatorComponent.cpp)
 
orocos_component(markerLocator ${MARKER_LOCATOR_CPP})

#set(AR_TOOLKIT_LIBS ARgsub AR ARMulti ARvideo) 
set(AR_TOOLKIT_PLUS_LIBS ARToolKitPlus) 

target_link_libraries(markerLocator ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} mbn_common)
target_link_libraries(markerLocator ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} ${OpenCV_LIBS} ${AR_TOOLKIT_PLUS_LIBS})


orocos_generate_package()
