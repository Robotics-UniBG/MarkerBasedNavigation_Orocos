cmake_minimum_required(VERSION 2.6.3)

project(HokuyoDriver)

#set(CMAKE_INCLUDE_PATH /usr/local/Aria/include)



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


set(HOKUYO_DRIVER_CPP src/HokuyoDriverComponent.cpp src/HokuyoDriver.cpp src/hokuyo.cpp)

#set(PIONEER_DRIVER_LIBS Aria pthread dl rt)

orocos_component(hokuyoDriver ${HOKUYO_DRIVER_CPP}) # ...you may add multiple source files

target_link_libraries(hokuyoDriver ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY})


orocos_generate_package()
