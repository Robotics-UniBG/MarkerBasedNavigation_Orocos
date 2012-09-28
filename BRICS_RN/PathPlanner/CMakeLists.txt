cmake_minimum_required(VERSION 2.6.3)

project(PathPlanner)

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

include_directories(/usr/include/glib-2.0 
                    /usr/lib/i386-linux-gnu/glib-2.0/include/
                    /usr/lib/glib-2.0/include/)

set(PATH_PLANNER_CPP src/PathPlanner.cpp
                     src/rtt/kdtree.c
                     src/rtt/optsystem.c
                     src/rtt/opttree.c)

orocos_component(PathPlanner ${PATH_PLANNER_CPP}) 

target_link_libraries( PathPlanner ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} glib-2.0)

orocos_generate_package()
