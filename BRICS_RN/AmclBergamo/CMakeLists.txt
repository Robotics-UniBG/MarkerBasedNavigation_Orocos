cmake_minimum_required(VERSION 2.6.3)

project(AmclBergamo)

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


set(AMCL_BERGAMO_CPP src/AmclBergamo.cpp 
                     src/amcl.cpp
                     src/map/map.c
                     src/map/map_store.c
                     src/map/map_range.c
                     src/map/map_draw.c
                     src/map/map_cspace.cpp
                     src/pf/eig3.c
                     src/pf/pf_draw.c
                     src/pf/pf_kdtree.c
                     src/pf/pf_pdf.c
                     src/pf/pf_vector.c
                     src/pf/pf.c
                     src/sensors/amcl_sensor.cpp
                     src/sensors/amcl_laser.cpp
                     src/sensors/amcl_odom.cpp)

orocos_component(AmclBergamo ${AMCL_BERGAMO_CPP}) 

target_link_libraries(AmclBergamo ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} )

orocos_generate_package()
