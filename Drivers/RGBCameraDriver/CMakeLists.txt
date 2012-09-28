cmake_minimum_required(VERSION 2.6.3)

project(RGBCameraOpenCv)

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

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(/usr/include/gstreamer-0.10 /usr/include/glib-2.0 /usr/lib/glib-2.0/include 
/usr/lib/i386-linux-gnu/glib-2.0/include 
/usr/lib/x86_64-linux-gnu/glib-2.0/include /usr/include/libxml2)

# Creates a component library 

set(RGC_CAMERA_OPEN_CV_CPP src/RGBCameraDriverComponent.cpp)

orocos_component(rgbCameraOpenCv ${RGC_CAMERA_OPEN_CV_CPP}) 

#set(RGB_CAMERA_LIBS gstreamer-0.10 gstbase-0.10 
#gobject-2.0 gmodule-2.0 gthread-2.0 rt xml2 glib-2.0 gstapp-0.10 pthread)

#set(RGB_CAMERA_LIBS gstreamer-0.10 gstbase-0.10 
#gobject-2.0 gmodule-2.0 gthread-2.0 rt xml2 glib-2.0 gstapp-0.10 pthread)

target_link_libraries(rgbCameraOpenCv ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} mbn_common)
target_link_libraries( rgbCameraOpenCv ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} ${RGB_CAMERA_LIBS} ${OpenCV_LIBS})


orocos_generate_package()
