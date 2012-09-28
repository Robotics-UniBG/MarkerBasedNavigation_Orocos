roslaunch youbot_description youbot_publisher_camera.launch&
sleep 4
rosrun ocl deployer-gnulinux -s ./solutions/simulator_application_test.xml

