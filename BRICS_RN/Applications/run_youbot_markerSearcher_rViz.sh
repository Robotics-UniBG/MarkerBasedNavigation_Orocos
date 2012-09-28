roslaunch Applications youbot_base_joint_state_publisher.launch  &
sleep 4

rosrun ocl deployer-gnulinux -s solutions/youbot_markerSearcher_rViz.xml
