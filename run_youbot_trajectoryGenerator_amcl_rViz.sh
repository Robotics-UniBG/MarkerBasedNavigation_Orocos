roslaunch Applications youbot_base_joint_state_publisher.launch  &
sleep 4

rosrun ocl deployer-gnulinux -s solutions/youbot_trajectoryGenerator_amcl_rViz.xml 
