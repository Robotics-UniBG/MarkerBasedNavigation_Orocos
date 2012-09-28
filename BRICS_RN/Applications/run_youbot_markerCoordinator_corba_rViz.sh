#Export the variables fot the nameservice
IP=$(ifconfig | awk -F':' '/inet addr/&&!/127.0.0.1/{split($2,_," ");print _[1]}')
export ORBInitRef=NameService=corbaname::$IP
export ORBDefaultInitRef=corbaloc::iiop:localhost

roslaunch Applications youbot_base_joint_state_publisher.launch  &
sleep 4

rosrun ocl deployer-corba-gnulinux -s solutions/youbot_markerCoordinator_corba_rViz.xml
