/********************************************************************************
 *
 * TrajectoryFollower
 *
 * Copyright (c) 2012
 * All rights reserved.
 *
 * Luca Gherardi
 * University of Bergamo
 * Dept. of Information Technology and Mathematics
 *
 * -------------------------------------------------------------------------------
 *
 * File: TrajectoryFollower.cpp
 * Created: Feb 09, 2012
 *
 * Author: <A HREF="mailto:luca.gherardi@unibg.it">Luca Gherardi</A>
 *
 * Supervised by: <A HREF="mailto:brugali@unibg.it">Davide Brugali</A>
 *
 * -------------------------------------------------------------------------------
 *
 * This sofware is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * -------------------------------------------------------------------------------
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of the University of Bergamo nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 *******************************************************************************/

#include "TrajectoryFollower/TrajectoryFollower.hpp"


namespace Navigation{


TrajectoryFollower::TrajectoryFollower(string const& name)
: TaskContext(name),
  odometryInPort("odometryInPort"),
  targetTrajectoryInPort("targetTrajectoryInPort"),
  twistOutPort("twistOutPort"),
  motionStatusOutPort("motionStatusOutPort")
{

	twistOutPort.keepLastWrittenValue(true);
	motionStatusOutPort.keepLastWrittenValue(true);

	this->addPort(odometryInPort).doc("Robot odometry input port");
	this->addPort(targetTrajectoryInPort).doc("Robot target trajectory input port");


	this->addPort(twistOutPort).doc("Robot twist output port");
	this->addPort(motionStatusOutPort).doc("Motion status output port");


	this->addAttribute("lastOdometry",lastOdometry);
	this->addAttribute("nextTargetVel",nextTargetVel);
	this->addAttribute("nextTargetPose",nextTargetPose);
	this->addAttribute("trajectoryTarget",trajectoryTarget);
	this->addAttribute("trajectoryComputed",isFollowing);

	this->addProperty("minDeltaVel",minDeltaVel)
						.doc("Minimun velocity variation that will produce a new trajectory generation");

	// Treesholds
	this->addProperty("goalDistanceThreeshold",goalDistanceThreeshold)
						.doc("The min distance between the current pose and the target pose for considering the transaltion as done");
	this->addProperty("goalOrientationThreeshold",goalOrientationThreeshold)
						.doc("The min diffrence between the current theta and the target theta for considering the rotation as done");

	// Velocity and acceleration limits
	this->addProperty("minLinearVelocity",minLinearVelocity).doc("The min linear velocity");
	this->addProperty("maxLinearVelocity",maxLinearVelocity).doc("The max linear velocity");
	this->addProperty("minAngularVelocity",minAngularVelocity).doc("The min angular velocity");
	this->addProperty("maxAngularVelocity",maxAngularVelocity).doc("The max angular velocity");
	this->addProperty("minLinearAcceleration",minLinearAcceleration).doc("The min linear acceleration");
	this->addProperty("maxLinearAcceleration",maxLinearAcceleration).doc("The max linear acceleration");
	this->addProperty("minAngularAcceleration",minAngularAcceleration).doc("The min angular acceleration");
	this->addProperty("maxAngularAcceleration",maxAngularAcceleration).doc("The max angular acceleration");

	trajectorySet = false;
	odometryUpdated = false;
	isFollowing = false;

	nextTargetPose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	lastOdometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

	marshalling = this->getProvider<Marshalling>("marshalling");


}

bool TrajectoryFollower::startHook(){

	// TODO: 0.5 should be replaced with a parameter.
	// Alexey said he will compute these values automatically in its new implementation
	segmentController = new OmnidriveController(minLinearVelocity, maxLinearVelocity,
			minAngularVelocity, maxAngularVelocity, 0.5, 0.5);

	return true;

}

bool TrajectoryFollower::configureHook(){

	return true;

}

void TrajectoryFollower::stopHook(){


}

void TrajectoryFollower::cleanupHook(){

	//marshalling->writeProperties("properties/TrajectoryFollower.cpf");

}

void TrajectoryFollower::updateHook() {

	// read the input port and set the recomputeTrajectoryFlag
	readInputPorts();

	// Check whether the odometry error is grater than a threshold in order
	// to regenerate the trajectory if needed
	if(odometryUpdated){
		//		if(checkOdometry(nextTargetPose.pose)){
		//			trajectoryComputed = false;
		//			imposedTwist.linear.x = 0;
		//			imposedTwist.linear.y = 0;
		//			imposedTwist.linear.z = 0;
		//			imposedTwist.angular.x = 0;
		//			imposedTwist.angular.y = 0;
		//			imposedTwist.angular.z = 0;
		//			twistOutPort.write(imposedTwist);
		//		}
	}

	imposedTwist.linear.x = 0;
	imposedTwist.linear.y = 0;
	imposedTwist.linear.z = 0;
	imposedTwist.angular.x = 0;
	imposedTwist.angular.y = 0;
	imposedTwist.angular.z = 0;

	if(isFollowing){

		segmentControlHook();

	}

	twistOutPort.write(imposedTwist);

}

double TrajectoryFollower::computeEuclideanDistance(geometry_msgs::Point target, geometry_msgs::Point odometry){

	return sqrt(pow(target.x - odometry.x, 2) + pow(target.y - odometry.y, 2));

}

void TrajectoryFollower::readInputPorts(){

	geometry_msgs::PoseStamped inputPose;
	brics_rn_msgs::Velocity inputVel;
	nav_msgs::Odometry inputOdometry;
	brics_rn_msgs::Trajectory inputTraj;

	odometryUpdated = false;

	// stores the last odometry if a new data is available
	// updates currentPose and currentVel
	if(odometryInPort.read(inputOdometry) == NewData){
		lastOdometry = inputOdometry;

		lastOdometryPose.x = inputOdometry.pose.pose.position.x;
		lastOdometryPose.y = inputOdometry.pose.pose.position.y;
		lastOdometryPose.theta = angles::normalize_angle(tf::getYaw(
				inputOdometry.pose.pose.orientation));

		lastOdometryVel.v = inputOdometry.twist.twist.linear.x;
		lastOdometryVel.v_dot = 0;
		lastOdometryVel.w = inputOdometry.twist.twist.angular.z;
		lastOdometryVel.w_dot = 0;
		odometryUpdated = true;
	}

	if(targetTrajectoryInPort.read(inputTraj) ==  NewData){

		trajectoryTarget = inputTraj;
		currentTrajectoryIndex = 0;

		if(trajectoryTarget.waypoints.size()>0){

			nextTargetPose = trajectoryTarget.waypoints.at(currentTrajectoryIndex).pose;

			nextTargetVel.v = trajectoryTarget.waypoints.at(currentTrajectoryIndex).linear_vel;
			nextTargetVel.w = trajectoryTarget.waypoints.at(currentTrajectoryIndex).angular_vel;

			segmentController->setInitialPose(lastOdometry.pose.pose);
			segmentController->setInitialVelocity(lastOdometryVel);
			segmentController->setTargetVelocity(nextTargetVel);

		}

		trajectorySet = true;

	}

	if(odometryUpdated && trajectorySet){

		isFollowing = true;
		trajectorySet = false;

	}

}

bool TrajectoryFollower::checkOdometry(geometry_msgs::Pose expectedPose){

	if(computeEuclideanDistance(expectedPose.position, lastOdometry.pose.pose.position) >  goalDistanceThreeshold){
		return false;
	}
	if(fabs( angles::normalize_angle(tf::getYaw(expectedPose.orientation)) -
			angles::normalize_angle(tf::getYaw(lastOdometry.pose.pose.orientation)) ) > goalOrientationThreeshold){
		return false;
	}
	return true;
}

void TrajectoryFollower::segmentControlHook(){

	float distance = segmentController->getDistance(lastOdometry.pose.pose, nextTargetPose);

	float goalAngle = segmentController->getYawAngleFromPose(nextTargetPose);
	float actualAngle = segmentController->getYawAngleFromPose(lastOdometry.pose.pose);
	float orientation = segmentController->getShortestAngle(goalAngle, actualAngle);

	// check whether the target is reached or not
	if(distance < goalDistanceThreeshold && fabs(orientation) < goalOrientationThreeshold){

		// target reached, replace it with the next one
		currentTrajectoryIndex ++;
		if(currentTrajectoryIndex < trajectoryTarget.waypoints.size()){

			std::cout << "Target reached" << std::endl;
			nextTargetPose = trajectoryTarget.waypoints.at(currentTrajectoryIndex).pose;

			nextTargetVel.v = trajectoryTarget.waypoints.at(currentTrajectoryIndex).linear_vel;
			nextTargetVel.w = trajectoryTarget.waypoints.at(currentTrajectoryIndex).angular_vel;

			segmentController->setInitialPose(lastOdometry.pose.pose);
			segmentController->setInitialVelocity(lastOdometryVel);
			segmentController->setTargetVelocity(nextTargetVel);

		}else{
			// goal reached, stop the robot
			std::cout << "Goal reached" << std::endl;
			std_msgs::String event;
			event.data = MOTION_DONE_EVENT;
			motionStatusOutPort.write(event);

			isFollowing = false;
			return;
		}
	}


	//cout << "Distance: " << distance;
	if (distance > goalDistanceThreeshold){
		imposedTwist = segmentController->computeLinearVelocity(lastOdometry.pose.pose, nextTargetPose);
	}

	//cout << " - Orientation: " << orientation << endl;
	if (fabs(orientation) > goalOrientationThreeshold){
		geometry_msgs::Twist rot;
		rot = segmentController->computeAngularVelocity(lastOdometry.pose.pose, nextTargetPose);
		imposedTwist.angular.z = rot.angular.z;
	}

}



}

ORO_CREATE_COMPONENT( Navigation::TrajectoryFollower );

