/********************************************************************************
 *
 * OmnidriveController
 *
 * Copyright (c) 2012
 * All rights reserved.
 *
 * Alexey Zakharov
 * ...
 * ...
 *
 * -------------------------------------------------------------------------------
 *
 * File: OmnidriveController.cpp
 * Created: May 23, 2012
 *
 * ...
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

#include "TrajectoryFollower/OmnidriveController.hpp"



namespace Navigation{





OmnidriveController::OmnidriveController(float minLinearVelocity, float maxLinearVelocity,
		float minAngularVelocity, float maxAngularVelocity,
		float linearVelocityRampSlope, float angularVelocityRampSlope){
	this->minLinearVelocity = minLinearVelocity;
	this->maxLinearVelocity = maxLinearVelocity;
	this->minAngularVelocity = minAngularVelocity;
	this->maxAngularVelocity = maxAngularVelocity;
	this->linearVelocityRampSlope = linearVelocityRampSlope;
	this->angularVelocityRampSlope = angularVelocityRampSlope;
	initialVelocity.v = 0;
	initialVelocity.v_dot = 0;
	initialVelocity.w = 0;
	initialVelocity.w_dot = 0;
	targetVelocity.v = 0;
	targetVelocity.v_dot = 0;
	targetVelocity.w = 0;
	targetVelocity.w_dot = 0;
}

OmnidriveController::~OmnidriveController(){

}

void OmnidriveController::setInitialVelocity(brics_rn_msgs::Velocity initialVelocity){
	if(initialVelocity.v>0.1){
		this->initialVelocity.v = initialVelocity.v;
	}else{
		this->initialVelocity.v = 0.1;
	}
	if(initialVelocity.w>0.1){
		this->initialVelocity.w = initialVelocity.w;
	}else{
		this->initialVelocity.w = 0.1;
	}
	// TODO took this lines from goalPoseCallback
	linearVelocityRamp.setInitialVelocity(linearVelocityRamp.getActualVelocity());
	angularVelocityRamp.setInitialVelocity(angularVelocityRamp.getActualVelocity());
}

void OmnidriveController::setTargetVelocity(brics_rn_msgs::Velocity targetVelocity){
	this->targetVelocity = targetVelocity;
	linearVelocityRamp.setFinalVelocity(targetVelocity.v);
	angularVelocityRamp.setFinalVelocity(targetVelocity.w);
}

void OmnidriveController::setInitialPose(geometry_msgs::Pose initialPose){
	this->initialPose = initialPose;
}

float OmnidriveController::getYawAngleFromPose(geometry_msgs::Pose pose){

	tf::Quaternion orientation;
	tf::quaternionMsgToTF(pose.orientation, orientation);
	btScalar yaw, pitch, roll;
	btMatrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
	return  yaw;
}

float OmnidriveController::getShortestAngle (float goalAngle, float actualAngle) {
	return atan2(sin(goalAngle-actualAngle), cos(goalAngle-actualAngle));
}

float OmnidriveController::getDistance(geometry_msgs::Pose actualPose, geometry_msgs::Pose goalPose)
{
	return sqrt( (actualPose.position.x - goalPose.position.x) * (actualPose.position.x - goalPose.position.x) +
			(actualPose.position.y - goalPose.position.y) * (actualPose.position.y - goalPose.position.y) );

}


geometry_msgs::Twist OmnidriveController::computeAngularVelocity(geometry_msgs::Pose actualPose, geometry_msgs::Pose goalPose)
{

	float actualAngle = getYawAngleFromPose(actualPose);
	float initialAngle = getYawAngleFromPose(initialPose);
	float goalAngle = getYawAngleFromPose(goalPose);

	//   float acceleration = 0.5; // zero is no acceleration, 1 is a step function

	float totalDistance  = getShortestAngle(goalAngle, initialAngle);
	float actualDistance = totalDistance - getShortestAngle(goalAngle, actualAngle);


	angularVelocityRamp.setTotalDistance(totalDistance);
	angularVelocityRamp.setActualDistance(actualDistance);
	//TODO replaced acceleration with slope
	angularVelocityRamp.setAcceleration(angularVelocityRampSlope);

	float angularVelocity = angularVelocityRamp.computeVelocity();



	geometry_msgs::Twist twist;

	// float sign = getShortestAngle(goalAngle, actualAngle) / fabs(getShortestAngle(goalAngle, actualAngle));
	//ROS_INFO("sign=%f goal=%f, actual=%f", sign, goalAngle, actualAngle);

	twist.angular.z = angularVelocity;

	return twist;
}

geometry_msgs::Twist OmnidriveController::computeLinearVelocity(geometry_msgs::Pose actualPose, geometry_msgs::Pose goalPose)
{
	float actualAngle = getYawAngleFromPose(actualPose);
	//float goalAngle = getYawAngleFromPose(goalPose);
	geometry_msgs::Point goalPosition = goalPose.position;
	geometry_msgs::Point actualPosition = actualPose.position;
	geometry_msgs::Point initialPosition = initialPose.position;

	geometry_msgs::Twist twist;

	//float signLinearX = (goalPosition.x - actualPosition.x) / fabs(goalPosition.x - actualPosition.x);
	//float signLinearY = (goalPosition.y - actualPosition.y) / fabs(goalPosition.y - actualPosition.y);

	float totalDistance = getDistance(initialPose, goalPose);
	float actualDistance = totalDistance - getDistance(actualPose, goalPose);

	//float acceleration = 0.5; // zero is no acceleration, 1 is a step function


	//float linearVelocity = rampGenearator(totalDistance, actualDistance, acceleration,1,0.1);

	linearVelocityRamp.setTotalDistance(totalDistance);
	linearVelocityRamp.setActualDistance(actualDistance);
	//TODO replaced acceleration with slope
	linearVelocityRamp.setAcceleration(linearVelocityRampSlope);
	// linearVelocityRamp.setInitialVelocity(sqrt(initialTwist.linear.x*initialTwist.linear.x + initialTwist.linear.y*initialTwist.linear.y));

	float linearVelocity = linearVelocityRamp.computeVelocity();

	//cout << "linearVelocity = " << linearVelocity << endl;

	twist.linear.x = ((goalPosition.x * cos(actualAngle) + goalPosition.y * sin(actualAngle)) -
			(actualPosition.x * cos(actualAngle) + actualPosition.y * sin(actualAngle)));
	twist.linear.y = ((goalPosition.y * cos(actualAngle) - goalPosition.x * sin(actualAngle)) -
			(actualPosition.y * cos(actualAngle) - actualPosition.x * sin(actualAngle)));

	float x = fabs(twist.linear.x);
	float y = fabs(twist.linear.y);
	float norm = (x < y) ? y : x;


	//  ROS_INFO("initialVel=%f",sqrt(initialTwist.linear.x*initialTwist.linear.x + initialTwist.linear.y*initialTwist.linear.y));


	twist.linear.x = twist.linear.x / norm * linearVelocity;

	twist.linear.y = twist.linear.y / norm * linearVelocity;
	// ROS_INFO("%f -> %f", linearVelocity, sqrt(twist.linear.x*twist.linear.x+twist.linear.y*twist.linear.y));


	return twist;
}


//geometry_msgs::Pose2D OmnidriveController::poseToPose2D(const geometry_msgs::Pose pose){
//
//	geometry_msgs::Pose2D pose2D;
//	// use tf-pkg to convert angles
//	tf::Pose pose_tf;
//
//	// convert geometry_msgs::PoseS to tf::Pose
//	tf::poseMsgToTF(pose, pose_tf);
//
//	// now get Euler-Angles from pose_tf
//	float useless_pitch, useless_roll, yaw;
//	pose_tf.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);
//
//	// normalize angle
//	yaw = angles::normalize_angle(yaw);
//
//	// and set to pose2D
//	pose2D.x = pose.position.x;
//	pose2D.y = pose.position.y;
//	pose2D.theta = yaw;
//
//	return pose2D;
//}

geometry_msgs::Pose OmnidriveController::pose2DToPose(const geometry_msgs::Pose2D pose2D){

	geometry_msgs::Pose pose;

	// use tf-pkg to convert angles
	tf::Quaternion frame_quat;

	// transform angle from euler-angle to quaternion representation
	frame_quat = tf::createQuaternionFromYaw(pose2D.theta);

	// set position
	pose.position.x = pose2D.x;
	pose.position.y = pose2D.y;
	pose.position.z = 0.0;

	// set quaternion
	pose.orientation.x = frame_quat.x();
	pose.orientation.y = frame_quat.y();
	pose.orientation.z = frame_quat.z();
	pose.orientation.w = frame_quat.w();

	return pose;
}

}
