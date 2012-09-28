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
 * File: OmnidriveController.h
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

#ifndef OMNIDRIVE_CONTROLLER_HPP
#define OMNIDRIVE_CONTROLLER_HPP

#include <nav_msgs/typekit/Types.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <brics_rn_msgs/typekit/Types.h>

#include <tf/tf.h>
#include <angles/angles.h>

#include "TrajectoryFollower/RampGenerator.hpp"

using namespace std;

namespace Navigation {

//class Ramp{
//public:
//
//	Ramp()
//	{
//		initialVelocity = 0;
//		actualVelocity = 0;
//	}
//
//	float compute()
//	{
//		if (totalDistance == 0)
//		{
//			actualVelocity = 0;
//			return actualVelocity;
//		}
//
//
//		const float minVelocity = 0.1;
//		const float maxVelocity = 1;
//		//  initialVelocity = 0;
//		// finalVelocity = 0;
//
//		// acceleration is from 0 to 1;
//		// 1 - pi / 2
//		// 0 - 0
//		// ROS_INFO("totalDist=%f, actualDist=%f", totalDistance, actualDistance);
//		float velocity = 0;
//		//float phi = acceleration * M_PI / 2.0;
//		float halfTotalDistance = (totalDistance) / 2.0;
//		float velocityLimit_ = halfTotalDistance * tan(acceleration);
//		// TODO add check for admissible values;
//		// ROS_INFO("phi=%f, actualDistance=%f, halfTotalDistance=%f, maxVelocity=%f", phi, actualDistance, halfTotalDistance, maxVelocity);
//		if (fabs(actualDistance) <= fabs(halfTotalDistance))
//		{
//
//			velocity = ((velocityLimit_ - initialVelocity) / halfTotalDistance) * actualDistance + initialVelocity;
//			//ROS_INFO("ACC=%f initialVel=%f actualDistance=%f", velocity, initialVelocity, actualDistance);
//		}
//		else if (fabs(actualDistance) > fabs(halfTotalDistance))
//		{
//			velocity = ((finalVelocity - velocityLimit_) / halfTotalDistance) * actualDistance - finalVelocity + 2 * velocityLimit_;
//		}
//
//		if ((actualDistance < 0) && (halfTotalDistance > 0))
//		{
//
//			velocity = tan(acceleration) * actualDistance - 2 * initialVelocity;
//			velocity = +0.1;
//
//			//initialTwist = actualTwist;
//			//        ROS_INFO("HOLY CRAP! braking=%f initialVel=%f actualDistance=%f", velocity, initialVelocity, actualDistance);
//
//
//		}
//
//
//		if ((actualDistance > 0) && (halfTotalDistance < 0))
//		{
//
//			//  velocity = tan(acceleration) * actualDistance - 2 * initialVelocity;
//			velocity = -0.1;
//
//			//initialTwist = actualTwist;
//			//      ROS_INFO("HOLY CRAP! braking=%f initialVel=%f actualDistance=%f", velocity, initialVelocity, actualDistance);
//
//
//		}
//
//		if ((velocity > -minVelocity) && (velocity < minVelocity) && (halfTotalDistance > 0))
//		{
//			velocity = minVelocity;
//			//       ROS_INFO("setting +minVelocity");
//		}
//		if ((velocity > -minVelocity) && (velocity < minVelocity) && (halfTotalDistance < 0))
//		{
//			velocity = -minVelocity;
//			//       ROS_INFO("setting -minVelocity");
//		}
//
//		if (velocity > maxVelocity)
//			velocity = maxVelocity;
//
//		if (velocity < -maxVelocity)
//			velocity = maxVelocity;
//
//
//		actualVelocity =  velocity;
//
//		return actualVelocity;
//	}
//
//	void setTotalDistance(float totalDistance)
//	{
//		this->totalDistance = totalDistance;
//	}
//
//	void setActualDistance (float actualDistance)
//	{
//		this->actualDistance = actualDistance;
//	}
//
//
//	float getActualDistance() {
//		return actualDistance;
//	}
//
//	float getActualVelocity() {
//		return actualVelocity;
//	}
//
//	void setAcceleration (float acceleration)
//	{
//		this->acceleration = acceleration;
//	}
//
//
//
//	void setMaxVelocity (float maxVelocity)
//	{
//		this->maxVelocity = maxVelocity;
//	} /*= 1.0*/
//
//	void setMinVelocity (float minVelocity) /*= 1.0*/
//	{
//		this->minVelocity = minVelocity;
//	}
//
//	void setInitialVelocity (float initialVelocity)
//	{
//		this->initialVelocity = initialVelocity;
//	} /*= 0.1*/
//
//	void setFinalVelocity (float finalVelocity)
//	{
//		this->finalVelocity = finalVelocity;
//	} /*= 0*/
//
//
//
//
//
//private:
//	float totalDistance;
//	float actualDistance;
//	float acceleration;
//	float maxVelocity; /*= 1.0*/
//	float minVelocity; /*= 1.0*/
//	float initialVelocity; /*= 0.1*/
//	float finalVelocity; /*= 0*/
//	float actualVelocity;
//};


class OmnidriveController {

private:


	//geometry_msgs::Pose actualPose;
	geometry_msgs::Pose initialPose;
	//geometry_msgs::Pose goalPose;

	//geometry_msgs::Twist actualTwist;
	//geometry_msgs::Twist initialTwist;
	//geometry_msgs::Twist goalTwist;


	//const float angularVelocity = 1.0;
	//const float linearVelocity = 1.0;

	//const float positionThreshold = 0.1;
	//const float orientationThreshold = 0.1;

	/**
	 * The min linear velocity that can be imposed to the robot
	 */
	float minLinearVelocity;

	/**
	 * The max linear velocity that can be imposed to the robot
	 */
	float maxLinearVelocity;

	/**
	 * The min angular velocity that can be imposed to the robot
	 */
	float minAngularVelocity;

	/**
	 * The max angular velocity that can be imposed to the robot
	 */
	float maxAngularVelocity;
	/**
	 * The min linear acceleration that can be imposed to the robot
	 */
	float minLinearAcceleration;

	/**
	 * The max linear acceleration that can be imposed to the robot
	 */
	float maxLinearAcceleration;

	/**
	 * The max angular acceleration that can be imposed to the robot
	 */
	float minAngularAcceleration;

	/**
	 * The max angular acceleration that can be imposed to the robot
	 */
	float maxAngularAcceleration;

	float linearVelocityRampSlope;
	float angularVelocityRampSlope;


	brics_rn_msgs::Velocity initialVelocity;
	brics_rn_msgs::Velocity targetVelocity;

	RampGenerator angularVelocityRamp;
	RampGenerator linearVelocityRamp;


	float rampGenearator(float totalDistance, float actualDistance,  float acceleration,
			float maxVelocity, float initialVelocity, float targetVelocity);


	//geometry_msgs::Pose2D poseToPose2D(const geometry_msgs::Pose pose);
	geometry_msgs::Pose pose2DToPose(const geometry_msgs::Pose2D pose2D);



public:

	OmnidriveController(float minLinearVelocity, float maxLinearVelocity,
			float minAngularVelocity, float maxAngularVelocity,
			float linearVelocityRampSlope, float angularVelocityRampSlope);

	virtual ~OmnidriveController();

	void setInitialVelocity(brics_rn_msgs::Velocity initialVelocity);

	void setTargetVelocity(brics_rn_msgs::Velocity targetVelocity);

	void setInitialPose(geometry_msgs::Pose initialPose);


	geometry_msgs::Twist computeAngularVelocity(geometry_msgs::Pose actualPose, geometry_msgs::Pose goalPose);

	geometry_msgs::Twist computeLinearVelocity(geometry_msgs::Pose actualPose, geometry_msgs::Pose goalPose);

	float getYawAngleFromPose(geometry_msgs::Pose pose);

	float getShortestAngle (float goalAngle, float actualAngle);

	float getDistance(geometry_msgs::Pose actualPose, geometry_msgs::Pose goalPose);

};
}

#endif
