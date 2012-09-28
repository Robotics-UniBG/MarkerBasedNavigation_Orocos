/********************************************************************************
 *
 * RampGenerator
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
 * File: RampGenereator.cpp
 * Created: May 25, 2012
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



#include "TrajectoryFollower/RampGenerator.hpp"



namespace Navigation{

RampGenerator::RampGenerator()
{
	initialVelocity = 0;
	actualVelocity = 0;
	minVelocity = 0;
	maxVelocity = 0.1;
	acceleration = 0.5;
}

float RampGenerator::computeVelocity()
{
	if (totalDistance == 0)
	{
		actualVelocity = 0;
		return actualVelocity;
	}


	const float minVelocity = 0.1;
	const float maxVelocity = 1;
	//  initialVelocity = 0;
	// finalVelocity = 0;

	// acceleration is from 0 to 1;
	// 1 - pi / 2
	// 0 - 0
	// ROS_INFO("totalDist=%f, actualDist=%f", totalDistance, actualDistance);
	float velocity = 0;
	//float phi = acceleration * M_PI / 2.0;
	float halfTotalDistance = (totalDistance) / 2.0;
	float velocityLimit_ = halfTotalDistance * tan(acceleration);
	// TODO add check for admissible values;
	// ROS_INFO("phi=%f, actualDistance=%f, halfTotalDistance=%f, maxVelocity=%f", phi, actualDistance, halfTotalDistance, maxVelocity);
	if (fabs(actualDistance) <= fabs(halfTotalDistance))
	{

		velocity = ((velocityLimit_ - initialVelocity) / halfTotalDistance) * actualDistance + initialVelocity;
		//ROS_INFO("ACC=%f initialVel=%f actualDistance=%f", velocity, initialVelocity, actualDistance);
	}
	else if (fabs(actualDistance) > fabs(halfTotalDistance))
	{
		velocity = ((finalVelocity - velocityLimit_) / halfTotalDistance) * actualDistance - finalVelocity + 2 * velocityLimit_;
	}

	if ((actualDistance < 0) && (halfTotalDistance > 0))
	{

		velocity = tan(acceleration) * actualDistance - 2 * initialVelocity;
		velocity = +0.1;

		//initialTwist = actualTwist;
		//        ROS_INFO("HOLY CRAP! braking=%f initialVel=%f actualDistance=%f", velocity, initialVelocity, actualDistance);


	}


	if ((actualDistance > 0) && (halfTotalDistance < 0))
	{

		//  velocity = tan(acceleration) * actualDistance - 2 * initialVelocity;
		velocity = -0.1;

		//initialTwist = actualTwist;
		//      ROS_INFO("HOLY CRAP! braking=%f initialVel=%f actualDistance=%f", velocity, initialVelocity, actualDistance);


	}

	if ((velocity > -minVelocity) && (velocity < minVelocity) && (halfTotalDistance > 0))
	{
		velocity = minVelocity;
		//       ROS_INFO("setting +minVelocity");
	}
	if ((velocity > -minVelocity) && (velocity < minVelocity) && (halfTotalDistance < 0))
	{
		velocity = -minVelocity;
		//       ROS_INFO("setting -minVelocity");
	}

	if (velocity > maxVelocity)
		velocity = maxVelocity;

	if (velocity < -maxVelocity)
		velocity = maxVelocity;


	actualVelocity =  velocity;

	return actualVelocity;
}

void RampGenerator::setTotalDistance(float totalDistance)
{
	this->totalDistance = totalDistance;
}

void RampGenerator::setActualDistance (float actualDistance)
{
	this->actualDistance = actualDistance;
}


float RampGenerator::getActualDistance() {
	return actualDistance;
}

float RampGenerator::getActualVelocity() {
	return actualVelocity;
}

void RampGenerator::setAcceleration (float acceleration)
{
	this->acceleration = acceleration;
}



void RampGenerator::setMaxVelocity (float maxVelocity)
{
	this->maxVelocity = maxVelocity;
} /*= 1.0*/

void RampGenerator::setMinVelocity (float minVelocity) /*= 1.0*/
{
	this->minVelocity = minVelocity;
}

void RampGenerator::setInitialVelocity (float initialVelocity)
{
	this->initialVelocity = initialVelocity;
} /*= 0.1*/

void RampGenerator::setFinalVelocity (float finalVelocity)
{
	this->finalVelocity = finalVelocity;
} /*= 0*/

}
