/********************************************************************************
 *
 * PathPlanner
 *
 * Copyright (c) 2012
 * All rights reserved.
 *
 * Luca Gherardi and Alexey Zakharov
 * University of Bergamo
 * Dept. of Information Technology and Mathematics
 *
 * -------------------------------------------------------------------------------
 *
 * File: PathPlanner.cpp
 * Created: June 4, 2012
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

#include "PathPlanner/PathPlanner.hpp"

#include <rtt/os/main.h>

#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Service.hpp>
#include <rtt/Component.hpp>

#include <math.h>
#include <sys/time.h>

#include <tf/tf.h>
#include <angles/angles.h>

namespace Navigation{


PathPlanner::PathPlanner(string const& name)
: TaskContext(name),
  pathOutPort("pathOutPort"),
  goalInPort("goalInPort"),
  odometryInPort("odometryInPort")
{

	this->addEventPort(goalInPort).doc("Next goal position input port");
	this->addPort(odometryInPort).doc("Robot odometry input port");

	this->addPort(pathOutPort).doc("Computed path port");

	this->addAttribute("currentGoal",currentGoal);
	//	this->addProperty("splineEtaParameters",splineEtaParameters).doc("The eta parameters of the spline");

	this->addAttribute("plannedPath",plannedPath);

	marshalling = this->getProvider<Marshalling>("marshalling");

	this->addProperty("minDeltaDistance",minDeltaDistance).doc("");
	this->addProperty("minDeltaTheta",minDeltaTheta).doc("");
	this->addProperty("circumference",circumference).doc("");

	//	minDeltaDistance = 0.4;
	//	minDeltaTheta = 0.1;


}

bool PathPlanner::startHook(){

	return true;

}

bool PathPlanner::configureHook(){

	//marshalling = this->getProvider<Marshalling>("marshalling");
	return true;

}

void PathPlanner::stopHook(){



}

void PathPlanner::cleanupHook(){
	//marshalling->writeProperties("properties/PathPlanner.cpf");
}

void PathPlanner::updateHook() {

	geometry_msgs::PoseStamped goalInput;
	nav_msgs::Odometry odometryInput;

	if(odometryInPort.read(odometryInput) == NewData){

		lastOdometry = odometryInput;

	}

	if(goalInPort.read(goalInput) == NewData){

		currentGoal = goalInput;
		computePlan();
		pathOutPort.write(plannedPath);
	}

}


bool PathPlanner::computePlan(){

	plannedPath.poses.clear();

	double currentX = lastOdometry.pose.pose.position.x;
	double currentY = lastOdometry.pose.pose.position.y;
	double currentTheta = angles::normalize_angle(tf::getYaw(lastOdometry.pose.pose.orientation));

	double goalX = currentGoal.pose.position.x;
	double goalY = currentGoal.pose.position.y;
	double goalTheta = angles::normalize_angle(tf::getYaw(currentGoal.pose.orientation));

	double deltaX = goalX - currentX;
	double deltaY = goalY - currentY;

	double deltaDistance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

	double deltaTheta = angles::normalize_angle(goalTheta - currentTheta);

	int n1 = round(deltaDistance / minDeltaDistance) + 1;
	int n2 = round(fabs(deltaTheta) / minDeltaTheta) + 1;

	int numWaypoints = (n1 > n2) ? n1 : n2;


	double alpha = atan2(deltaY, deltaX) + M_PI;

	double midX = lastOdometry.pose.pose.position.x + deltaX / 2.0;
	double midY = lastOdometry.pose.pose.position.y + deltaY / 2.0;

	double dX = deltaX / numWaypoints;
	double dY = deltaY / numWaypoints;
	double dTheta = deltaTheta / numWaypoints;

	for(int i = 0; i < numWaypoints - 1; i++){

		geometry_msgs::PoseStamped pose;

		currentTheta += dTheta;

		if(circumference){

			alpha -= (M_PI / numWaypoints);
			pose.pose.position.x = midX + (deltaDistance / 2.0)*cos(alpha);
			pose.pose.position.y = midY + (deltaDistance / 2.0)*sin(alpha);
			pose.pose.orientation = tf::createQuaternionMsgFromYaw(currentTheta);

		}else{

			currentX += dX;
			currentY += dY;

			pose.pose.position.x = currentX;
			pose.pose.position.y = currentY;
			pose.pose.orientation = tf::createQuaternionMsgFromYaw(currentTheta);

		}

		pose.header.frame_id = "/odom";
		pose.header.seq = i;
		plannedPath.poses.push_back(pose);


	}

	geometry_msgs::PoseStamped pose;
	pose.pose = currentGoal.pose;
	pose.header.frame_id = "/odom";
	pose.header.seq = numWaypoints;
	plannedPath.poses.push_back(pose);

	plannedPath.header.frame_id = "/odom";
	plannedPath.header.stamp = ros::Time::now();

	return true;

}



}

ORO_CREATE_COMPONENT( Navigation::PathPlanner );

