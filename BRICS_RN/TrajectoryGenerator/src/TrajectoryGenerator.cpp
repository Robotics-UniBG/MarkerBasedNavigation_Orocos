/********************************************************************************
 *
 * TrajectoryGenerator
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
 * File: TrajectoryGenerator.cpp
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

#include "TrajectoryGenerator/TrajectoryGenerator.hpp"

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


TrajectoryGenerator::TrajectoryGenerator(string const& name)
: TaskContext(name),
  trajectoryOutPort("trajectoryOutPort"),
  pathInPort("pathInPort")
{

	this->addPort(trajectoryOutPort).doc("Computed trajectory output port");
	this->addEventPort(pathInPort).doc("Path input port");

	this->addAttribute("currentPath",currentPath);
	this->addAttribute("plannedTrajectory",plannedTrajectory);

	this->addProperty("maxLinVel",maxLinVel).doc("The input linear velocity of the robot");
	this->addProperty("maxAngAcc",maxAngVel).doc("The maximum angular acceleration of the robot");
	this->addProperty("maxAngVel",maxAngAcc).doc("The maximum angular velocity of the robot");

	marshalling = this->getProvider<Marshalling>("marshalling");


}

bool TrajectoryGenerator::startHook(){

	return true;

}

bool TrajectoryGenerator::configureHook(){

	marshalling = this->getProvider<Marshalling>("marshalling");
	return true;

}

void TrajectoryGenerator::stopHook(){



}

void TrajectoryGenerator::cleanupHook(){
	//marshalling->writeProperties("properties/TrajectoryGenerator.cpf");
}

void TrajectoryGenerator::updateHook() {

	geometry_msgs::PoseStamped goalInput;
	nav_msgs::Path pathInput;

	if(pathInPort.read(pathInput) == NewData){

		currentPath = pathInput;
		computeTrajectory();
		trajectoryOutPort.write(plannedTrajectory);
	}

}


bool TrajectoryGenerator::computeTrajectory(){

	plannedTrajectory.waypoints.clear();

	double previousX = 0;
	double previousY = 0;
	double previousTheta = 0;

	for(unsigned int i = 0; i < currentPath.poses.size(); i++){

		brics_rn_msgs::Waypoint waypoint;

		waypoint.pose = currentPath.poses.at(i).pose;

		if( i == 0 || i == currentPath.poses.size() - 1){

			waypoint.linear_vel = 0.0;
			waypoint.angular_vel = 0.0;
			previousX = waypoint.pose.position.x;
			previousY = waypoint.pose.position.y;
			previousTheta = angles::normalize_angle(tf::getYaw(waypoint.pose.orientation));

		}else{

			double deltaTheta = angles::normalize_angle(
					fabs(angles::normalize_angle(tf::getYaw(waypoint.pose.orientation)) - previousTheta));


			if(deltaTheta == 0){
				waypoint.linear_vel = maxLinVel;
			}else{

				// compute the time needed for completing the translation at input lin vel

				double deltaX = waypoint.pose.position.x - previousX;
				double deltaY = waypoint.pose.position.y - previousY;
				double deltaTranslation = sqrt(deltaX * deltaX + deltaY * deltaY);

				double timeAtInputLinVel = deltaTranslation / maxLinVel;
				//cout << "DS: " << deltaTranslation << " - t:" << timeAtInputLinVel << endl;

				// compute the time needed for completing the rotation at maxAngVel

				double angAccTime = (maxAngVel - 0) / maxAngAcc;

				// angle traveled when the robot has reached the max ang vel
				double accDistance = 0;
				double currentVel = 0;
				double dt = 0.001;

				for(double t = 0; t < angAccTime; t = t + dt){
					accDistance = currentVel * dt;
					currentVel += maxAngAcc * dt;
				}

				double timeForRotation = 2*angAccTime + (deltaTheta - 2*accDistance) / maxAngVel;
				//cout << "DO: " << deltaTheta << " - t:" << timeForRotation << endl;

				if(timeAtInputLinVel < timeForRotation){

					waypoint.linear_vel = maxLinVel * (timeAtInputLinVel / timeForRotation);

				}else{
					waypoint.linear_vel = maxLinVel;
				}

				//cout << "linerVel: " << waypoint.linear_vel << endl << endl;

			}

			waypoint.angular_vel = 0;

			//			double linTimeAtMaxVel = deltaTranslation / maxLinVel;
			//			double angTimeAtMaxVel = deltaTheta / maxAngVel;
			//
			//			if(linTimeAtMaxVel > angTimeAtMaxVel){
			//				waypoint.linear_vel = maxLinVel;
			//				waypoint.angular_vel = maxAngVel * (angTimeAtMaxVel/linTimeAtMaxVel);
			//			}else{
			//				waypoint.linear_vel = maxLinVel * (linTimeAtMaxVel/angTimeAtMaxVel);
			//				waypoint.angular_vel = maxAngVel;
			//			}

			previousX = waypoint.pose.position.x;
			previousY = waypoint.pose.position.y;
			previousTheta = angles::normalize_angle(tf::getYaw(waypoint.pose.orientation));

		}

		plannedTrajectory.waypoints.push_back(waypoint);

	}

	return true;

}



}

ORO_CREATE_COMPONENT( Navigation::TrajectoryGenerator );

