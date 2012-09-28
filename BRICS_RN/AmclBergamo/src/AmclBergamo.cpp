/********************************************************************************
 *
 * AmclBergamo
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
 * File: AmclBergamo.cpp
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

#include "AmclBergamo.hpp"

#include <rtt/os/main.h>

#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Service.hpp>
#include <rtt/Component.hpp>

#include <math.h>
#include <stdio.h>
#include <sys/time.h>

#include <tf/tf.h>
#include <angles/angles.h>

namespace Navigation{


AmclBergamo::AmclBergamo(string const& name)
: TaskContext(name),
  robotPoseOutPort("robotPoseOutPort"),
  laserScanInPort("laserScanInPort"),
  odometryInPort("odometryInPort"),
  initialPoseInPort("initialPoseInPort"),
  occupancyGridPtrInPort("occupancyGridPtrInPort")
{
// ...
	this->addPort(robotPoseOutPort).doc("The estimated robot position output port");

	this->addPort(laserScanInPort).doc("Laser scan input port");
	this->addPort(odometryInPort).doc("Odometry input port");
	this->addPort(initialPoseInPort).doc("Initial pose input port (Mean and covariance with which to (re-)initialize the particle filter)");
	this->addPort(occupancyGridPtrInPort).doc("The occupancy grid input port");


	amcLocalizer = new AMCLocalizer();
	mapInitialized = false;

	occupancyGridsMap.clear();


//	this->addAttribute("inputTrajectory",targetTrajectory);

//	this->addProperty("goalDistanceThreeshold",goalDistanceThreeshold)
//							.doc("min difference between current pose and target pose "
//									"in order to consider the goal position reached");

	marshalling = this->getProvider<Marshalling>("marshalling");


}

bool AmclBergamo::startHook(){

	return true;

}

bool AmclBergamo::configureHook(){

	//amcLocalizer->reconfigure();
	return true;

}

void AmclBergamo::stopHook(){



}

void AmclBergamo::cleanupHook(){
	marshalling->writeProperties("properties/AmclBergamo.cpf");
}

void AmclBergamo::updateHook() {

	nav_msgs::OccupancyGridConstPtr occupancyGridPtrInput;
	geometry_msgs::PoseWithCovarianceStamped initialPoseInput;
	sensor_msgs::LaserScan laserScanInput;
	nav_msgs::Odometry odometryInput;

	if(occupancyGridPtrInPort.read(occupancyGridPtrInput)){

//		if(occupancyGridsMap.count(occupancyGridPtrInput)){
//
//		}

		currentOccupancyGrid = occupancyGridPtrInput;
		std::cout << "map Received, size: " <<
				currentOccupancyGrid.get()->info.width << ", " <<
				currentOccupancyGrid.get()->info.height << std::endl;
		amcLocalizer->initMap(*currentOccupancyGrid);
		std::cout << "map initialized" << std::endl;
		amcLocalizer->reconfigure();
		std::cout << "reconfigured " << std::endl;
		mapInitialized = true;

	}

	if(odometryInPort.read(odometryInput) == NewData){

		lastOdometry = odometryInput;


	}

	if(laserScanInPort.read(laserScanInput) == NewData){

		if(mapInitialized){

			lastLaserScan = laserScanInput;

		}

	}

	if(initialPoseInPort.read(initialPoseInput) == NewData){

		initialPose = initialPoseInput;

	}

    // do something

//	geometry_msgs::PoseStamped estimatedPose;
//	robotPoseOutPort.write(estimatedPose);

}



}

ORO_CREATE_COMPONENT( Navigation::AmclBergamo );

