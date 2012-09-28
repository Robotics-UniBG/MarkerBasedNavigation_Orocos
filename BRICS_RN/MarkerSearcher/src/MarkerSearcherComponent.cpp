/********************************************************************************
 *
 * MarkerSearcherComponent
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
 * File: MarkerSearcherComponent.cpp
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

#include "MarkerSearcher/MarkerSearcherComponent.hpp"


namespace Navigation{


MarkerSearcherComponent::MarkerSearcherComponent(string const& name)
: TaskContext(name),
  eventInPort("eventInPort"),
  markersIDsInPort("markersIDsInPort"),
  odometryInPort("odometryInPort"),
  eventOutPort("eventOutPort"),
  pathOutPort("pathOutPort")
{

	markerSearcherComputation = new mbn_common::MarkerSearcherComputation();
	markerSearcherCoordination = new mbn_common::MarkerSearcherCoordination(markerSearcherComputation);

	this->addEventPort(eventInPort).doc("The event input port");
	this->addPort(markersIDsInPort).doc("The marker ID input port");
	this->addPort(odometryInPort).doc("The Odometry input port");

	this->addPort(eventOutPort).doc("The event output port");
	this->addPort(pathOutPort).doc("The path out port");

	this->addProperty("angleIncrement",angleIncrement).
			doc("The orientation increment that has to be sent to the robot each period");

	this->addAttribute("targetMarker",targetMarker);
	this->addAttribute("enabled",enabled);

	marshalling = this->getProvider<Marshalling>("marshalling");

	enabled = false;

}

bool MarkerSearcherComponent::startHook(){

	return true;

}

bool MarkerSearcherComponent::configureHook(){

	markerSearcherComputation->setAngleIncrement(angleIncrement);
	return true;

}

void MarkerSearcherComponent::stopHook(){



}

void MarkerSearcherComponent::cleanupHook(){
	//marshalling->writeProperties("properties/MarkerSearcher.cpf");
}

void MarkerSearcherComponent::updateHook() {

	std_msgs::String eventInput;
	mbn_msgs::MarkersIDs markersIDsInput;

	if(eventInPort.read(eventInput) == NewData){

		string event = eventInput.data;

		if(event.compare(mbn_common::GO_EVENT) == 0){
			enabled = false;
		}else if(event.compare(0,mbn_common::SEARCH_EVENT.length(),mbn_common::SEARCH_EVENT) == 0){

			enabled = true;
			int startIndex = event.find(":") + 1;
			string markerID = event.substr(startIndex,event.length()-startIndex);
			// save marker target, in this way it is visible from the RTT deployer
			targetMarker = atoi(markerID.c_str());
			markerSearcherComputation->setTargetMarkerID(targetMarker);

			markerSearcherCoordination->notifyTargetMarkerIDReceived();

			log(Info) << "Looking for marker " << targetMarker << endlog();
		}else if(event.compare(mbn_common::MOTION_STARTED_EVENT) == 0){
			markerSearcherCoordination->notifyMotionStarted();
		}else if(event.compare(mbn_common::POSE_GOAL_REACHED_EVENT) == 0){
			markerSearcherCoordination->notifySearchPoseReached();
		}

		return;

	}

	if(enabled){

		nav_msgs::Odometry odometryInput;

		if(odometryInPort.read(odometryInput) == NewData){

			tf::Pose lastOdometryPose;
			tf::poseMsgToTF(odometryInput.pose.pose, lastOdometryPose);
			markerSearcherComputation->setOdometryPose(lastOdometryPose);

		}

		visibleMarkersIDs.markersIDs.clear();

		if(markersIDsInPort.read(markersIDsInput) == NewData){

			visibleMarkersIDs = markersIDsInput;

		}

		markerSearcherComputation->setVisibleMarkersIDs(visibleMarkersIDs.markersIDs);
		markerSearcherCoordination->notifyVisibleMarkersIDsReceived();
		markerSearcherCoordination->notifyTimeElapsed();

		std_msgs::String event;
		event.data = markerSearcherCoordination->getOutputEvent();
		if(event.data.compare(mbn_common::TARGET_MARKER_FOUND_EVENT) == 0){
			enabled = false;
		}
		eventOutPort.write(event);

		geometry_msgs::PoseStamped goalPoseStamped;

		tf::Pose goalTfPose;
		if(markerSearcherComputation->getNextSearchPose(goalTfPose)){

			tf::poseTFToMsg(goalTfPose, goalPoseStamped.pose);

			nav_msgs::Path outputPath;
			outputPath.poses.push_back(goalPoseStamped);
			pathOutPort.write(outputPath);

		}

	}

}


}


ORO_CREATE_COMPONENT( Navigation::MarkerSearcherComponent );

