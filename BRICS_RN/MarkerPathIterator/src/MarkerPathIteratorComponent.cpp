/********************************************************************************
 *
 * MarkerPathIteratorComponent
 *
 * Copyright (c) 2012
 * All rights reserved.
 *
 * Luca Gherardi and Andrea Luzzana
 * University of Bergamo
 * Dept. of Information Technology and Mathematics
 *
 * -------------------------------------------------------------------------------
 *
 * File: MarkerPathIteratorComponent.cpp
 * Created: June 13, 2012
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


#include "MarkerPathIterator/MarkerPathIteratorComponent.hpp"

namespace Navigation{

MarkerPathIteratorComponent::MarkerPathIteratorComponent(string const& name)
: TaskContext(name),
  markerIdPathInPort("markerIdPathInPort"),
  markersPosesInPort("markerPoseInPort"),
  odometryInPort("odometryInPort"),
  eventInPort("eventInPort"),
  pathOutPort("pathOutPort"),
  goalArrayOutPort("goalArrayOutPort"),
  eventOutPort("eventOutPort")
{

	markerPathIteratorComputation = new mbn_common::MarkerPathIteratorComputation();
	markerPathIteratorCoordination = new mbn_common::MarkerPathIteratorCoordination(markerPathIteratorComputation);

	this->addEventPort("markerIdPathInPort",markerIdPathInPort).doc("Marker id Path input port");
	this->addEventPort("markersPosesInPort",markersPosesInPort).doc("Marker position input wrt the robot frame");
	this->addPort("odometryInPort",odometryInPort).doc("Odometry input port");
	this->addEventPort("eventInPort",eventInPort).doc("The event input port");

	this->addPort("pathOutPort",pathOutPort).doc("Path with the marker pose");
	this->addPort("goalArrayOutPort",goalArrayOutPort).doc("Pose array of the markers");
	this->addPort("eventOutPort",eventOutPort).doc("Pose event output port");

	this->addAttribute("currentMarkerGoal", currentMarkerTarget);
	this->addAttribute("nextMarkerGoal", nextMarkerTarget);
	this->addAttribute("lastTargetMarkerRead",lastTargetMarkerRead);
	this->addAttribute("enabled",enabled);

	currentMarkerTarget = markerPathIteratorComputation->getCurrentTargetMarkerID();
	nextMarkerTarget = markerPathIteratorComputation->getNextTargetMarkerID();
	lastTargetMarkerRead = -1;

	enabled = true;
	pathReceived = false;
	goalArray.poses.clear();
	goalArray.header.frame_id = "/odom";

	marshalling = this->getProvider<Marshalling>("marshalling");


}


bool MarkerPathIteratorComponent::startHook(){
	return true;
}

bool MarkerPathIteratorComponent::configureHook(){
	return true;
}

void MarkerPathIteratorComponent::stopHook(){
}

void MarkerPathIteratorComponent::cleanupHook(){

	//	marshalling->writeProperties("properties/MarkerPathIterator.cpf");
}



void MarkerPathIteratorComponent::updateHook() {

	//Read Marker pose
	mbn_msgs::MarkersIDs markerPathInput;
	mbn_msgs::MarkersPoses markersPosesInput;
	std_msgs::String eventInput;
	int goalId = -1;
	bool markerFound = false;
	tf::Pose goalTfPose;

	// read the input events
	if(eventInPort.read(eventInput) == NewData){

		string event = eventInput.data;
		if(event.compare(mbn_common::GO_EVENT) == 0){

			markerPathIteratorCoordination->notifyMarkerFound();
			enabled = true;

		}else if(event.compare(mbn_common::SEARCH_EVENT) == 0){

			enabled = false;

		}else if(event.compare(mbn_common::POSE_GOAL_REACHED_EVENT) == 0){

			markerPathIteratorCoordination->notifyCurrentMarkerReached();

		}else if(event.compare(mbn_common::MOTION_STARTED_EVENT) == 0){

			markerPathIteratorCoordination->notifyMotionStarted();

		}

		string outEvent = markerPathIteratorCoordination->getOutputEvent();
		if(outEvent.compare("") != 0){
			std_msgs::String eventMsg;
			eventMsg.data = outEvent;
			eventOutPort.write(eventMsg);
		}

		if(outEvent.compare("MARKER_GOAL_REACHED") == 0){
			// reach the goal, stop the marker reading
			enabled = false;
		}

		return;

	}

	if(markerIdPathInPort.read(markerPathInput) == NewData){

		pathReceived = true;
		markerPathIteratorComputation->setMarkerPath(markerPathInput.markersIDs);
		markerPathIteratorCoordination->notifyMarkerPathReceived();

		return;


	}

	visibleMarkersPoses.markersPoses.clear();
	if(markersPosesInPort.read(markersPosesInput) == RTT::NewData){

		visibleMarkersPoses = markersPosesInput;

	}

	/**
	 * read the input markers. It's done only if the marker path
	 * has been received and the computation is enabled
	 */
	if(pathReceived && enabled){

		nav_msgs::Odometry odometryInput;
		// read and update the odometry
		if(odometryInPort.read(odometryInput) == RTT::NewData){

			tf::poseMsgToTF(odometryInput.pose.pose, lastOdometryPose);
			markerPathIteratorComputation->setOdometryPose(lastOdometryPose);

		}

		nav_msgs::Path computedPath;
		geometry_msgs::PoseStamped markerAbsPoseStamped;

		vector<tf::Pose> receivedMarkersPoses;
		vector<int> receivedMarkersIDs;
		tf::Pose markerPose;

		if(visibleMarkersPoses.markersPoses.size() > 0){

			for(vector<mbn_msgs::MarkerPose>::iterator it = visibleMarkersPoses.markersPoses.begin();
					it != visibleMarkersPoses.markersPoses.end(); it++){

				tf::poseMsgToTF(it->poseWRTRobotFrame, markerPose);
				receivedMarkersPoses.push_back(markerPose);
				receivedMarkersIDs.push_back(it->marker_id);
			}

		}

		markerPathIteratorComputation->setDetectedMarkers(receivedMarkersPoses, receivedMarkersIDs);
		markerPathIteratorCoordination->notifyDetectedMarkersReceived();
		goalId = markerPathIteratorComputation->getGoalID();
		markerFound = markerPathIteratorComputation->getGoalPose(goalTfPose);

	}

	// some logging
	if(markerPathIteratorComputation->getCurrentTargetMarkerID() != currentMarkerTarget){
		log(Info) << "Marker goal changed: " << markerPathIteratorComputation->getCurrentTargetMarkerID() << endlog();
	}

	// Now retrieve the information that has to be write on the output ports.
	// It's not possible that events and marker pose come together.
	// Hence we will only write on one output port: event or path

	string event = markerPathIteratorCoordination->getOutputEvent();
	if(event.compare("") != 0){
		std_msgs::String eventMsg;
		eventMsg.data = event;
		eventOutPort.write(eventMsg);
	}

	if(event.compare("MARKER_GOAL_REACHED") == 0){
		// reach the goal, stop the marker reading
		enabled = false;
	}

	if(markerFound && enabled){

		log(Info) << "Detected marker " << goalId << endlog();

		nav_msgs::Path computedPath;
		geometry_msgs::PoseStamped goalPoseStamped;

		// retrieve the goal position, create the path and write it on the output port
		tf::poseTFToMsg(goalTfPose, goalPoseStamped.pose);
		goalPoseStamped.header.frame_id = "/odom";
		goalPoseStamped.header.seq = 0;
		computedPath.poses.push_back(goalPoseStamped);
		computedPath.header.frame_id = "/odom";
		computedPath.header.stamp = ros::Time::now();
		pathOutPort.write(computedPath);

		log(Info) << "Marker Pose - x : " << goalPoseStamped.pose.position.x
				<< " , y : " << goalPoseStamped.pose.position.y
				<< " , z : " << goalPoseStamped.pose.position.z
				<< endlog();

		// Update the markers geometry path for rViz
		// if the marker is new add it,
		// else remove the old one and update
		if(goalId == lastTargetMarkerRead){
			if(goalArray.poses.size()!=0){
				goalArray.poses.pop_back();
			}
		}
		goalArray.poses.push_back(goalPoseStamped.pose);

		goalArrayOutPort.write(goalArray);

		// update the last detected marker
		lastTargetMarkerRead = goalId;

	}else{


		// just logging
		log(Info) << "No target markers Detected" << endlog();
		// The vector of markers doesn't contain the current or the next marker goal
		// moreover the current marker hasn't been yet detected.
		// We have to raise an event

		// this is now done in the getEvent method

		//		stringstream event;
		//		event << TARGET_MARKER_NOT_FOUND_EVENT << markerPathIterator->currentMarkerTarget;
		//		eventOutPort.write(event.str());

	}

	currentMarkerTarget = markerPathIteratorComputation->getCurrentTargetMarkerID();
	nextMarkerTarget = markerPathIteratorComputation->getNextTargetMarkerID();

}






}

ORO_CREATE_COMPONENT( Navigation::MarkerPathIteratorComponent );


