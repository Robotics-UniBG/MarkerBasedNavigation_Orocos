/********************************************************************************
 *
 * Coordinator
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
 * File: ArmDriver.cpp
 * Created: Jan 23, 2012
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

#include "Coordinator/CoordinatorComponent.hpp"

namespace Navigation{



CoordinatorComponent::CoordinatorComponent(string const& name)
: TaskContext(name),
  goalInputPort("goalInputPort"),
  eventInputPort("eventInputPort"),
  eventOutputPort("eventOutputPort"),
  goalMarkerIDOutputPort("goalMarkerIDOutputPort"),
  goalPoseOutputPort("goalPoseOutputPort")
{

	coord = new Coordinator();

	//Ports
	this->addEventPort("goalInputPort",goalInputPort).doc("...");
	this->addEventPort("eventInputPort",eventInputPort).doc("...");
	this->addPort("eventOutputPort",eventOutputPort).doc("...");
	this->addPort("goalMarkerIDOutputPort",goalMarkerIDOutputPort).doc("...");
	this->addPort("goalPoseOutputPort",goalPoseOutputPort).doc("...");

	//Properties
	this->addProperty("homingMarkerID",coord->homingMarkerID).doc("");


	//Attributes init
	this->addAttribute("state", coord->coordinatorStateString);
	this->addAttribute("trig", trig);
	previousState = "";

}


// HOOKS
bool CoordinatorComponent::startHook(){
	return startCoordinator();
}

bool CoordinatorComponent::configureHook(){
	//marshalling->readProperties("/properties/Kinematics.cpf");
	return configureCoordinator();
}

void CoordinatorComponent::stopHook(){
}

void CoordinatorComponent::cleanupHook(){

	//	marshalling->writeProperties("properties/Kinematics.cpf");
}




void CoordinatorComponent::updateHook() {
	//Current data for input ports;
	string currentInputGoal;
	std_msgs::String currentInputEvent;

	//Read from ports. If no data is available on port, the variable is set to NULL
	if(eventInputPort.read(currentInputEvent) == NewData){
		inputEvent = currentInputEvent.data;
	}else{
		inputEvent = NULL_EVENT;
	}

	if(goalInputPort.read(currentInputGoal) == NewData){
		inputGoal = currentInputGoal;
	}else{
		inputGoal = "6";
	}

	//Data for output ports
	std::string outputEvent = NULL_EVENT;
	int outputGoalID = NULLMARKER_ID;
	geometry_msgs::PoseStamped outputGoalPose;

	//CORODINATOR STATE MACHINE

	//Update state
	coord->computeStateMachine(inputGoal, inputEvent, outputGoalID, outputEvent);


	//Set the attribute for monitoring
	if(previousState.compare(coord->coordinatorStateString) != 0 ){
		previousState = coord->coordinatorStateString;
		cout << "Coordinator state changed: " << previousState << endl;
	}

	//Write output data (NULL data means that no data has to be written)
	if(outputEvent.compare(NULL_EVENT) != 0){
		std_msgs::String event;
		event.data = outputEvent;
		eventOutputPort.write(event);
	}
	if(outputGoalID != NULLMARKER_ID){
		std_msgs::Int32 makerGoalID;
		makerGoalID.data = outputGoalID;
		goalMarkerIDOutputPort.write(makerGoalID);
	}


}

//Kinematics Component methods
bool CoordinatorComponent::configureCoordinator(){
	return true;
}

bool CoordinatorComponent::startCoordinator(){
	return true;
}

void CoordinatorComponent::stopCoordinator(){
}


}//Namespace
ORO_CREATE_COMPONENT( Navigation::CoordinatorComponent );


