/*
 * CoordinatorMapAndMarkers.cpp
 *
 *  Created on: Jun 11, 2012
 *      Author: andrea
 */

#include "Coordinator/Coordinator.hpp"
#include "mbn_common/Events.hpp"

using namespace std;

namespace Navigation {

Coordinator::Coordinator() {
}

Coordinator::~Coordinator() {

}

void Coordinator::startCoordinator(){
	this->coordinatorState = INIT;
}

void Coordinator::setCoordinatorState(CoordinatorMarkerState state){
	this->coordinatorState = state;
}

void Coordinator::computeStateMachine(string inputGoal, string inputEvent, int& outputMarkerID, string& outputEvent){

	//Parse input events
	int index = inputEvent.find(":") + 1;
	string markerIdStr;
	if(index != 0){ //Event + ID received
		markerIdStr = inputEvent.substr(index,inputEvent.length()-index);
	}else{ //Event witohout ID received, force to NULLMARKER the ID field
		markerIdStr = NULL_EVENT;
	}


	switch(coordinatorState){
	case INIT: 		//exit: [START] / SEARCH ID -> SEARCH
		//Generate output goal ID
		outputMarkerID = NULLMARKER_ID;  //No output
		//Generate output event
		outputEvent = mbn_common::SEARCH_EVENT + homingMarkerID;
		//Change state
		this->coordinatorState = SEARCH;
		homing = true;
		break;

	case SEARCH:     //exit: [MARKER ID FOUND] / GO + GOAL ID -> MARKER_BASED_NAV
		if(inputEvent.compare(0, mbn_common::TARGET_MARKER_FOUND_EVENT.length(),mbn_common:: TARGET_MARKER_FOUND_EVENT) == 0){
			//			if(markerIdStr.compare(this->homingMarkerID) == 0){ //if homing found
			//Generate output goal ID
			if(homing){
				outputMarkerID = atoi(this->homingMarkerID.c_str());
			}
			//Generate output event
			outputEvent = mbn_common::GO_EVENT;
			//Change state
			this->coordinatorState = MARKER_BASED_NAV;
		}
		break;

	case MARKER_BASED_NAV: //exit1: [MARKER ID NOT FOUND] / search ID -> SEARCH |||| exit2: [MARKER GOAL REACHED] -> IDLE
		if(inputEvent.compare(0, mbn_common::TARGET_MARKER_NOT_FOUND_EVENT.length(), mbn_common::TARGET_MARKER_NOT_FOUND_EVENT) == 0){
			//Generate output goal ID
			outputMarkerID = NULLMARKER_ID;  //No output
			//Generate output event
			outputEvent = mbn_common::SEARCH_EVENT + markerIdStr;
			//Change state
			this->coordinatorState = SEARCH;
		}else if(inputEvent.compare(0, mbn_common::MARKER_GOAL_REACHED_EVENT.length(), mbn_common::MARKER_GOAL_REACHED_EVENT) == 0){

			if(homing){
				homing = false;
			}
			//Generate output goal ID
			outputMarkerID = NULLMARKER_ID;  //No output
			//Generate output event
			outputEvent = NULL_EVENT;        //No output
			//Change state
			this->coordinatorState = IDLE;
		}
		break;

	case IDLE: // exit: [new goal input] / GO + GOAL ID
		if(inputGoal.compare(NULL_EVENT) != 0){ //New goal arrived
			//Generate output goal ID
			outputMarkerID = atoi(inputGoal.c_str());  //No output
			//Generate output event
			outputEvent = mbn_common::GO_EVENT;        //No output
			//Change state
			this->coordinatorState = MARKER_BASED_NAV;
		}
		break;


	}

	//Update state string for attribute monitoring
	this->coordinatorStateString = coordinatorStateToString();
}

string Coordinator::coordinatorStateToString(){
	string state;
	switch(this->coordinatorState){
	case INIT:
		state = "INIT";
		break;

	case IDLE:
		state = "IDLE";
		break;

	case SEARCH:
		state = "SEARCH";
		break;

	case MARKER_BASED_NAV:
		state = "MARKER_BASED_NAV";
		break;
	}

	return state;
}

} /* namespace Navigation */
