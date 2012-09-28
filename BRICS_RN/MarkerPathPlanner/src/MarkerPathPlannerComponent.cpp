/********************************************************************************
 *
 * MarkerPathPlannerComponent
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
 * File: MarkerPathPlannerComponent.cpp
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

#include "MarkerPathPlanner/MarkerPathPlannerComponent.hpp"

#include <yaml-cpp/yaml.h>

namespace Navigation{


MarkerPathPlannerComponent::MarkerPathPlannerComponent(string const& name)
: TaskContext(name),
  goalIdInPort("goalIdInPort"),
  visibleMarkersInPort("visibleMarkersInPort"),
  markerIdPathOutPort("markerIdPathOutPort")
{

	this->addEventPort(goalIdInPort).doc("The goal marker id input port");
	this->addEventPort(visibleMarkersInPort).doc("The visible markers input port");
	this->addPort(markerIdPathOutPort).doc("Marker id Path output port");

	this->addAttribute("startId",startId);
	this->addAttribute("goalId",goalId);

	this->addProperty("pathA1_A2_file", pathA1_A2_file).doc("The path of file containing the markers between A1 and A2");
	this->addProperty("pathA2_A1_file", pathA2_A1_file).doc("The path of file containing the markers between A2 and A1");

	this->addAttribute("plannedPath",plannedPath);
	this->addAttribute("trig",trig);
	this->addAttribute("pathA1_A2",pathA1_A2);
	this->addAttribute("pathA2_A1",pathA2_A1);

	marshalling = this->getProvider<Marshalling>("marshalling");

	trig = false;

	pathA1_A2_file = "markerPaths/pathA1_A2.yaml";
	pathA2_A1_file = "markerPaths/pathA2_A1.yaml";

	//startId = 13;
	//goalId = 2;

}

bool MarkerPathPlannerComponent::startHook(){

	//	markerPathPlanner = new mbn_common::MarkerPathPlanner(pathA1_A2, pathA2_A1);
	markerPathPlanner = new mbn_common::MarkerPathPlannerComputation();
	if(! markerPathPlanner->loadPathA1_A2(pathA1_A2_file)){
		return false;
	}
	if(! markerPathPlanner->loadPathA2_A1(pathA2_A1_file)){
		return false;
	}
	pathA1_A2 = markerPathPlanner->getPathA1_A2();
	pathA2_A1 = markerPathPlanner->getPathA2_A1();
	return true;

}

bool MarkerPathPlannerComponent::configureHook(){


	return true;
	//	return loadPaths();

}

void MarkerPathPlannerComponent::stopHook(){



}

void MarkerPathPlannerComponent::cleanupHook(){
	//marshalling->writeProperties("properties/MarkerPathPlanner.cpf");
}

void MarkerPathPlannerComponent::updateHook() {

	std_msgs::Int32 goalIdInput;
	mbn_msgs::MarkersIDs markersIDsInput;
	vector<int> detectedMarkersIDs;


	if(goalIdInPort.read(goalIdInput) == NewData || trig){

		if(!trig){
			goalId = goalIdInput.data;
		}else{
			// just for testing without the others components
			detectedMarkersIDs.push_back(1);
			detectedMarkersIDs.push_back(3);
			detectedMarkersIDs.push_back(12);
			detectedMarkersIDs.push_back(14);
			detectedMarkersIDs.push_back(7);
			detectedMarkersIDs.push_back(8);
		}
		goalReceived = true;


		trig = false;
		return;
	}

	visibleMarkersIDs.markersIDs.clear();

	if(visibleMarkersInPort.read(markersIDsInput) == NewData){
		visibleMarkersIDs = markersIDsInput;
	}

	if(goalReceived){

		markerPathPlanner->setVisibleMarkersIDs(visibleMarkersIDs.markersIDs);

		if(markerPathPlanner->computePath(goalId, plannedPath)){

			log(Info) << "Marker path computed: ";
			startId = *(plannedPath.begin());
			for(vector<int>::iterator it = plannedPath.begin(); it!=plannedPath.end(); it++){
				log(Info) << *it;
				if(it != (plannedPath.end()-1)){
					log(Info) << ", ";
				}else{
					log(Info) << endlog();
				}
			}
			mbn_msgs::MarkersIDs markerIDsPath;

			markerIDsPath.markersIDs = plannedPath;
			markerIdPathOutPort.write(markerIDsPath);
			goalReceived = false;

		}else{
			// log(Info) << "There is no path between " << startId << " and the visible markers" << endlog();
		}
	}

}

//bool MarkerPathPlannerComponent::loadPaths(){
//
//	string pathFile = "markerPaths/pathA1_A2.yaml";
//	std::ifstream pathDescritptorFile(pathFile.c_str());
//	if (pathDescritptorFile.fail()) {
//		log(Error) << "Marker Path Planner could not open " << pathFile.c_str() << endlog();
//		return false;
//	}
//	YAML::Parser parser(pathDescritptorFile);
//	YAML::Node doc;
//	parser.GetNextDocument(doc);
//
//	int id;
//
//	const YAML::Node& markers = doc["markers"];
//	for(unsigned i=0;i<markers.size();i++) {
//		int id;
//		try {
//			markers[i] >> id;
//		} catch (YAML::InvalidScalar&) {
//			log(Error) << "Something went wrong during the parsing of pathA1_A2" << endlog();
//			return false;
//		}
//		pathA1_A2.push_back(id);
//	}
//
//	pathFile = "markerPaths/pathA2_A1.yaml";
//	std::ifstream pathDescritptorFile2(pathFile.c_str());
//	if (pathDescritptorFile2.fail()) {
//		log(Error) << "Marker Path Planner could not open " << pathFile.c_str() << endlog();
//		return false;
//	}
//	YAML::Parser parser2(pathDescritptorFile2);
//	parser2.GetNextDocument(doc);
//
//	const YAML::Node& markers2 = doc["markers"];
//	for(unsigned i=0;i<markers2.size();i++) {
//		try {
//			markers2[i] >> id;
//		} catch (YAML::InvalidScalar&) {
//			log(Error) << "Something went wrong during the parsing of pathA2_A1" << endlog();
//			return false;
//		}
//		pathA2_A1.push_back(id);
//	}
//
//	return true;
//
//}

}

ORO_CREATE_COMPONENT( Navigation::MarkerPathPlannerComponent );

