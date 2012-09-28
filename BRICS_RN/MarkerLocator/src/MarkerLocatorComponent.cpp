/********************************************************************************
 *
 * MarkerLocator
 *
 * Copyright (c) 2012
 * All rights reserved.
 *
 * Luca Gherardi and Aldo Biziak
 * University of Bergamo
 * Dept. of Information Technology and Mathematics
 *
 * -------------------------------------------------------------------------------
 *
 * File: MarkerLocator.cpp
 * Created: Jan 20, 2012
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

#include "MarkerLocator/MarkerLocatorComponent.hpp"
using namespace mbn_common;

namespace VisualNavigation{

MarkerLocatorComponent::MarkerLocatorComponent(string const& name)
: TaskContext(name),
  imageInPort("imageInPort"),
  cameraAbsTransformInPort("cameraAbsPoseInPort"),
  markersPosesOutPort("markersPosesOutPort"),
  markersIDsOutPort("markersIDsOutPort")
{

	markersPosesOutPort.keepLastWrittenValue(true);

	this->addPort(cameraAbsTransformInPort).doc("Camera absolute pose input port");
	this->addPort(imageInPort).doc("Image input port(bgr8 encoding)");
	this->addPort(markersPosesOutPort).doc("Marker absolute pose out port");
	this->addPort(markersIDsOutPort).doc("Marker ID out port");

	this->addProperty("markersBaseWidth",markersBaseWidth).doc("Markers default width is used if no specific width is specified");
	this->addProperty("markersVectorIDtoFind",markersVectorIDtoFind).doc("Markers IDs list");
	this->addProperty("markersDimensions",markersDimensions).doc("Markers dimensions list");
	this->addProperty("cameraDistorsionParamPath",cameraDistorsionParamPath).doc("Camera parameters file path");

	this->addProperty("cameraWidth",cameraWidth).doc("Camera width");
	this->addProperty("cameraHeight",cameraHeight).doc("Camera height");
	this->addProperty("useBCH",useBCH).doc("set to true if you want use BCH code");
	this->addProperty("enableIDfilter",enableIDfilter).doc("boolean to activate markerID filter");
	this->addProperty("enableDetectOnlyTheBest",enableDetectOnlyTheBest).doc("boolean to detect only the marker with the best confidence");
	this->addProperty("blackWhiteThreshold",blackWhiteThreshold).doc("set threshold for black/white, if value is <= 0, a value will be set automatically");
	this->addProperty("fixedCameratransform",fixedCameratransform).doc("this is the inner roto-translation that could be used if activateFixedCameraMode=true");

	marshalling = this->getProvider<Marshalling>("marshalling");
	markersBaseWidth=40.0;//this will be overwritten by MarkerLocator.cpf
	min_confidence=0.5;
}

bool MarkerLocatorComponent::startHook(){
	return true;
}

bool MarkerLocatorComponent::configureHook(){
	//	tf::Pose tempPose;
	//		tempPose.setOrigin(btVector3(0.290427,0.04,0.48475));
	//		tempPose.setRotation(btQuaternion(-0.605143,0.60452,-0.365789,0.366271));
	//tempPose.setOrigin(btVector3(0,0,0));
	//tempPose.setRotation(btQuaternion(0,0,0,1));
	//	tf::poseTFToMsg(tempPose, fixedCameratransform);
	return configureMarkerLocator();
}

void MarkerLocatorComponent::stopHook(){
	return;
}

void MarkerLocatorComponent::cleanupHook(){
	//				marshalling->writeProperties("properties/MarkerLocator.cpf");//this instruction allow the rewriting of MarkerLocator.cpf properties
}



void MarkerLocatorComponent::updateHook() {
	sensor_msgs::ImageConstPtr imageInput;
	FlowStatus flowStatus;


	flowStatus = imageInPort.read(imageInput);

	if(flowStatus == NewData){
		cv_bridge::CvImageConstPtr openCvImage;
		try{
			// convert ROS image to openCv image,
			// sharing it when possible
			openCvImage = cv_bridge::toCvShare(imageInput,"bgr8");
		}catch (cv_bridge::Exception & e){
			log(Error) << "Could not convert from " << imageInput->encoding.c_str() <<
					" to 'bgr8'." << endlog();
		}
		mbn_msgs::MarkersPoses detectedMarkersPoses;
		mbn_msgs::MarkersIDs detectedMarkersIDs;

		vector<int> vectorMarkersIDsFound;
		vector<tf::Pose> vectorMarkersPosesFound;
		markerLocator->findMarkers(openCvImage.get()->image.data,&vectorMarkersIDsFound,&vectorMarkersPosesFound);

		for(unsigned int x=0; x < vectorMarkersIDsFound.size(); x++){
			int currentMarkerID = vectorMarkersIDsFound.at(x);
			mbn_msgs::MarkerPose markerPose=mbn_msgs::MarkerPose();
			markerPose.marker_id=currentMarkerID;
			markerPose.header.stamp= imageInput->header.stamp;
			//log info useful for debugging
			log(Info) << "Marker found ID: " << markerPose.marker_id<<endlog();
			log(Info) << utility.getPositionAndOrientation(vectorMarkersPosesFound.at(x)) << endlog();

			tf::poseTFToMsg(vectorMarkersPosesFound.at(x), markerPose.poseWRTRobotFrame);

			detectedMarkersPoses.markersPoses.push_back(markerPose);
			detectedMarkersIDs.markersIDs.push_back(markerPose.marker_id);
		}
		markersPosesOutPort.write(detectedMarkersPoses);
		markersIDsOutPort.write(detectedMarkersIDs);

	}

}

bool MarkerLocatorComponent::configureMarkerLocator(){
	markerLocator=new MarkerLocatorComputation("MarkerLocator");
	if(!markerLocator->setCameraDistorsionParamPath(cameraDistorsionParamPath)){
		log(Error)<<"Error setting camera distorsion parameters"<<endl;
		return false;
	}
	if(!markerLocator->setCameraWidth(cameraWidth)){
		log(Error)<<"Error setting camera width"<<endl;
		return false;
	}
	if(!markerLocator->setCameraHeight(cameraHeight)){
		log(Error)<<"Error setting camera height"<<endl;
		return false;
	}
	if(!markerLocator->setMinConfidence(min_confidence)){//the minimal confidence used as edge to recognize the markers
		log(Error)<<"Error setting minimum confidence"<<endl;
		return false;
	}
	if(!markerLocator->setBlackWhiteThreshold(blackWhiteThreshold)){
		log(Error)<<"Error setting black/white threshold"<<endl;
		return false;
	}
	markerLocator->setEnableIDfilter(enableIDfilter);//if true, only the markers defined in markersVectorIDtoFind will be searched, you can enable this only if enableDetectOnlyTheBest is disabled
	markerLocator->setEnableDetectOnlyTheBest(enableDetectOnlyTheBest);//if true, marker locator will look for the marker with the best confidence in the image
	markerLocator->setUseBCH(useBCH);

	markerLocator->setMarkersBaseWidth(markersBaseWidth);//it's the default distance used for all markers

	markerLocator->markersVectorIDtoFind=markersVectorIDtoFind;//the list of BHC ID that we want to use
	markerLocator->markersDimensions=markersDimensions;//the list of markers dimensions
	tf::Pose cameraTFPose;
	tf::poseMsgToTF(fixedCameratransform, cameraTFPose);
	markerLocator->setCameraPose(cameraTFPose);

	return markerLocator->configureMarkerLocatorComputation();

}
}
ORO_CREATE_COMPONENT( VisualNavigation::MarkerLocatorComponent );
