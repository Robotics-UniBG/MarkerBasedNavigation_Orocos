/********************************************************************************
 *
 * ArmDriver
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


#include "Slam/Converter.hpp"
#include "Slam/Slam.hpp"
#include <rtt/os/main.h>

#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Service.hpp>
#include <rtt/Component.hpp>

#include <math.h>
#include <sys/time.h>

#include <tf/tf.h>


using namespace std;
using namespace KDL;

namespace slam{



Slam::Slam(string const& name)
: TaskContext(name),
  markerPoseInputPort("markerPoseInputPort"),
  odometryInputPort("odometryInputPort"),
  markerAbsolutePoseOutputPort("markerAbsolutePoseOutputPort")
{
	//marshalling = this->getProvider<Marshalling>("marshalling");
	this->addEventPort("markerPoseInputPort",markerPoseInputPort).doc("Marker position input wrt the robot frame");
	this->addPort("odometryInputPort",odometryInputPort).doc("Odometry input port");
	this->addPort("markerAbsolutePoseOutputPort",markerAbsolutePoseOutputPort).doc("Marker absolute position wrt the world frame");

	//	markerRead = false;
	//	odometryRead = false;


}


// HOOKS
bool Slam::startHook(){
	return startSlam();
}

bool Slam::configureHook(){
	//marshalling->readProperties("/properties/Kinematics.cpf");
	return configureSlam();
}

void Slam::stopHook(){
}

void Slam::cleanupHook(){

	//	marshalling->writeProperties("properties/Kinematics.cpf");
}



void Slam::updateHook() {
	RTT::log(RTT::Info)<<"Running slam"<<RTT::endlog();

	//Read Marker pose
	brics_rn_msgs::MarkerPose markerPose_temp;
	if(markerPoseInputPort.read(markerPose_temp) == RTT::NewData){
		markerPose = markerPose_temp;
		//save timestamp
		ts = markerPose.header.stamp;
		//Convert odometry.transform to KDL frame to do the product
		converter.poseTokdl(markerPose.poseWRTRobotFrame, markerFrame);
		//		markerRead = true;
	}

	//Read odometry
	nav_msgs::Odometry odometry_temp;
	if(odometryInputPort.read(odometry_temp) == RTT::NewData){
		odometry = odometry_temp;
		//Convert markerpose.pose to KDL frame to do the product
		converter.poseTokdl(odometry.pose.pose, odometryFrame);
		//		odometryRead = true;
	}

	//	if(markerRead && odometryRead){
	//Product

	tf::Vector3 markerTranslation;
	markerTranslation.setX(markerPose.poseWRTRobotFrame.position.x);
	markerTranslation.setY(markerPose.poseWRTRobotFrame.position.y);
	markerTranslation.setZ(markerPose.poseWRTRobotFrame.position.z);

	tf::Quaternion markerRotation;
	markerRotation.setX(markerPose.poseWRTRobotFrame.orientation.x);
	markerRotation.setY(markerPose.poseWRTRobotFrame.orientation.y);
	markerRotation.setZ(markerPose.poseWRTRobotFrame.orientation.z);
	markerRotation.setW(markerPose.poseWRTRobotFrame.orientation.w);

	tf::Pose markerTfPose;
	markerTfPose.setOrigin(markerTranslation);
	markerTfPose.setRotation(markerRotation);

	tf::Pose odometryTfPose;
	tf::poseMsgToTF(odometry.pose.pose, odometryTfPose);

	tf::Pose markerAbsTfPose;
	markerAbsTfPose.mult(odometryTfPose, markerTfPose);

	markerAbsoluteFrame = odometryFrame*markerFrame;

	//Re-convert into a KDL frame
	converter.kdlToPose(markerAbsoluteFrame, markerAbsolutePose.pose);

	geometry_msgs::Pose markerAbsPose;
	tf::poseTFToMsg(markerAbsTfPose, markerAbsPose);

	cout << "Andrea - Tx:" << markerAbsolutePose.pose.position.x
			<< ", Ty" <<  markerAbsolutePose.pose.position.y
			<< ", Tz" <<  markerAbsolutePose.pose.position.z
			<< ", Rx" <<  markerAbsolutePose.pose.orientation.x
			<< ", Ry" <<  markerAbsolutePose.pose.orientation.y
			<< ", Rz" <<  markerAbsolutePose.pose.orientation.z
			<< ", Rw" <<  markerAbsolutePose.pose.orientation.w << endl;

	cout << "Luca - Tx:" << markerAbsPose.position.x
			<< ", Ty" <<  markerAbsPose.position.y
			<< ", Tz" <<  markerAbsPose.position.z
			<< ", Rx" <<  markerAbsPose.orientation.x
			<< ", Ry" <<  markerAbsPose.orientation.y
			<< ", Rz" <<  markerAbsPose.orientation.z
			<< ", Rw" <<  markerAbsPose.orientation.w << endl;

	//Add the timestamp
	markerAbsolutePose.header.stamp = ts;
	markerAbsolutePose.header.frame_id = "odom";
	//Write on port
	markerAbsolutePoseOutputPort.write(markerAbsolutePose);

	//		markerRead = false;
	//		odometryRead = false;
	//	}


}

//Kinematics Component methods
bool Slam::configureSlam(){
	return true;
}

bool Slam::startSlam(){
	return true;
}

void Slam::stopSlam(){
}
}//Namespace
ORO_CREATE_COMPONENT( slam::Slam );


