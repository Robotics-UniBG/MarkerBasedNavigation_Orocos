/********************************************************************************
 *
 * RGBCameraOpenCv
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
 * File: RGBCameraOpenCv.cpp
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

#include "RGBCameraDriver/RGBCameraDriverComponent.hpp"

#include <rtt/os/main.h>

#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Service.hpp>
#include <rtt/Component.hpp>


#include <math.h>
#include <sys/time.h>

#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

using namespace std;
using namespace mbn_common;

namespace VisualNavigation{


RGBCameraDriverComponent::RGBCameraDriverComponent(string const& name)
: TaskContext(name),
  imageOutPort("imageOutPort")
{
	imageOutPort.keepLastWrittenValue(true);

	this->addPort(imageOutPort).doc("Image output port (bgr8 encoding)");

	this->addProperty("cameraDevice",cameraDevice).doc("you must set only number N of device (eg./dev/videoN)");
	this->addProperty("alwaysCopy",alwaysCopy).doc("...");
	this->addProperty("width",width).doc("...");
	this->addProperty("height",height).doc("...");
	this->addProperty("numOfImagesToDiscard", numOfImagesToDiscard).doc("...");

	marshalling = this->getProvider<Marshalling>("marshalling");

}

bool RGBCameraDriverComponent::startHook(){

	return true;

}

bool RGBCameraDriverComponent::configureHook(){
	rgbCameraDriver=new RGBCameraDriver();
	rgbCameraDriver->setCameraDevice(cameraDevice);
	rgbCameraDriver->setAlwaysCopy(alwaysCopy);
	rgbCameraDriver->setWidth(width);
	rgbCameraDriver->setHeight(height);
	return rgbCameraDriver->configureRGBCameraOpenCv();
}

void RGBCameraDriverComponent::stopHook(){
	return;

}

void RGBCameraDriverComponent::cleanupHook(){
//	marshalling->writeProperties("properties/RGBCameraOpenCv.cpf");
}

void RGBCameraDriverComponent::updateHook() {
//	RTT::os::TimeService::ticks timestamp = RTT::os::TimeService::Instance()->getTicks();
	cv_bridge::CvImage image;

	image.encoding=sensor_msgs::image_encodings::BGR8;
	rgbCameraDriver->getImage(image.image);


	image.header.stamp= ros::Time::now();
	sensor_msgs::ImageConstPtr imagePtr = image.toImageMsg();

	if (numOfImagesToDiscard<0)
		imageOutPort.write(imagePtr);

	//Seconds elapsed = RTT::os::TimeService::Instance()->secondsSince(timestamp);

	if(numOfImagesToDiscard>=0)
		numOfImagesToDiscard--;

}


}

ORO_CREATE_COMPONENT( VisualNavigation::RGBCameraDriverComponent );
