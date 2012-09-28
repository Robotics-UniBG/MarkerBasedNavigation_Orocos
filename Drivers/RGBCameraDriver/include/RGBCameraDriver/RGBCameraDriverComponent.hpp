/********************************************************************************
 *
 * RGBCameraOpenCv
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
 * File: RGBCameraOpenCv.hpp
 * Created: Jan 23, 2012
 *
 * Author: <A HREF="mailto:luca.gherardi@unibg.it">Luca Gherardi</A>
 * 
 * Supervised by: <A HREF="mailto:brugali@unibg.it">Davide Brugali</A>
 * 
 * -------------------------------------------------------------------------------
 *
 * This sofware is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that useextern "C"{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappbuffer.h>
}rs of this
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
#ifndef RGBCAMERADRIVERCOMPONENT_HPP_
#define RGBCAMERADRIVERCOMPONENT_HPP_
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>

#include <rtt/marsh/Marshalling.hpp>

#include <geometry_msgs/typekit/Types.h>
#include <sensor_msgs/typekit/Types.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include "mbn_common/RGBCameraDriver.hpp"

using namespace RTT;
using namespace cv;
using namespace mbn_common;

namespace VisualNavigation{

class RGBCameraDriverComponent : public TaskContext{

public:

	RGBCameraDriverComponent(string const& name);

private:


	OutputPort<sensor_msgs::ImageConstPtr> imageOutPort;

	sensor_msgs::ImageConstPtr outputImage;

	boost::shared_ptr<Marshalling> marshalling;

	RGBCameraDriver *rgbCameraDriver;

	int cameraDevice;
	bool alwaysCopy;
	int width;
	int height;

	int numOfImagesToDiscard;

	bool startHook();
	bool configureHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	bool configureRGBCameraOpenCv();


};

}

#endif /*RGBCAMERADRIVERCOMPONENT_HPP_*/
