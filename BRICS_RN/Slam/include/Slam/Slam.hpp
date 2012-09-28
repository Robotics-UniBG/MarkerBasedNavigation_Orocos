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
 * File: ArmDriver.hpp
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

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>
#include <rtt/marsh/Marshalling.hpp>
#include <rtt/Logger.hpp>

#include <kdl/frames_io.hpp>

#include <geometry_msgs/typekit/Types.h>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>
#include <youbot_msgs/typekit/Types.h>
#include "brics_rn_msgs/JointPositionsStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "brics_rn_msgs/MarkerPose.h"
#include "nav_msgs/typekit/Types.hpp"

#include "Slam/Converter.hpp"
#include "Slam/thresholdFilter.hpp"

using namespace RTT;

namespace slam{

class Slam : public TaskContext{

public:

	Slam(std::string const& name);

private:

	//Ports
	InputPort<brics_rn_msgs::MarkerPose> markerPoseInputPort;
	InputPort<nav_msgs::Odometry> odometryInputPort;
	OutputPort<geometry_msgs::PoseStamped> markerAbsolutePoseOutputPort;

	//Marshalling
	boost::shared_ptr<Marshalling> marshalling;

	//Properties
	double traslThreshold;
	double rotoThreshold;
	//Data
	brics_rn_msgs::MarkerPose markerPose;
	nav_msgs::Odometry odometry;
	geometry_msgs::PoseStamped markerAbsolutePose;
	ros::Time ts;
	geometry_msgs::Transform oldMarkerPositionTransform;

	KDL::Frame markerFrame;
	KDL::Frame markerAbsoluteFrame;
	KDL::Frame odometryFrame;
	Converter converter;
	thresholdFilter filter;

	//Methods
	bool startHook();
	bool configureHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	bool configureSlam();
	bool startSlam();
	void stopSlam();
};

}

