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
 * File: MarkerPathIteratorComponent.hpp
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

#ifndef MARKER_PATH_ITERATOR_COMPONENT_HPP
#define MARKER_PATH_ITERATOR_COMPONENT_HPP

#include <rtt/os/main.h>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>
#include <rtt/marsh/Marshalling.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Component.hpp>

#include <geometry_msgs/typekit/Types.hpp>
#include <nav_msgs/typekit/Types.hpp>
#include <mbn_msgs/typekit/Types.hpp>
#include <std_msgs/typekit/Types.hpp>
#include <tf/tf.h>

#include <vector>

#include "mbn_common/MarkerPathIteratorCoordination.hpp"
#include "mbn_common/MarkerPathIteratorComputation.hpp"
#include "mbn_common/Events.hpp"


using namespace RTT;
using namespace std;

namespace Navigation{


class MarkerPathIteratorComponent : public TaskContext{

public:

	MarkerPathIteratorComponent(string const& name);

private:

	//Ports
	InputPort<mbn_msgs::MarkersIDs> markerIdPathInPort;
	InputPort<mbn_msgs::MarkersPoses> markersPosesInPort;
	InputPort<nav_msgs::Odometry> odometryInPort;
	InputPort<std_msgs::String> eventInPort;

	OutputPort<nav_msgs::Path> pathOutPort;
	OutputPort<geometry_msgs::PoseArray> goalArrayOutPort;
	OutputPort<std_msgs::String> eventOutPort;

	//Marshalling
	boost::shared_ptr<Marshalling> marshalling;

	geometry_msgs::PoseArray goalArray;
	tf::Pose lastOdometryPose;
	mbn_msgs::MarkersPoses visibleMarkersPoses;

	mbn_common::MarkerPathIteratorCoordination* markerPathIteratorCoordination;
	mbn_common::MarkerPathIteratorComputation* markerPathIteratorComputation;

	int currentMarkerTarget;
	int nextMarkerTarget;

	/**
	 * This is true when the marker path iterator is enabled.
	 * It is set to true when the event go is received from the coordinator.
	 * It is set to false when the event search is received from the coordinator.
	 */
	bool enabled;

	bool pathReceived;

	/**
	 * It is used for updating the poseArray message in the right way.
	 * Just for rViz visulization
	 */
	int lastTargetMarkerRead;

	 //Methods
	 bool startHook();
	 bool configureHook();
	 void updateHook();
	 void stopHook();
	 void cleanupHook();

};

}

#endif
