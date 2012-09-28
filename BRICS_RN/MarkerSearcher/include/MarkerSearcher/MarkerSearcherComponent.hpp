/********************************************************************************
 *
 * MarkerSearcherComponent
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
 * File: MarkerSearcherComponent.hpp
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

#ifndef MARKER_PATH_PLANNER_COMPONENT_HPP
#define MARKER_PATH_PLANNER_COMPONENT_HPP


#include <rtt/os/main.h>

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Component.hpp>

#include <rtt/marsh/Marshalling.hpp>

#include <stdio.h>
#include <vector>

#include <tf/tf.h>

#include <nav_msgs/typekit/Types.hpp>
#include <std_msgs/typekit/Types.hpp>
#include <mbn_msgs/typekit/Types.hpp>

#include "mbn_common/MarkerSearcherCoordination.hpp"
#include "mbn_common/MarkerSearcherComputation.hpp"
#include "mbn_common/Events.hpp"

using namespace std;
using namespace RTT;

namespace Navigation{

class MarkerSearcherComponent : public TaskContext{

public:

	MarkerSearcherComponent(string const& name);

private:


	InputPort<std_msgs::String> eventInPort;
	InputPort<mbn_msgs::MarkersIDs> markersIDsInPort;
	InputPort<nav_msgs::Odometry> odometryInPort;

	OutputPort<std_msgs::String> eventOutPort;
	OutputPort<nav_msgs::Path> pathOutPort;

	int targetMarker;

	double angleIncrement;
	mbn_msgs::MarkersIDs visibleMarkersIDs;

	mbn_common::MarkerSearcherCoordination* markerSearcherCoordination;
	mbn_common::MarkerSearcherComputation* markerSearcherComputation;

	boost::shared_ptr<Marshalling> marshalling;

	bool enabled;

	bool startHook();
	bool configureHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

};

}

#endif
