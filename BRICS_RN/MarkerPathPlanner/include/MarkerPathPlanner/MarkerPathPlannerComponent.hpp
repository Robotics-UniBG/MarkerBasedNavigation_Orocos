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
 * File: MarkerPathPlannerComponent.hpp
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

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include <std_msgs/typekit/Types.hpp>
#include <mbn_msgs/typekit/Types.hpp>

#include "mbn_common/MarkerPathPlannerComputation.hpp"

using namespace std;
using namespace RTT;

namespace Navigation{

class MarkerPathPlannerComponent : public TaskContext{

public:

	MarkerPathPlannerComponent(string const& name);

private:

	InputPort<std_msgs::Int32> goalIdInPort;
	InputPort<mbn_msgs::MarkersIDs> visibleMarkersInPort;
	OutputPort<mbn_msgs::MarkersIDs> markerIdPathOutPort; // Rviz debug

	int startId;
	int goalId;

	vector<int> pathA1_A2;
	vector<int> pathA2_A1;

	mbn_common::MarkerPathPlannerComputation* markerPathPlanner;
	vector<int> plannedPath;

	string pathA1_A2_file;
	string pathA2_A1_file;

	mbn_msgs::MarkersIDs visibleMarkersIDs;

	boost::shared_ptr<Marshalling> marshalling;

	bool goalReceived;
	bool trig;

	bool startHook();
	bool configureHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

//	bool loadPaths();

};

}

#endif
