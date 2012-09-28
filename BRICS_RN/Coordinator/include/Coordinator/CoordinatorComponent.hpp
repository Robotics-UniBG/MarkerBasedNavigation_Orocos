/********************************************************************************
 *
 * Coordinator
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

#include <rtt/os/main.h>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>
#include <rtt/marsh/Marshalling.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Component.hpp>

#include <geometry_msgs/typekit/Types.hpp>
#include <std_msgs/typekit/Types.hpp>

#include "Coordinator/Coordinator.hpp"

using namespace RTT;
using namespace std;

namespace Navigation{
//Constants


class CoordinatorComponent : public TaskContext{

public:

	CoordinatorComponent(string const& name);

private:

	//Ports
	InputPort<string> goalInputPort;
	InputPort<std_msgs::String> eventInputPort;

	OutputPort<std_msgs::String> eventOutputPort;
	OutputPort<std_msgs::Int32> goalMarkerIDOutputPort;
	OutputPort<geometry_msgs::PoseStamped> goalPoseOutputPort;

	//Data for input ports
	string inputGoal;
	string inputEvent;

	//Coordinator instance
	Coordinator* coord;

	//Attributes
	bool trig;
	string previousState;

	//Marshalling
//	boost::shared_ptr<Marshalling> marshalling;


	//Methods
	bool startHook();
	bool configureHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	bool configureCoordinator();
	bool startCoordinator();
	void stopCoordinator();


};

}

