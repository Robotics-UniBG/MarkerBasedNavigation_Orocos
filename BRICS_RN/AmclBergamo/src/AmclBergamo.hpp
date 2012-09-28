/********************************************************************************
 *
 * AmclBergamo
 *
 * Copyright (c) 2012
 * All rights reserved.
 *
 * Luca Gherardi and Alexey Zakharov
 * University of Bergamo
 * Dept. of Information Technology and Mathematics
 *
 * -------------------------------------------------------------------------------
 *
 * File: AmclBergamo.hpp
 * Created: Feb 17, 2012
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

#ifndef TRAJECTORY_ADAPTER_HPP
#define TRAJECOTRY_ADAPTER_HPP

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>

#include <rtt/marsh/Marshalling.hpp>

#include <geometry_msgs/typekit/Types.hpp>
#include <brics_rn_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>
#include <nav_msgs/typekit/Types.hpp>

#include "amcl.hpp"

#include <vector>
#include <map>

using namespace std;
using namespace RTT;

namespace Navigation{

class AmclBergamo : public TaskContext{

public:

	AmclBergamo(string const& name);

private:


	OutputPort<geometry_msgs::PoseWithCovarianceStamped> robotPoseOutPort;

	InputPort<sensor_msgs::LaserScan> laserScanInPort;
	InputPort<nav_msgs::Odometry> odometryInPort;
	InputPort<geometry_msgs::PoseWithCovarianceStamped> initialPoseInPort;
	InputPort<nav_msgs::OccupancyGridConstPtr> occupancyGridPtrInPort;

	map<std::string, nav_msgs::OccupancyGridConstPtr> occupancyGridsMap;
	nav_msgs::OccupancyGridConstPtr currentOccupancyGrid;

	geometry_msgs::PoseWithCovarianceStamped initialPose;

	nav_msgs::Odometry lastOdometry;
	sensor_msgs::LaserScan lastLaserScan;

	AMCLocalizer* amcLocalizer;

	bool mapInitialized;

	boost::shared_ptr<Marshalling> marshalling;

	bool startHook();
	bool configureHook();
	void updateHook();
	void stopHook();
	void cleanupHook();


};

}

#endif
