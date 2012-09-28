/********************************************************************************
 *
 * WorldModel
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
 * File: WorldModel.hpp
 * Created: June 07, 2012
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

#ifndef WORLD_MODEL_HPP
#define WORLD_MODEL_HPP

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>

#include <rtt/marsh/Marshalling.hpp>

#include <geometry_msgs/typekit/Types.hpp>
#include <brics_rn_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>
#include <nav_msgs/typekit/Types.hpp>

#include <vector>
#include <map>

using namespace std;
using namespace RTT;

namespace Navigation{

class WorldModel : public TaskContext{

public:

	WorldModel(string const& name);

private:


	OutputPort<nav_msgs::OccupancyGridConstPtr> occupancyGridPtrOutPort;
	OutputPort<nav_msgs::OccupancyGrid> occupancyGridOutPort;

	InputPort<std::string> mapIdInPort;

	map<std::string, nav_msgs::OccupancyGridConstPtr> occupancyGridsMap;

	boost::shared_ptr<Marshalling> marshalling;

	bool trig;
	string mapName;

	bool startHook();
	bool configureHook();
	void updateHook();
	void stopHook();
	void cleanupHook();



	/**
	 * Read the image from file and return a pointer to the occupancy grid
	 *
	 * @param fname Path to the image file containing the occupancy data;
	 * can be absolute, or relative to the location of the YAML file

	 * @param res Resolution of the map, meters / pixel
	 *
	 * @param originX The 2-D X coord of the lower-left pixel in the map
	 * @param originY The 2-D Y coord of the lower-left pixel in the map
	 * @param originYaw The 2-D Yaw coord of the lower-left pixel in the map,
	 * with yaw as counterclockwise rotation (yaw=0 means no rotation).

	 *
	 * @throws std::runtime_error If the image file can't be loaded
	 * */
	bool loadOccupancyGridFromFile(string mapDescriptionPath,
			nav_msgs::OccupancyGrid& occupancyGrid);

};

}

#endif
