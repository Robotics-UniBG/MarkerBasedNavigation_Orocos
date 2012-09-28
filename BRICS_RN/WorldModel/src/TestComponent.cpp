/********************************************************************************
 *
 * TestComponent
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
 * File: TestComponent.cpp
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

#include "WorldModel/TestComponent.hpp"

#include <rtt/os/main.h>

#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Service.hpp>
#include <rtt/Component.hpp>

#include <math.h>
#include <sys/time.h>
#include <libgen.h>
#include <fstream>

#include <tf/tf.h>

#include <SDL/SDL_image.h>
#include "yaml-cpp/yaml.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace Navigation{


TestComponent::TestComponent(string const& name)
: TaskContext(name),
  occupancyGridInPort("occupancyGridInPort")
{

	this->addPort(occupancyGridInPort).doc("The occupancy grid input port");

}

bool TestComponent::startHook(){

	return true;

}

bool TestComponent::configureHook(){

	return true;

}

void TestComponent::stopHook(){



}

void TestComponent::cleanupHook(){

}

void TestComponent::updateHook() {

	nav_msgs::OccupancyGridConstPtr occupancyGridPtr;

	if(occupancyGridInPort.read(occupancyGridPtr) == NewData){

		cout << "Test component received this occupancy map:" << endl;

		for(unsigned int i= 0; i < occupancyGridPtr.get()->data.size(); i++){

			int value = static_cast<int16_t>(occupancyGridPtr.get()->data.at(i));
			cout << value << ", ";
			if((i+1) % occupancyGridPtr.get()->info.width == 0){
				cout << endl;
			}else{
				cout << "\t";
			}


		}
		cout << endl;

	}


}


}

ORO_CREATE_COMPONENT( Navigation::TestComponent );

