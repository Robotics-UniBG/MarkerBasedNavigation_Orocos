/********************************************************************************
 *
 * GoalPublisher
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
 * File: GoalPublisher.cpp
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

#include "GoalPublisher/GoalPublisher.hpp"

#include <tf/tf.h>

#include <SDL/SDL_image.h>
#include "yaml-cpp/yaml.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace Navigation{


GoalPublisher::GoalPublisher(string const& name)
: TaskContext(name),
  goalOutPort("goalOutPort")
{

	this->addPort(goalOutPort).doc("The goal output port");

	marshalling = this->getProvider<Marshalling>("marshalling");



}

bool GoalPublisher::startHook(){

	return true;

}

bool GoalPublisher::configureHook(){

	return true;

}

void GoalPublisher::stopHook(){



}

void GoalPublisher::cleanupHook(){
	//marshalling->writeProperties("properties/GoalPublisher.cpf");
}

void GoalPublisher::updateHook() {




}


}

ORO_CREATE_COMPONENT( Navigation::GoalPublisher );

