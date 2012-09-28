/********************************************************************************
 *
 * RampGenerator
 *
 * Copyright (c) 2012
 * All rights reserved.
 *
 * Alexey Zakharov
 * ...
 * ...
 *
 * -------------------------------------------------------------------------------
 *
 * File: RampGenerator.h
 * Created: Mat 25, 2012
 *
 * Author: ...
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

#ifndef RAMP_GENERATOR_HPP
#define RAMP_GENERATOR_HPP


#include <math.h>

using namespace std;

namespace Navigation {

class RampGenerator{
public:

	RampGenerator();

	float computeVelocity();

	void setTotalDistance(float totalDistance);

	void setActualDistance (float actualDistance);

	float getActualDistance();

	float getActualVelocity();

	void setAcceleration (float acceleration);

	void setMaxVelocity (float maxVelocity); /*= 1.0*/

	void setMinVelocity (float minVelocity);

	void setInitialVelocity (float initialVelocity);

	void setFinalVelocity (float finalVelocity);




private:
	float totalDistance;
	float actualDistance;
	float acceleration;
	float maxVelocity; /*= 1.0*/
	float minVelocity; /*= 1.0*/
	float initialVelocity; /*= 0.1*/
	float finalVelocity; /*= 0*/
	float actualVelocity;
};

}

#endif
