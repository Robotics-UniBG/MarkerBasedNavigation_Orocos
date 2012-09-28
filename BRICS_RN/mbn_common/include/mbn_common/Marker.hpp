/********************************************************************************
 *
 * Marker
 *
 * Copyright (c) 2012
 * All rights reserved.
 *
 * Davide Brugali, Aldo Biziak, Luca Gherardi, Andrea Luzzana
 * University of Bergamo
 * Dept. of Information Technology and Mathematics
 *
 * -------------------------------------------------------------------------------
 *
 * File: Marker.hpp
 * Created: Jan 22, 2012
 *
 * Author: <A HREF="mailto:luca.gherardi@unibg.it">Luca Gherardi</A>
 * Author: <A HREF="mailto:andrea.luzzana@unibg.it">Andrea Luzzana</A>
 * Author: <A HREF="mailto:aldo.biziak@unibg.it">Aldo Biziak</A>
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
#ifndef MARKER_HPP_
#define MARKER_HPP_

#include <vector>
#include <string>
#include <tf/tf.h>
using namespace std;


namespace mbn_common{

class Marker{

public:

	/**
	 * Class constructor.
	 * \param[in] the id of the marker.
	 * \param[in] the dimension of the marker.
	 */
	Marker(int id, double width);

	/**
	 * \return marker ID
	 */
	int getId();

	/**
	 * \param[in] marker ID to set
	 */
	void setId(int id);

	/**
	 * \return marker width
	 */
	double getWidth();

	/**
	 * \param[in] marker width to set
	 */
	void setWidth(double width);

private:
	/**
	 * The marker size in millimeters.
	 * Note: every marker is a square so we need to specify only one dimension.
	 */
	double width;

	/**
	 * The inner marker ID.
	 * If marker has got a BCH encoding, its ID will be a number bound between 0 and 4095.
	 */
	int id;

};

}
#endif /* MARKER_HPP_ */
