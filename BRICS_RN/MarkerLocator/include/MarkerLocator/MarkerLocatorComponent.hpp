/********************************************************************************
 *
 * MarkerLocator
 *
 * Copyright (c) 2012
 * All rights reserved.
 *
 * Luca Gherardi and Aldo Biziak
 * University of Bergamo
 * Dept. of Information Technology and Mathematics
 *
 * -------------------------------------------------------------------------------
 *
 * File: MarkerLocator.hpp
 * Created: Jan 20, 2012
 *
 * Author: <A HREF="mailto:luca.gherardi@unibg.it">Luca Gherardi</A>
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
#ifndef MARKERLOCATORCOMPONENT_HPP_
#define MARKERLOCATORCOMPONENT_HPP_
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>
#include <rtt/os/main.h>
#include <rtt/Logger.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Service.hpp>
#include <rtt/Component.hpp>
#include <rtt/marsh/Marshalling.hpp>
#include <mbn_msgs/typekit/Types.hpp>
#include <std_msgs/typekit/Types.hpp>

//opencv bridge includes
#include <cv_bridge/cv_bridge.h>

#include <vector>
#include <string>
#include <tf/tf.h>

//our includes
#include "mbn_common/Utility.hpp"
#include "mbn_common/Marker.hpp"
#include "mbn_common/MarkerLocatorComputation.hpp"

using namespace RTT;
using namespace std;
using namespace Utilities;
using namespace mbn_common;

namespace VisualNavigation{


class MarkerLocatorComponent : public TaskContext{

public:
	/**
	 * Class constructor.
	 * \param[in] the name of the component instance.
	 */
	MarkerLocatorComponent(string const& name);

private:
	/**
	 * millimeters to meters conversion
	 */
	static const double MM_TO_METERS = 0.001;

	/**
	 * the inner MarkerLocator instance
	 */
	MarkerLocatorComputation *markerLocator;

	//INPUT PORTS
	// received image
	InputPort<sensor_msgs::ImageConstPtr> imageInPort;
	// camera position
	InputPort<geometry_msgs::PoseStamped> cameraAbsTransformInPort;

	//OUTPUT PORTS
	// marker pose
	OutputPort<mbn_msgs::MarkersPoses> markersPosesOutPort;
	//ID of found marker
	OutputPort<mbn_msgs::MarkersIDs> markersIDsOutPort;

	//PROPERTIES
	//  the name of the file that contains camera distorsion parameters
	string cameraDistorsionParamPath;
	// camera resolution
	int cameraWidth;
	int cameraHeight;
	// Black-White threshold,if value is <=0, a set of thresholds will be set automatically
	int	blackWhiteThreshold;
	// the minimal requested confidence necessary to recognize the markers
	double min_confidence;
	// if true, only the markers defined in markersVectorIDtoFind will be searched, you should enable this only if enableDetectOnlyTheBest is disabled
	bool enableIDfilter;
	// if true, marker locator will look for the marker with the best confidence in the image
	bool enableDetectOnlyTheBest;
	// if true it will use markers with BCH encoding, otherwise it will use 9bit encoding
	bool useBCH;
	// The default distance used for all markers
	double markersBaseWidth;
	// The list of markers ID that we want to use
	vector<int> markersVectorIDtoFind;
	// The list of markers dimensions
	vector<double> markersDimensions;
	// the static position of the camera
	geometry_msgs::Pose fixedCameratransform;

	//LOCAL VARIABLE
	geometry_msgs::PoseStamped poseInput;

	//inner utility class
	Utility utility;
	boost::shared_ptr<Marshalling> marshalling;

	//METHODS
	bool startHook();
	bool configureHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	/**
	 * Configures the inner MarkerLocator to work.
	 * Returns true if the cofigurations has success
	 */
	bool configureMarkerLocator();


};

}
#endif /* MARKERLOCATORCOMPONENT_HPP_ */
