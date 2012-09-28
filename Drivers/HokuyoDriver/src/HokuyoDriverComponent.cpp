/********************************************************************************
 *
 * HokuyoDriver
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
 * File: HokuyoDriver.hpp
 * Created: May 10, 2012
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

#include "HokuyoDriver/HokuyoDriverComponent.hpp"

namespace Hokuyo{



HokuyoDriverComponent::HokuyoDriverComponent(string const& name)
: TaskContext(name),
  laserScanOutPort("laserScanOutPort"),
  diagnosticOutPort("diagnosticOutPort")
{


	this->addPort(laserScanOutPort).doc("Laser scan output");
	this->addPort(diagnosticOutPort).doc("Diagnostic output");

	marshalling = this->getProvider<Marshalling>("marshalling");

	this->addProperty("min_ang", hokuyoDriver.min_ang).
			doc("The angle of the first range measurement in radians (range is [-π,π], though most devices have a smaller feasible range)."); // (double, default: -π/2)

	this->addProperty("max_ang", hokuyoDriver.max_ang)
						.doc("The angle of the last range measurement in radians (range is [-π,π], though most devices have a smaller feasible range)."); // (double, default: π/2)

	this->addProperty("intensity", hokuyoDriver.intensity)
						.doc("Whether or not the hokuyo returns intensity values."); // (bool, default: false)

	this->addProperty("cluster", hokuyoDriver.cluster)
						.doc("The number of adjacent range measurements to cluster into a single reading; the shortest reading from the cluster is reported."); // (int, default: 1)

	this->addProperty("skip", hokuyoDriver.skip)
						.doc("The number of scans to skip between each measured scan. This controls the update rate. For a UTM-30LX, the hokuyo will scan at 40Hz, so setting skip to 1 makes it publish at 20Hz."); // (int, default: 0)

	this->addProperty("port", hokuyoDriver.port)
						.doc("The port where the hokuyo device can be found (default: /dev/ttyACM0s)."); // (string, default: /dev/ttyACM0)

	this->addProperty("calibrate_time", hokuyoDriver.calibrate_time)
						.doc("Whether the node should calibrate the hokuyo's time offset on startup. If true, the node will exchange of series of messages with the device in order to determine the time delay in the USB connection. This calibration step is necessary to produce accurate time stamps on scans."); // (bool, default: true)

	this->addProperty("frame_id", hokuyoDriver.frame_id)
						.doc("The frame in which laser scans will be returned. This frame should be at the optical center of the laser, with the x-axis along the zero degree ray, and the y-axis along the 90 degree ray."); // (string, default: laser)

	this->addProperty("time_offset",hokuyoDriver.time_offset)
						.doc("An offset to add to the timestamp before publication of a scan Range: -0.25 to 0.25."); // (double, default: 0.0)

	this->addProperty("allow_unsafe_settings", hokuyoDriver.allow_unsafe_settings)
						.doc("Turn this on if you wish to use the UTM-30LX with an unsafe angular range. Turning this option on may cause occasional crashes or bad data. This option is a tempory workaround that will hopefully be removed in an upcoming driver version."); // (bool, default: False)

	// Read-only parameters

	this->addProperty("min_ang_limit", hokuyoDriver.max_ang_limit)
						.doc("Read only parameter indicating the smallest allowed value for ~min_ang on the currently connected device.");

	this->addProperty("max_ang_limit", hokuyoDriver.max_ang_limit)
						.doc("Read only parameter indicating the largest allowed value for ~max_ang on the currently connected device.");

	this->addProperty("min_range", hokuyoDriver.min_range)
						.doc("Read only parameter indicating the smallest distance that can be measured by the currently connected device.");

	this->addProperty("max_range", hokuyoDriver.max_range)
			.doc("Read only parameter indicating the largest distance that can be measured by the currently connected device.");


//	hokuyoDriver.min_ang = -1.3; //- M_PI / 2;
//	hokuyoDriver.max_ang = 1.3; // M_PI / 2;
//	hokuyoDriver.intensity = false;
//	hokuyoDriver.cluster = 1;
//	hokuyoDriver.skip = 0;
//	hokuyoDriver.port = "/dev/ttyACM0";
//	hokuyoDriver.calibrate_time = true;
//	hokuyoDriver.frame_id = "/laser";
//	hokuyoDriver.time_offset = 0.0;
//	hokuyoDriver.allow_unsafe_settings = false;
//
//	hokuyoDriver.min_ang_limit = -M_PI/2;
//	hokuyoDriver.max_ang_limit = M_PI/2;
//	hokuyoDriver.min_range = -0;
//	hokuyoDriver.max_range = 100000000;


}

bool HokuyoDriverComponent::startHook(){

	hokuyoDriver.configure();

	return true;

}

bool HokuyoDriverComponent::configureHook(){


	hokuyoDriver.doOpen();

	return true;

}

void HokuyoDriverComponent::stopHook(){

	hokuyoDriver.stop();
	hokuyoDriver.doClose();

}

void HokuyoDriverComponent::cleanupHook(){
	//marshalling->writeProperties("properties/HokuyoDriver.cpf");
}

void HokuyoDriverComponent::updateHook() {

	hokuyo::LaserScan scan;

	bool status = hokuyoDriver.performSingleScan(scan);

	if(!status){
		return;
	}

	sensor_msgs::LaserScan scan_msg_;

	scan_msg_.angle_min = scan.config.min_angle;
	scan_msg_.angle_max = scan.config.max_angle;
	scan_msg_.angle_increment = scan.config.ang_increment;
	scan_msg_.time_increment = scan.config.time_increment;
	scan_msg_.scan_time = scan.config.scan_time;
	scan_msg_.range_min = scan.config.min_range;
	scan_msg_.range_max = scan.config.max_range;
	scan_msg_.ranges = scan.ranges;
	scan_msg_.intensities = scan.intensities;
	scan_msg_.header.stamp = ros::Time().fromNSec((uint64_t)scan.system_time_stamp) + ros::Duration(hokuyoDriver.time_offset);
	scan_msg_.header.frame_id = hokuyoDriver.frame_id;



//	double angle = scan_msg_.angle_min;
//
//	log(Info) << "----------------------------------\n timeStamp:" << scan_msg_.header.stamp << "angleMin: " << scan_msg_.angle_min
//			<< "\n angleMax: " << scan_msg_.angle_max << "\n";
//	for(unsigned int i = 0 ; i< scan_msg_.ranges.size();i++){
//		if(i%10==0){
//			log(Info) << " Measure " << i << " - Angle: " << angle << ", Range: " << scan_msg_.ranges.at(i) << "\n";
//		}
//		angle += scan_msg_.angle_increment;
//	}
//	log(Info) << "----------------------------------" << endlog();


//  desired_freq_ = (1. / scan.config.scan_time);
//	if(!use_rep_117_){ // Filter out all NaNs, -Infs, and +Infs; replace them with 0 since that is more consistent with the previous behavior
//		for(uint i = 0; i < scan_msg_.ranges.size(); i++){
//			if(std::isnan(scan_msg_.ranges[i]) || std::isinf(scan_msg_.ranges[i])){
//				scan_msg_.ranges[i] = 0;
//			}
//		}
//	}

	laserScanOutPort.write(scan_msg_);

}


}

ORO_CREATE_COMPONENT( Hokuyo::HokuyoDriverComponent );
