/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008-2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef HOKUYO_DRIVER_HPP
#define HOKUYO_DRIVER_HPP

#include <assert.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <iomanip>

//#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"

//#include "hokuyo_node/HokuyoConfig.h"

#include "HokuyoDriver/hokuyo.h"



using namespace std;

namespace Hokuyo{

class HokuyoDriver
{
//	friend class HokuyoNode;


public:

	typedef char state_t;

	static const state_t CLOSED = 0; // Not connected to the hardware.
	static const state_t OPENED = 1; // Connected to the hardware, ready to start streaming.
	static const state_t RUNNING = 2; // Streaming data.

	/***********************************
	 * Parameters
	 ***********************************/

	/**
	 * The angle of the first range measurement in radians (range is [-π,π], though most devices have a smaller feasible range).
	 */
	double min_ang; // (double, default: -π/2)

	/**
	 * The angle of the last range measurement in radians (range is [-π,π], though most devices have a smaller feasible range).
	 */
	double max_ang; // (double, default: π/2)

	/**
	 * Whether or not the hokuyo returns intensity values.
	 */
	bool intensity; // (bool, default: false)

	/**
	 * The number of adjacent range measurements to cluster into a single reading; the shortest reading from the cluster is reported.
	 */
	int cluster; // (int, default: 1)

	/**
	 * The number of scans to skip between each measured scan. This controls the update rate. For a UTM-30LX, the hokuyo will scan at 40Hz, so setting "skip" to 1 makes it publish at 20Hz.
	 */
	int skip; // (int, default: 0)

	/**
	 * The port where the hokuyo device can be found.
	 */
	string port; // (string, default: /dev/ttyACM0)

	/**
	 * Whether the node should calibrate the hokuyo's time offset on startup. If true, the node will exchange of series of messages with the device in order to determine the time delay in the USB connection. This calibration step is necessary to produce accurate time stamps on scans.
	 */
	bool calibrate_time; // (bool, default: true)

	/**
	 * The frame in which laser scans will be returned. This frame should be at the optical center of the laser, with the x-axis along the zero degree ray, and the y-axis along the 90 degree ray.
	 */
	string frame_id; // (string, default: laser)

	/**
	 * An offet to add to the timestamp before publication of a scan Range: -0.25 to 0.25 (New in release 1.0.1)
	 */
	double time_offset; // (double, default: 0.0)

	/**
	 * Turn this on if you wish to use the UTM-30LX with an unsafe angular range. Turning this option on may cause occasional crashes or bad data. This option is a tempory workaround that will hopefully be removed in an upcoming driver version. (New in release 1.0.3)
	 */
	bool allow_unsafe_settings; // (bool, default: False)

	// Read-only parameters

	/**
	 * Read only parameter indicating the smallest allowed value for ~min_ang on the currently connected device.
	 */
	double min_ang_limit;

	/**
	 * Read only parameter indicating the largest allowed value for ~max_ang on the currently connected device.
	 */
	double max_ang_limit;

	/**
	 * Read only parameter indicating the smallest distance that can be measured by the currently connected device.
	 */
	double min_range;

	/**
	 * Read only parameter indicating the largest distance that can be measured by the currently connected device.
	 */
	double max_range;

	//typedef boost::function<void(const hokuyo::LaserScan &)> UseScanFunction;
	//UseScanFunction useScan_;

private:

//	boost::shared_ptr<boost::thread> scan_thread_;

	std::string device_status_;
	std::string device_id_;
	std::string last_seen_device_id_;

	bool first_scan_;

	std::string vendor_name_;
	std::string product_name_;
	std::string protocol_version_;
	std::string firmware_version_;

	std::string connect_fail_;

	hokuyo::LaserScan  scan_;
	hokuyo::Laser laser_;
	hokuyo::LaserConfig laser_config_;

	bool calibrated_;
	int lost_scan_thread_count_;
	int corrupted_scan_count_;

	state_t state_;

public:
	//hokuyo_node::HokuyoConfig config_;
	//typedef hokuyo_node::HokuyoConfig Config;

	HokuyoDriver();

	bool checkAngleRange();
	bool checkIntensitySupport();

	void doOpen();
	void doClose();
	void configure();
	void start();
	void stop();
	std::string getID();
	bool performSingleScan(hokuyo::LaserScan& scan_);
	std::string getStatus();
};
}

#endif
