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



#include <assert.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <iomanip>

//#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"

//#include "hokuyo_node/HokuyoConfig.h"

#include "HokuyoDriver/HokuyoDriver.hpp"

using namespace std;

namespace Hokuyo{


HokuyoDriver::HokuyoDriver()
{
	calibrated_ = false;
	lost_scan_thread_count_ = 0;
	corrupted_scan_count_ = 0;
}

bool HokuyoDriver::checkAngleRange()
{
	bool changed = false;

	if (min_ang < laser_config_.min_angle)
	{
		changed = true;
		if (laser_config_.min_angle - min_ang > 1e-10)  /// @todo Avoids warning when restarting node pending ros#2353 getting fixed.
		{
			printf("Requested angle (%f rad) out of range, using minimum scan angle supported by device: %f rad.",
					min_ang, laser_config_.min_angle);
		}
		min_ang = laser_config_.min_angle;
	}

	double max_safe_angular_range_per_cluster_deg = 95;
	if (firmware_version_ == "1.16.01(16/Nov./2009)")
		max_safe_angular_range_per_cluster_deg = 190;

	int real_cluster = cluster == 0 ? 1 : cluster;
	double max_safe_angular_range = (real_cluster * max_safe_angular_range_per_cluster_deg) * M_PI / 180;

	if (intensity && (max_ang - min_ang) > max_safe_angular_range + 1e-8 &&
			!allow_unsafe_settings && laser_.getProductName() ==
					"SOKUIKI Sensor TOP-URG UTM-30LX")
	{
		changed = true;
		max_ang = min_ang + max_safe_angular_range;
		printf("More than %f degree/cluster scan range requested on UTM-30LX firmware version %s in intensity mode with cluster=%i. The max_ang was adjusted to limit the range. You may extend the scanner's angular range using the allow_unsafe_settings option, but this may result in incorrect data or laser crashes that will require a power cycle of the laser.", max_safe_angular_range_per_cluster_deg, firmware_version_.c_str(), real_cluster);
	}

	if (max_ang - laser_config_.max_angle > 1e-10)   /// @todo Avoids warning when restarting node pending ros#2353 getting fixed.
	{
		changed = true;
		printf("Requested angle (%f rad) out of range, using maximum scan angle supported by device: %f rad.",
				max_ang, laser_config_.max_angle);
		max_ang = laser_config_.max_angle;
	}

	if (min_ang > max_ang)
	{
		changed = true;
		if (max_ang < laser_config_.min_angle)
		{
			if (laser_config_.min_angle - max_ang > 1e-10)  /// @todo Avoids warning when restarting node pending ros#2353 getting fixed.
				printf("Requested angle (%f rad) out of range, using minimum scan angle supported by device: %f rad.",
						max_ang, laser_config_.min_angle);
			max_ang = laser_config_.min_angle;
		}
		printf("Minimum angle must be greater than maximum angle. Adjusting min_ang.");
		min_ang = max_ang;
	}

	return changed;
}

bool HokuyoDriver::checkIntensitySupport()
{
	if (intensity && !laser_.isIntensitySupported())
	{
		ROS_WARN("This unit does not appear to support intensity mode. Turning intensity off.");
		intensity = false;
		return true;
	}
	return false;
}

void HokuyoDriver::doOpen()
{
	try
	{
		std::string old_device_id = device_id_;
		device_id_ = "unknown";
		device_status_ =  "unknown";
		first_scan_ = true;

		laser_.open(port.c_str());

		device_id_ = getID();
		vendor_name_ = laser_.getVendorName();
		firmware_version_ = laser_.getFirmwareVersion();
		product_name_ = laser_.getProductName();
		protocol_version_ = laser_.getProtocolVersion();

		device_status_ = laser_.getStatus();
		if (device_status_ != std::string("Sensor works well."))
		{
			doClose();
			printf("Laser returned abnormal status message, aborting: %s You may be able to find further information at http://www.ros.org/wiki/hokuyo_node/Troubleshooting/", device_status_.c_str());
			return;
		}

		if (old_device_id != device_id_)
		{
			printf("Connected to device with ID: %s", device_id_.c_str());

			if (last_seen_device_id_ != device_id_)
			{
				// Recalibrate when the device changes.
				last_seen_device_id_ = device_id_;
				calibrated_ = false;
			}

			// Do this elaborate retry circuis if we were just plugged in.
			for (int retries = 10;; retries--)
				try {
					laser_.laserOn();
					break;
				}
			catch (hokuyo::Exception &e)
			{
				if (!retries)
					throw e; // After trying for 10 seconds, give up and throw the exception.
				else if (retries == 10)
					printf("Could not turn on laser. This may happen just after the device is plugged in. Will retry for 10 seconds.");
				ros::Duration(1).sleep();
			}
		}
		else
			laser_.laserOn(); // Otherwise, it should just work, so no tolerance.

		if (calibrate_time && !calibrated_)
		{
			ROS_INFO("Starting calibration. This will take up a few seconds.");
			double latency = laser_.calcLatency(false && intensity, min_ang, max_ang, cluster, skip) * 1e-9;
			calibrated_ = true; // This is a slow step that we only want to do once.
			ROS_INFO("Calibration finished. Latency is: %0.4f", latency);
		}
		else
		{
			calibrated_ = false;
			laser_.clearLatency();
		}

		printf("Device opened successfully.");
		laser_.getConfig(laser_config_);

		state_ = OPENED;
	}
	catch (hokuyo::Exception& e)
	{
		doClose();
		printf("Exception thrown while opening Hokuyo.\n%s", e.what());
		return;
	}
}

void HokuyoDriver::doClose()
{
	try
	{
		laser_.close();
		printf("Device closed successfully.");
	} catch (hokuyo::Exception& e) {
		printf("Exception thrown while trying to close:\n%s",e.what());
	}

	state_ = CLOSED; // If we can't close, we are done for anyways.
}

void HokuyoDriver::configure()
{
	try
	{
		laser_.laserOn();

//		int status = laser_.requestScans(intensity, min_ang, max_ang, cluster, skip);
//
//		if (status != 0) {
//			printf("Failed to request scans from device.  Status: %d.", status);
//			corrupted_scan_count_++;
//			return;
//		}

		printf("Waiting for first scan.");
		start();

		//scan_thread_.reset(new boost::thread(boost::bind(&HokuyoDriver::scanThread, this)));
	}
	catch (hokuyo::Exception& e)
	{
		doClose();
		printf("Exception thrown while starting Hokuyo.\n%s", e.what());
		connect_fail_ = e.what();
		return;
	}
}

void HokuyoDriver::start()
{
	state_ = RUNNING;
}

void HokuyoDriver::stop()
{
	if (state_ != RUNNING) // RUNNING can exit asynchronously.
		return;

	state_ = OPENED;
	laser_.laserOff();

	// Commented because I want to perform a single scan and not a set of scans

	//		if (scan_thread_ && !scan_thread_->timed_join((boost::posix_time::milliseconds) 2000))
	//		{
	//			ROS_ERROR("scan_thread_ did not die after two seconds. Pretending that it did. This is probably a bad sign.");
	//			lost_scan_thread_count_++;
	//		}
	//		scan_thread_.reset();

	printf("Stopped.");
}

std::string HokuyoDriver::getID()
{
	std::string id = laser_.getID();
	if (id == std::string("H0000000"))
		return "unknown";
	return id;
}

bool HokuyoDriver::performSingleScan(hokuyo::LaserScan& scan_)
{

	if(state_ == RUNNING)
	{
		try
		{
			int status = laser_.pollScan(scan_, min_ang, max_ang);
			// int status = laser_.serviceScan(scan_);

			if(status != 0)
			{
				printf("Error getting scan: %d", status);
			}
		} catch (hokuyo::CorruptedDataException &e) {
			printf("Skipping corrupted data");
		} catch (hokuyo::Exception& e) {
			printf("Exception thrown while trying to get scan.\n%s", e.what());
			doClose();
			return false;
		}

		//useScan_(scan_);
		return true;
	}else{
		return false;
	}

	// Commented because now the method perform only one scan
	//		try
	//		{
	//			laser_.stopScanning(); // This actually just calls laser Off internally.
	//		} catch (hokuyo::Exception &e)
	//		{
	//			ROS_WARN("Exception thrown while trying to stop scan.\n%s", e.what());
	//		}
	//		state_ = OPENED;
}

std::string HokuyoDriver::getStatus(){
	return laser_.getStatus();
}


//class HokuyoNode : public driver_base::DriverNode<HokuyoDriver>
//{
//private:
//	string connect_fail_;
//
//	double desired_freq_;
//
//	ros::NodeHandle node_handle_;
//	diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan> scan_pub_;
//	sensor_msgs::LaserScan scan_msg_;
//	diagnostic_updater::FunctionDiagnosticTask hokuyo_diagnostic_task_;
//
//	// Tick-tock transition variable, controls if the driver outputs NaNs and Infs
//	bool use_rep_117_;
//
//public:
//	HokuyoNode(ros::NodeHandle &nh) :
//		driver_base::DriverNode<HokuyoDriver>(nh),
//		node_handle_(nh),
//		scan_pub_(node_handle_.advertise<sensor_msgs::LaserScan>("scan", 100),
//				diagnostic_,
//				diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05),
//				diagnostic_updater::TimeStampStatusParam()),
//				hokuyo_diagnostic_task_("Hokuyo Diagnostics", boost::bind(&HokuyoNode::connectionStatus, this, _1))
//				{
//		desired_freq_ = 0;
//		driver_.useScan_ = boost::bind(&HokuyoNode::publishScan, this, _1);
//		driver_.setPostOpenHook(boost::bind(&HokuyoNode::postOpenHook, this));
//
//		// Check whether or not to support REP 117
//		std::string key;
//		if (node_handle_.searchParam("use_rep_117", key))
//		{
//			node_handle_.getParam(key, use_rep_117_);
//		} else {
//			use_rep_117_ = false;
//		}
//
//		if(!use_rep_117_){ // Warn the user that they need to update their code.
//			ROS_WARN("The use_rep_117 parameter has not been set or is set to false.  Please see: http://ros.org/wiki/rep_117/migration");
//		}
//				}
//
//	void postOpenHook()
//	{
//		private_node_handle_.setParam("min_ang_limit", (double) (driver_.laser_config_.min_angle));
//		private_node_handle_.setParam("max_ang_limit", (double) (driver_.laser_config_.max_angle));
//		private_node_handle_.setParam("min_range", (double) (driver_.laser_config_.min_range));
//		private_node_handle_.setParam("max_range", (double) (driver_.laser_config_.max_range));
//
//		diagnostic_.setHardwareID(driver_.getID());
//
//		if (driver_.checkIntensitySupport(driver_.config_) ||
//				driver_.checkAngleRange(driver_.config_)) // Might have been set before the device's range was known.
//			reconfigure_server_.updateConfig(driver_.config_);
//
//		scan_pub_.clear_window(); // Reduce glitches in the frequency diagnostic.
//	}
//
//
//
//};
}


