/********************************************************************************
 *
 * TrajectoryFollower
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
 * File: TrajectoryFollower.hpp
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

#ifndef BASETPOSITIONCONTROOLER_HPP
#define BASETPOSITIONCONTROOLER_HPP

#include <rtt/os/main.h>
#include <rtt/Logger.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Service.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>
#include <rtt/marsh/Marshalling.hpp>

#include <math.h>

#include <angles/angles.h>
#include <tf/tf.h>

#include <geometry_msgs/typekit/Types.hpp>
#include <nav_msgs/typekit/Types.hpp>
#include <brics_rn_msgs/typekit/Types.hpp>
#include <std_msgs/typekit/Types.hpp>

#include <vector>

#include "TrajectoryFollower/OmnidriveController.hpp"

#include "mbn_common/Events.hpp"


using namespace std;
using namespace RTT;


namespace Navigation{

const string MOTION_DONE_EVENT = "MOTION_DONE";
const string MOTION_ERROR_EVENT  = "MOTION_ERROR";

class TrajectoryFollower : public TaskContext{

public:

	TrajectoryFollower(string const& name);

private:

	/*******************************
	 * Component ports
	 *******************************/

	InputPort<nav_msgs::Odometry> odometryInPort;
	InputPort<brics_rn_msgs::Trajectory> targetTrajectoryInPort;

	OutputPort<geometry_msgs::Twist> twistOutPort;
	OutputPort<std_msgs::String> motionStatusOutPort;

	/******************************
	 * Trajecotry Generation variables
	 ******************************/

	/**
	 * Last odometry information read on the input port
	 */
	nav_msgs::Odometry lastOdometry;

	/**
	 * Last position of the robot taken from the odometry
	 */
	geometry_msgs::Pose2D lastOdometryPose;

	/**
	 * Last velocities and accelerations (linear and angular)
	 * of the robot taken from the odometry
	 */
	brics_rn_msgs::Velocity lastOdometryVel;

	/**
	 * Next target position
	 */
	geometry_msgs::Pose nextTargetPose;

	/**
	 * Desired velocities and accelerations (linear and angular)
	 * in the next target position
	 */
	brics_rn_msgs::Velocity nextTargetVel;

	/**
	 * Desired robot trajectory
	 */
	brics_rn_msgs::Trajectory trajectoryTarget;


	/**
	 * Index of the current target in the trajectory
	 */
	unsigned int currentTrajectoryIndex;

	/**
	 * The segment controller
	 */
	OmnidriveController* segmentController;

	/**
	 * True when a trajectory has been computed. This means that we run the control.
	 */
	bool isFollowing;

	/**
	 * True when the target position is set
	 */
	bool trajectorySet;

	/**
	 * True if the odometry has been set in the current cycle
	 */
	bool odometryUpdated;

	/**
	 * Minimun velocity variation that will produce a new trajectory generation
	 */
	double minDeltaVel;

	/******************************
	 * Trajecotry Control variables
	 ******************************/

	/**
	 * The min linear velocity that can be imposed to the robot
	 */
	float minLinearVelocity;

	/**
	 * The max linear velocity that can be imposed to the robot
	 */
	float maxLinearVelocity;

	/**
	 * The min angular velocity that can be imposed to the robot
	 */
	float minAngularVelocity;

	/**
	 * The max angular velocity that can be imposed to the robot
	 */
	float maxAngularVelocity;
	/**
	 * The min linear acceleration that can be imposed to the robot
	 */
	float minLinearAcceleration;

	/**
	 * The max linear acceleration that can be imposed to the robot
	 */
	float maxLinearAcceleration;

	/**
	 * The max angular acceleration that can be imposed to the robot
	 */
	float minAngularAcceleration;

	/**
	 * The max angular acceleration that can be imposed to the robot
	 */
	float maxAngularAcceleration;

	/**
	 * The min distance between the current and the target pose
	 * When the distance is lower than this value the robot is in a satisfactory <x,y>
	 */
	double goalDistanceThreeshold;

	/**
	 * The min difference of orientation between the current and the target pose
	 * When the difference is lower that this value the robot is in a satisfactory theta
	 */
	double goalOrientationThreeshold;

	/**
	 * The last imposed Twist.
	 * It is the result of computed twist + feedback controller compensation
	 */
	geometry_msgs::Twist imposedTwist;

	boost::shared_ptr<Marshalling> marshalling;

	bool startHook();
	bool configureHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	/*
	 * Read the input ports and decide whether a new trajectory has to be computed or not
	 */
	void readInputPorts();

	/**
	 * \param expectedPose the expected position. We compute the position error with
	 * respect to this position.
	 *
	 * Control whether the last odometry position error with respect to the expectPose parameter
	 * is greater than an orientation and euclidean thresholds
	 */
	bool checkOdometry(geometry_msgs::Pose expectedPose);

	void segmentControlHook();

	double computeEuclideanDistance(geometry_msgs::Point target, geometry_msgs::Point odometry);

};

}

#endif
