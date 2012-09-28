/*
 * KUKA Youbot component for Orocos RTT
 *
 * (C) 2012 Markus Klotzbuecher <markus.klotzbuecher@mech.kuleuven.be>
 *     2010 Ruben Smits <ruben.smits@mech.kuleuven.be>
 *     2010 Steven Bellens <steven.bellens@mech.kuleuven.be>
 *
 *            Department of Mechanical Engineering,
 *           Katholieke Universiteit Leuven, Belgium.
 *
 *  You may redistribute this software and/or modify it under either the
 *  terms of the GNU Lesser General Public License version 2.1 (LGPLv2.1
 *  <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) or (at you
 *  discretion) of the Modified BSD License:
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *  3. The name of the author may not be used to endorse or promote
 *  products derived from this software without specific prior written
 *  permission.
 *  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIREC
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 *  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISIN
 *  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef YOUBOT_DRIVER_COMPONENT_COMPLETE_HPP
#define YOUBOT_DRIVER_COMPONENT_COMPLETE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>
#include <rtt/Port.hpp>
#include <rtt/Operation.hpp>
#include <rtt/os/Timer.hpp>
#include <rtt/os/TimeService.hpp>

#include "youbot_helpers.hpp"
#include "youbot_types.hpp"

#include <youbot_msgs/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <nav_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>
#include <std_msgs/typekit/Types.hpp>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

using namespace RTT;

namespace YoubotDriver {

const std::string RESET_ODOMETRY_EVENT = "RESET_ODOMETRY";

class YoubotDriver: public RTT::TaskContext {
public:
	YoubotDriver(const std::string& name);
	~YoubotDriver();

protected:
	virtual bool configureHook();
	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();

	// eth0, eth1, ethcat
	std::string prop_ifname;
	char m_IOmap[4096];
	youbot_msgs::driver_state drv_state;
	bool armPresent;

	bool __sendMBX(bool gripper, uint8 instr_nr,uint8 param_nr, uint8 slave_nr, uint8 bank_nr, int32& value);
	bool sendMBX(bool gripper, uint8 instr_nr,uint8 param_nr, uint8 slave_nr, int32& value);
	bool sendMBXGripper(uint8 instr_nr,uint8 param_nr, uint8 slave_nr, uint8 bank_nr, int32& value);

	std::vector<RTT::OperationCaller<bool(void)> > configure_ops;
	std::vector<RTT::OperationCaller<bool(void)> > start_ops;
	std::vector<RTT::OperationCaller<void(void)> > update_ops;

	RTT::OutputPort<youbot_msgs::driver_state> drv_state_port;
	RTT::OutputPort<std::string> events_out_port;




	/******************************
	 * BASE PORTS
	 ******************************/

	InputPort<geometry_msgs::Twist> baseTwistInPort;
	InputPort<std_msgs::String> baseConsumedEventsInPort;
	OutputPort<nav_msgs::Odometry> baseOdometryOutPort;
	OutputPort<youbot_msgs::motor_states> baseMotorStatesOutPort;
	OutputPort<std_msgs::String> baseControlModeOutPort;
	OutputPort<std_msgs::String> baseProducedEventsOutPort;
	OutputPort<sensor_msgs::JointState> baseJointsStatesOutPort;

	/******************************
	 * BASE VARIABLES
	 ******************************/

	// Slaves corresponding with the wheels
	ec_slavet base_m_wheels[YOUBOT_NR_OF_WHEELS];
	int base_slave_nrs[YOUBOT_NR_OF_WHEELS];
	in_motor_t*  base_m_in_motor[YOUBOT_NR_OF_WHEELS];
	std::string base_m_events; // presized string for composing events.


	ControlMode base_m_control_mode;
	unsigned int base_module_init_status; // status of module initialization

	//Motor status variables
	youbot_msgs::motor_states base_m_motor_states;
	bool base_m_i2t_ex[YOUBOT_NR_OF_WHEELS];
	float base_m_wheel_velocities[YOUBOT_NR_OF_WHEELS];

	//Odometry variables
	nav_msgs::Odometry base_m_odom;
	sensor_msgs::JointState base_m_joints_states;
	int32 base_m_last_wheel_pos[YOUBOT_NR_OF_WHEELS];
	float base_m_delta_pos[YOUBOT_NR_OF_WHEELS];
	float base_m_odom_yaw;
	int base_m_odom_started;
	os::TimeService::nsecs base_m_last_cmd_twist;
	os::TimeService::nsecs base_m_timeout_nsec;

	// Used for rViz visualization

	tf::TransformBroadcaster transformBroadcaster;

	std::string youBotOdometryFrameId;
	std::string youBotOdometryChildFrameId;

	/******************************
	 * BASE METHODS
	 ******************************/

	void cartesianToWheelVelocities(geometry_msgs::Twist twist);

	void calculateOdometry();
	void resetOdometry();
	void readJointsStates();
	void copyMotorStates();
	bool setBaseControlMode(ControlMode mode);
	void resetWheelVel();
	bool check_status_flag();
	bool setDefParam(uint8 nr, int32 desval);
	bool setDefParams();
	void check_wd(bool);
	bool configured();

	bool baseCheckSlaves(unsigned int slave_nr);

	void baseAddPortsAndOperations();
	void baseInitVariables(unsigned int slave_nr);
	bool baseConfigure();
	bool baseStart();
	void baseUpdate();

//	/******************************
//	 * ARM PORTS
//	 ******************************/
//
//	RTT::OutputPort<youbot_msgs::motor_states> arm_motor_states;
//	RTT::OutputPort<sensor_msgs::JointState> arm_joint_state;
//	RTT::OutputPort<std::string> arm_control_mode;
//	RTT::OutputPort<std::string> arm_events;
//	RTT::InputPort<motion_control_msgs::JointVelocities> arm_cmd_vel;
//	RTT::InputPort<motion_control_msgs::JointPositions> arm_cmd_pos;
//	RTT::InputPort<motion_control_msgs::JointEfforts> arm_cmd_eff;
//
//	/******************************
//	 * ARM VARIABLES
//	 ******************************/
//
//	ec_slavet arm_m_joints[YOUBOT_NR_OF_JOINTS];
//	int arm_slave_nrs[YOUBOT_NR_OF_JOINTS];
//	in_motor_t*  arm_m_in_motor[YOUBOT_NR_OF_JOINTS];
//	bool arm_m_i2t_ex[YOUBOT_NR_OF_JOINTS];
//	bool arm_m_soft_limit_ex[YOUBOT_NR_OF_JOINTS]; // true if out of soft limit range
//	unsigned int arm_max_current[YOUBOT_NR_OF_JOINTS]; // cut off currents ourself
//
//	unsigned int arm_module_init_status; // status of module initialization
//
//	std::string arm_m_events; // presized string for composing events.
//
//	motion_control_msgs::JointVelocities arm_m_cmd_vel;
//	motion_control_msgs::JointPositions  arm_m_cmd_pos;
//	motion_control_msgs::JointEfforts  arm_m_cmd_eff;
//	sensor_msgs::JointState arm_m_joint_state;
//	youbot_msgs::motor_states arm_m_motor_states;
//	bool  arm_m_configured;
//	ControlMode arm_m_control_mode;
//	ControlMode arm_m_control_modes[YOUBOT_NR_OF_JOINTS];
//
//	/******************************
//	 * ARM METHODS
//	 ******************************/
//
//	//	bool __setControlMode(ControlMode mode);
//	//	bool setControlMode(ControlMode mode);
//	//	bool configured();
//	//	bool check_status();
//	//	void check_joint_limits();
//	//	void copyJointStates();
//	//	void copyMotorStates();
//	//	void process_data_handle();
//	//	bool setDefParam(uint8 nr, int32 desval);
//	//	bool setDefParams();
//
	bool armCheckSlaves(unsigned int slave_nr);

	bool armConfigure();
	bool armStart();
	void armUpdate();



	template<class T>
	inline std::string to_string(const T& t, std::ios_base & (*f) (std::ios_base&))
	{
		std::stringstream ss;
		ss << f << t;
		return ss.str();
	}

};//class

}//namespace

#endif
