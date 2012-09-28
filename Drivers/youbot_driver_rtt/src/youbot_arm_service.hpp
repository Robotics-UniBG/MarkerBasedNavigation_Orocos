/*
 * KUKA Youbot component for Orocos RTT
 *
 * (C) 2011 Markus Klotzbuecher <markus.klotzbuecher@mech.kuleuven.be>
 *     2010 Ruben Smits <ruben.smits@mech.kuleuven.be>
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

#ifndef __YOUBOT_ARM_SERVICE_HPP__
#define __YOUBOT_ARM_SERVICE_HPP__

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

#include "youbot_types.hpp"

using namespace std;
using namespace RTT;

namespace youbot_driver{

    namespace YoubotArm{
	bool check_slaves(unsigned int slave_nr);
    };

    class YoubotArmService : public Service{

    public:
	YoubotArmService(const string& name, TaskContext* parent, unsigned int slave_nr);
    private:
	RTT::OutputPort<youbot_msgs::motor_states> port_motor_states;
	RTT::OutputPort<sensor_msgs::JointState> port_joint_state;
	RTT::OutputPort<std::string> port_control_mode;
	RTT::OutputPort<std::string> port_events;
	RTT::InputPort<motion_control_msgs::JointVelocities> port_cmd_vel;
	RTT::InputPort<motion_control_msgs::JointPositions> port_cmd_pos;
	RTT::InputPort<motion_control_msgs::JointEfforts> port_cmd_eff;

	bool __setControlMode(ControlMode mode);
	bool setControlMode(ControlMode mode);
	bool start();
	void update();
	bool configure();
	bool configured();
	bool check_status();
	void check_joint_limits();
	void copyJointStates();
	void copyMotorStates();
	void process_data_handle();
	bool setDefParam(uint8 nr, int32 desval);
	bool setDefParams();

	std::string m_events; // presized string for composing events.
	ec_slavet m_joints[YOUBOT_NR_OF_JOINTS];
	int slave_nrs[YOUBOT_NR_OF_JOINTS];
	in_motor_t*  m_in_motor[YOUBOT_NR_OF_JOINTS];
	bool m_i2t_ex[YOUBOT_NR_OF_JOINTS];
	bool m_soft_limit_ex[YOUBOT_NR_OF_JOINTS]; // true if out of soft limit range
	unsigned int max_current[YOUBOT_NR_OF_JOINTS]; // cut off currents ourself

	unsigned int module_init_status; // status of module initialization

	motion_control_msgs::JointVelocities m_cmd_vel;
	motion_control_msgs::JointPositions  m_cmd_pos;
	motion_control_msgs::JointEfforts  m_cmd_eff;
	sensor_msgs::JointState m_joint_state;
	youbot_msgs::motor_states m_motor_states;
	bool m_configured;
	ControlMode m_control_mode;
	ControlMode m_control_modes[YOUBOT_NR_OF_JOINTS];
    };
}//namespace
#endif // __YOUBOT_ARM_SERVICE_HPP__
