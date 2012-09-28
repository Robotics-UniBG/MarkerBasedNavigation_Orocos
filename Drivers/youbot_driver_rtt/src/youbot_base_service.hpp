/*
 * KUKA Youbot component for Orocos RTT
 *
 * (C) 2011 Markus Klotzbuecher <markus.klotzbuecher@mech.kuleuven.be>
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

#ifndef __YOUBOT_BASE_SERVICE_HPP__
#define __YOUBOT_BASE_SERVICE_HPP__
#include <rtt/Service.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/TimeService.hpp>

#include <geometry_msgs/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>

#include "youbot_types.hpp"

using namespace std;
using namespace RTT;

namespace youbot_driver {

    namespace YoubotBase {
	bool check_slaves(unsigned int slave_nr);
    };

    class YoubotBaseService : public Service {

    public:
	YoubotBaseService(const string& name, TaskContext* parent, unsigned int slave_nr);

    private:
	//Ports
	InputPort<geometry_msgs::Twist> port_cmd_twist;
	OutputPort<nav_msgs::Odometry> port_odom;
	OutputPort<youbot_msgs::motor_states> port_motor_states;
	OutputPort<std::string> port_control_mode;
	OutputPort<std::string> events;

	// Slaves corresponding with the wheels
	ec_slavet m_wheels[YOUBOT_NR_OF_WHEELS];
	int slave_nrs[YOUBOT_NR_OF_WHEELS];
	in_motor_t*  m_in_motor[YOUBOT_NR_OF_WHEELS];
	std::string m_events; // presized string for composing events.

	ControlMode m_control_mode;
	unsigned int module_init_status; // status of module initialization

	//Motor status variables
	youbot_msgs::motor_states m_motor_states;
	bool m_i2t_ex[YOUBOT_NR_OF_WHEELS];
	float m_wheel_velocities[YOUBOT_NR_OF_WHEELS];

	//Odometry variables
	nav_msgs::Odometry m_odom;
	int32 m_last_wheel_pos[YOUBOT_NR_OF_WHEELS];
	float m_delta_pos[YOUBOT_NR_OF_WHEELS];
	float m_odom_yaw;
	int m_odom_started;
	os::TimeService::nsecs m_last_cmd_twist;
	os::TimeService::nsecs m_timeout_nsec;

	void cartesianToWheelVelocities(geometry_msgs::Twist twist);

	void calculateOdometry();
	void copyMotorStates();
	bool setControlMode(ControlMode mode);
	void resetWheelVel();
	bool check_status_flag();
	bool configure();
	bool setDefParam(uint8 nr, int32 desval);
	bool setDefParams();
	bool start();

	void check_wd(bool);
	bool configured();
	void update();


    }; //class

} //namespace

#endif
