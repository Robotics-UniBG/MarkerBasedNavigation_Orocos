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

#include <rtt/TaskContext.hpp>

extern "C" {
#include "ethercattype.h"
#include "ethercatmain.h"
}

#include "youbot_helpers.hpp"
#include "youbot_base_service.hpp"
#include <tf/tf.h>

using namespace RTT;

namespace youbot_driver{

    namespace YoubotBase{

	// This function is called by the main driver. The function
	// must check wether the given slaves correspond indeed a
	// youbot base.
	bool check_slaves(unsigned int slave_nr) {
	    if (strcmp(ec_slave[slave_nr].name, YOUBOT_WHEELPOWERBOARD_SLAVENAME) == 0){
		log(Info) << "Found base power board " << YOUBOT_WHEELPOWERBOARD_SLAVENAME << endlog();
		slave_nr++;
	    } else {
		log(Error) << "Base power board: "<< YOUBOT_WHEELPOWERBOARD_SLAVENAME
			   <<" not detected, found " << ec_slave[slave_nr].name << "instead."<<endlog();
		return false;
	    }

	    // The next 4 ones are wheels
	    for(unsigned int i=slave_nr; i < YOUBOT_NR_OF_WHEELS;i++) {
		if (strcmp(ec_slave[i].name, YOUBOT_WHEELCONTROLLER_SLAVENAME) == 0) {
		    log(Info) << "Found wheel slave " << ec_slave[i].name << endlog();
		} else {
		    log(Error) << "Wheel slave "<< i << ": "<< YOUBOT_WHEELCONTROLLER_SLAVENAME
			       <<" not detected, found " << ec_slave[i].name << "instead." << endlog();
		    return false;
		}
	    }
	    return true;
	};
    }

    // Constructor
    YoubotBaseService::YoubotBaseService(const string& name, TaskContext* parent, unsigned int slave_nr)
	: Service(name,parent)
    {
	this->addPort("cmd_twist",port_cmd_twist)
	    .doc("Commanded twist, expressed in the middle of the base");

	this->addPort("odometry",port_odom)
	    .doc("Odometry, from wheel positions, expr. in middle of the base WRT to starting point");

	this->addOperation("setControlMode",&YoubotBaseService::setControlMode,this)
	    .doc("Sets control mode to the desired one")
	    .arg("mode","Desired control mode");

	this->addPort("motor_states", port_motor_states).doc("Current state of motors");
	this->addPort("control_mode", port_control_mode).doc("Currently active control_mode");
	this->addPort("events", events).doc("Event outport");

	this->addOperation("start", &YoubotBaseService::start, this);
	this->addOperation("update", &YoubotBaseService::update, this);
	this->addOperation("configure", &YoubotBaseService::configure, this);
	this->addOperation("configured", &YoubotBaseService::configured, this);

	for(unsigned int i=0;i<YOUBOT_NR_OF_WHEELS;i++){
	    m_wheels[i] = ec_slave[++slave_nr];
	    slave_nrs[i] = slave_nr;
	    m_i2t_ex[i] = false;
	}

	m_timeout_nsec = base_timeout * 1000*1000*1000; // sec -> nsec
	m_motor_states.motor.resize(YOUBOT_NR_OF_WHEELS);
	m_events.reserve(max_event_strlen);
	module_init_status=0;

	// odometry pose estimates frame
	m_odom.header.frame_id = "odom";
	// odometry twist estimates frame
	m_odom.child_frame_id = "base_link";
	// odometry estimates - set to zero
	m_odom.pose.pose.position.x = 0;
	m_odom.pose.pose.position.y = 0;
	m_odom.pose.pose.position.z = 0;
	m_odom.pose.pose.orientation.x = 0;
	m_odom.pose.pose.orientation.y = 0;
	m_odom.pose.pose.orientation.z = 0;
	m_odom.pose.pose.orientation.w = 0;
	m_odom.twist.twist.linear.x = 0;
	m_odom.twist.twist.linear.y = 0;
	m_odom.twist.twist.linear.z = 0;
	m_odom.twist.twist.angular.x = 0;
	m_odom.twist.twist.angular.y = 0;
	m_odom.twist.twist.angular.z = 0;
	m_odom_yaw = 0;
	m_odom_started = 0;

	m_control_mode = MotorStop; //Initialize;
    }

    bool YoubotBaseService::configured() {
	return module_init_status & ((1 << YOUBOT_NR_OF_BASE_SLAVES)-1);
    }

    bool YoubotBaseService::configure() {
	log(Info) << "Configuring base." << endlog();
	setDefParams();
	return true;
    }

    // Calculate the wheel velocities given the desired twist
    void YoubotBaseService::cartesianToWheelVelocities(geometry_msgs::Twist twist) {
	m_wheel_velocities[0] = ( -twist.linear.x + twist.linear.y + twist.angular.z *
				  YOUBOT_ANGULAR_TO_WHEEL_VELOCITY ) * YOUBOT_CARTESIAN_VELOCITY_TO_RPM;

	m_wheel_velocities[1] = ( twist.linear.x + twist.linear.y + twist.angular.z *
				  YOUBOT_ANGULAR_TO_WHEEL_VELOCITY ) * YOUBOT_CARTESIAN_VELOCITY_TO_RPM;

	m_wheel_velocities[2] = ( -twist.linear.x - twist.linear.y + twist.angular.z  *
				  YOUBOT_ANGULAR_TO_WHEEL_VELOCITY ) * YOUBOT_CARTESIAN_VELOCITY_TO_RPM;

	m_wheel_velocities[3] = ( twist.linear.x - twist.linear.y + twist.angular.z *
				  YOUBOT_ANGULAR_TO_WHEEL_VELOCITY ) * YOUBOT_CARTESIAN_VELOCITY_TO_RPM;
    }

    // Calculate odometry information: estimated velocity and travelled distance
    void YoubotBaseService::calculateOdometry() {
	// wheel velocities to cartesian velocities
	m_odom.twist.twist.linear.x =
	    (float) ( m_motor_states.motor[0].velocity - m_motor_states.motor[1].velocity
		      + m_motor_states.motor[2].velocity - m_motor_states.motor[3].velocity )
	    / (float) ( YOUBOT_NR_OF_WHEELS *  YOUBOT_CARTESIAN_VELOCITY_TO_RPM );

	m_odom.twist.twist.linear.y =
	    (float) ( - m_motor_states.motor[0].velocity - m_motor_states.motor[1].velocity
		      + m_motor_states.motor[2].velocity + m_motor_states.motor[3].velocity )
	    / (float) ( YOUBOT_NR_OF_WHEELS * YOUBOT_CARTESIAN_VELOCITY_TO_RPM );

	m_odom.twist.twist.angular.z =
	    (float) ( - m_motor_states.motor[0].velocity - m_motor_states.motor[1].velocity
		      - m_motor_states.motor[2].velocity - m_motor_states.motor[3].velocity )
	    / (float) ( YOUBOT_NR_OF_WHEELS * YOUBOT_CARTESIAN_VELOCITY_TO_RPM * YOUBOT_ANGULAR_TO_WHEEL_VELOCITY);

	// wheel positions to cartesian positions
	// ugly hack: skip the first few samples - to make sure we start at 0 pose
	if(m_odom_started < 10) {
	    for(size_t i = 0; i < YOUBOT_NR_OF_WHEELS; i++){
		m_last_wheel_pos[i] = m_motor_states.motor[i].position;
		m_delta_pos[i] = 0;
		m_odom_started++;
	    }
	} else {
	    for(size_t i = 0; i < YOUBOT_NR_OF_WHEELS; i++) {
		m_delta_pos[i] =
		    (float) ( m_motor_states.motor[i].position - m_last_wheel_pos[i] )
		    * (float) (YOUBOT_WHEEL_CIRCUMFERENCE)
		    / (float) (YOUBOT_TICKS_PER_REVOLUTION * YOUBOT_MOTORTRANSMISSION ) ;
		m_last_wheel_pos[i] = m_motor_states.motor[i].position;
	    }

	    m_odom_yaw += ( m_delta_pos[0] + m_delta_pos[1] + m_delta_pos[2] + m_delta_pos[3] )
		/ ( YOUBOT_NR_OF_WHEELS * YOUBOT_ANGULAR_TO_WHEEL_VELOCITY );

	    m_odom.pose.pose.position.x +=
		- ( ( ( m_delta_pos[0] - m_delta_pos[1] + m_delta_pos[2] - m_delta_pos[3] )
		      / YOUBOT_NR_OF_WHEELS ) * cos( m_odom_yaw )
		    - ( ( - m_delta_pos[0] - m_delta_pos[1] + m_delta_pos[2] + m_delta_pos[3] )
			/ YOUBOT_NR_OF_WHEELS ) * sin( m_odom_yaw ));

	    m_odom.pose.pose.position.y +=
		- ( ( ( m_delta_pos[0] - m_delta_pos[1] + m_delta_pos[2] - m_delta_pos[3] )
		      / YOUBOT_NR_OF_WHEELS ) * sin( m_odom_yaw )
		    + ( ( - m_delta_pos[0] - m_delta_pos[1] + m_delta_pos[2] + m_delta_pos[3] )
			/ YOUBOT_NR_OF_WHEELS ) * cos( m_odom_yaw ));

	    m_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(m_odom_yaw);
	}
    }

    // Copy motor status information into ROS message
    void YoubotBaseService::copyMotorStates()
    {
	for(size_t i=0; i < YOUBOT_NR_OF_WHEELS; i++) {
	    m_motor_states.motor[i].position = m_in_motor[i]->position;
	    m_motor_states.motor[i].current = m_in_motor[i]->current;
	    m_motor_states.motor[i].velocity = m_in_motor[i]->velocity;
	    m_motor_states.motor[i].error_flags = m_in_motor[i]->error_flags;
	    m_motor_states.motor[i].temperature = m_in_motor[i]->temperature;
	}
    }

    bool YoubotBaseService:: check_status_flag()
    {
	bool fatal=false;
	for(size_t i=0; i < YOUBOT_NR_OF_WHEELS; i++) {
	    if (m_in_motor[i]->error_flags & OVERCURRENT) {
		m_motor_states.motor[i].counters.overcurrent++;
		events.write(make_event(m_events, "e_OVERCURRENT,wheelid:", i));
		fatal=true;
	    }
	    if (m_in_motor[i]->error_flags & UNDERVOLTAGE) {
		m_motor_states.motor[i].counters.undervoltage++;
		events.write(make_event(m_events, "e_UNDERVOLTAGE,wheelid:", i));
		m_control_mode=MotorStop;
		fatal=true;
	    }
	    if (m_in_motor[i]->error_flags & OVERTEMP) {
		m_motor_states.motor[i].counters.overtemp++;
		events.write(make_event(m_events, "e_OVERTEMP,wheelid:",i));
		m_control_mode=MotorStop;
		fatal=true;
	    }
	    if (m_in_motor[i]->error_flags & HALL_ERR) {
		m_motor_states.motor[i].counters.hall_err++;
		events.write(make_event(m_events, "e_HALL_ERR,wheelid:", i));
	    }
	    if (m_in_motor[i]->error_flags & ENCODER_ERR) {
		m_motor_states.motor[i].counters.encoder_err++;
		events.write(make_event(m_events, "e_ENCODER_ERR,wheelid:", i));
	    }
	    if (m_in_motor[i]->error_flags & SINE_COMM_INIT_ERR) {
		m_motor_states.motor[i].counters.sine_comm_init_err++;
		events.write(make_event(m_events, "e_SINE_COMM_INIT_ERR,wheelid:", i));
	    }
	    if (m_in_motor[i]->error_flags & EMERGENCY_STOP) {
		m_motor_states.motor[i].counters.emergency_stop++;
		events.write(make_event(m_events, "e_EMERGENCY_STOP,wheelid:", i));
	    }
	    if (m_in_motor[i]->error_flags & MODULE_INIT) {
		if (! (module_init_status & (1 << i))) {
		    log(Debug) << "base: MODULE_INIT set for slave" <<i<< endlog();
		}
		module_init_status |= 1 << i;
	    }
	    if (m_in_motor[i]->error_flags & EC_TIMEOUT) {
		m_motor_states.motor[i].counters.ec_timeout++;
		events.write(make_event(m_events, "e_EC_TIMEOUT,wheelid:", i));
		OperationCaller<bool(bool,uint8,uint8,uint8,int32&)>
		    sendMBX(this->getParent()->getOperation("sendMBX"),
			    this->getOwner()->engine());
		int32 dummy;
		if(!sendMBX(false,SAP,CLR_EC_TIMEOUT,slave_nrs[i],dummy)) {
		    log(Error) << "base: failed to clear EC_TIMEOUT flag" << endlog();
		    fatal = true;
		}
	    }

	    // i2c exceeded rising edge
	    if (m_in_motor[i]->error_flags & I2T_EXCEEDED && !m_i2t_ex[i]) {
		OperationCaller<bool(bool,uint8,uint8,uint8,int32&)>
		    sendMBX(this->getParent()->getOperation("sendMBX"),
			    this->getOwner()->engine());
		int32 dummy;

		m_motor_states.motor[i].counters.i2t_exceeded++;
		events.write(make_event(m_events, "e_I2T_EXCEEDED,wheelid:", i));
		m_control_mode=MotorStop;
		log(Warning) << "base: i2t execeeded on wheel " << i << endlog();
		fatal=true;
		m_i2t_ex[i]=true;

		// reset while set (should probably be done externally)
                if(!sendMBX(false,SAP,CLEAR_I2T,slave_nrs[i],dummy))
                    log(Warning) << "base: failed to clear I2T over flag" << endlog();

	    } else if (!(m_in_motor[i]->error_flags & I2T_EXCEEDED) && m_i2t_ex[i]) {
		// i2c exceeded falling edge
		events.write(make_event(m_events, "e_I2T_EXCEEDED_EXIT,wheelid:", i));
		m_i2t_ex[i]=false;
		log(Warning) << "base: i2t exceeded reset on wheel " << i << endlog();
	    }
	}
	return fatal;
    }

    // function to change the control mode.
    bool YoubotBaseService::setControlMode(ControlMode mode)
    {
	if (!configured()) {
	    log(Warning) << "setControlMode: refusing to set control mode before configured" << endlog();
	    return false;
	}

	if(mode==Velocity)
	    resetWheelVel();

	m_control_mode = mode;
	return true;
    }

    bool YoubotBaseService::setDefParam(uint8 nr, int32 desval)
    {
	int32 retval;
	bool succ = true;
	OperationCaller<bool(bool,uint8,uint8,uint8,int32&)>
	    sendMBX(this->getParent()->getOperation("sendMBX"),
		    this->getOwner()->engine());

	for(unsigned int i=0;i<YOUBOT_NR_OF_WHEELS;i++) {
	    log(Info) << "setDefParam: slave: " << slave_nrs[i] << "   param nr: " <<
		((int ) nr) << "   desval: " << desval << endlog();

	    // read parameter value
	    if(!sendMBX(false, GAP, nr, slave_nrs[i], retval)) {
		log(Error) << "setDefParam: failed to read param nr " << nr << endlog();
		return false;
	    }

	    // param is not set correctly, we need to set it.
	    if (retval != desval) {
		log(Info) << "setDefParam: param nr not yet set. trying to set" << endlog();
		// set param
		if(!sendMBX(false, SAP, nr, slave_nrs[i], desval)) {
		    log(Error) << "setDefParam: setting param nr " << nr << " to " << desval
			       << " failed. pwd?" << endlog();
		    succ = false;
		}

		// re-read and check
		if(!sendMBX(false, GAP, nr, slave_nrs[i], retval)) {
		    log(Error) << "setDefParam: failed to re-read param nr " << nr << endlog();
		    return false;
		}
		if (retval != desval) {
		    log(Info) << "setDefParam: setting failed" << endlog();
		}
	    }
	    /* param set should be set here (or we failed) */
	}
	return succ;
    }

    // setup default params
    bool YoubotBaseService::setDefParams()
    {
	setDefParam(159, 3); // Commutation mode
	setDefParam(167, 0); // Block PWM scheme
	// setDefParam(240, ?); // Block PWM scheme
	setDefParam(249, 1); // Block commutation init using hall sensors
	setDefParam(250, 4000); // nr encoder steps per rotation
	setDefParam(251, 1); // or 0?
	setDefParam(253, 16); // nr motor poles
	setDefParam(254, 1); // invert hall sensor
	return true;
    }

    // check / trigger watchdog
    // if trigger is true then reset watchdog
    void YoubotBaseService::check_wd(bool trigger)
    {
	os::TimeService::nsecs cur_nsec = os::TimeService::Instance()->getNSecs();
	os::TimeService::nsecs diff_nsec = os::TimeService::Instance()->getNSecs(m_last_cmd_twist);

	if(diff_nsec >= m_timeout_nsec) {
	    events.write("e_base_cmd_watchdog");
	    log(Error) << "base: watchdog emergency stop. No base twist in "
		       << base_timeout << "s" << endlog();
	    m_control_mode = MotorStop;
	}

	if(trigger)
	    m_last_cmd_twist = cur_nsec;
    }

    void YoubotBaseService::resetWheelVel()
    {
	for(int i=0; i<YOUBOT_NR_OF_WHEELS; i++)
	    m_wheel_velocities[i] = 0;

	m_last_cmd_twist = os::TimeService::InfiniteNSecs;
    }

    bool YoubotBaseService::start() {
	int32 dummy;

	OperationCaller<bool(bool,uint8,uint8,uint8,int32&)>
	    sendMBX(this->getParent()->getOperation("sendMBX"),
		    this->getOwner()->engine());

	// set wheel velocity to zero and reset EC_TIMEOUT
	resetWheelVel();

	for(int i=0; i<YOUBOT_NR_OF_WHEELS; i++) {
	    if(!sendMBX(false,SAP,CLR_EC_TIMEOUT,slave_nrs[i],dummy)) {
		log(Error) << "base: failed to clear EC_TIMEOUT flag" << endlog();
		return false;
	    }
	}

	log(Info) << "YoubotBaseService: started" << endlog();
	return true;
    }

    void YoubotBaseService::update() {
	geometry_msgs::Twist twist;
	bool fatal_err;
	FlowStatus fs;

	// Process received data
	// mk, todo: why one earth are we repeating this each time???
	for (size_t i = 0; i < YOUBOT_NR_OF_WHEELS; i++)
	    m_in_motor[i] = (in_motor_t*) m_wheels[i].inputs;

	fatal_err = check_status_flag(); // raise events ASAP
	copyMotorStates();
	port_motor_states.write(m_motor_states);

	if (!configured()) {
	    m_control_mode=Initialize;
	} else {
	    calculateOdometry();
	    port_odom.write(m_odom);
	}

	// tbd: only write when changed
	port_control_mode.write(control_mode2str(m_control_mode));

	for (size_t i = 0; i < YOUBOT_NR_OF_WHEELS; i++)
	    ((out_motor_t*) (m_wheels[i].outputs))->controller_mode = m_control_mode;

	switch (m_control_mode) {
	case MotorStop:
	    break;
	case Velocity:
	    fs = port_cmd_twist.read(twist);
	    if ( fs == NewData ) {
		cartesianToWheelVelocities(twist); // this sets up m_wheel_velocities
		check_wd(true);
	    } else if ( m_wheel_velocities[0] == 0 && m_wheel_velocities[1] == 0 &&
			m_wheel_velocities[2] == 0 && m_wheel_velocities[3] == 0 ) {
		check_wd(true); // if velocity is zero don't timeout.
	    } else {
		check_wd(false); // Not NewData and Velocity != 0
	    }

	    for (size_t i = 0; i < YOUBOT_NR_OF_WHEELS; i++)
		((out_motor_t*) (m_wheels[i].outputs))->value = m_wheel_velocities[i];

	    break;
	case Initialize:
	    break;
	default:
	    log(Info) << "base::update: unexpected control mode: " << m_control_mode << endlog();
	}
    }

} // namespace
