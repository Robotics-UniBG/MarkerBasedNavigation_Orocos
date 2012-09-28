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

#include <rtt/Service.hpp>
#include <rtt/TaskContext.hpp>

extern "C" {
#include "ethercattype.h"
#include "ethercatmain.h"
}

#include "youbot_helpers.hpp"
#include "youbot_arm_service.hpp"
#include "youbot_types.hpp"

using namespace RTT;
namespace youbot_driver{

    namespace YoubotArm{

        bool check_slaves(unsigned int slave_nr){
            if (strcmp(ec_slave[slave_nr].name, YOUBOT_ARMPOWERBOARD_SLAVENAME) == 0){
                log(Info) << "Found arm power board " << YOUBOT_ARMPOWERBOARD_SLAVENAME << endlog();
                slave_nr++;
            } else{
                log(Error) << "Arm power board: "<< YOUBOT_ARMPOWERBOARD_SLAVENAME
			   <<" not detected, found " << ec_slave[slave_nr].name 
			   << "instead."<<endlog();
                return false;
            }
            //The arm joints are the next 5 ones
            for(unsigned int i=slave_nr;i < YOUBOT_NR_OF_JOINTS;i++){
                if (strcmp(ec_slave[i].name, YOUBOT_JOINTCONTROLLER_SLAVENAME) == 0) {
                    log(Info) << "Found joint slave " << ec_slave[i].name << endlog();
                }else{
                    log(Error) << "Joint slave "<< i << ": "
			       << YOUBOT_JOINTCONTROLLER_SLAVENAME 
			       <<" not detected, found " 
			       << ec_slave[i].name << "instead."<<endlog();
                    return false;
                }
            }
            log(Info) << "Detected youbot arm." << endlog();
            return true;
        }
    }

    bool YoubotArmService::setDefParam(uint8 nr, int32 desval)
    {
	int32 retval;
	bool succ = true;
	OperationCaller<bool(bool,uint8,uint8,uint8,int32&)> sendMBX(this->getParent()->getOperation("sendMBX"),
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
    bool YoubotArmService::setDefParams()
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

    // constructor
    YoubotArmService::YoubotArmService(const string& name, TaskContext* parent, unsigned int slave_nr)
        : Service(name,parent){

        this->addOperation("setControlMode", &YoubotArmService::setControlMode, this, RTT::OwnThread)
	    .doc("change the control mode of all arm joint motors")
	    .arg("mode","desired control mode: MotorStop,Positioning,Velocity,NoAction,SetPositionToReference");

        this->addPort("motor_states",port_motor_states).doc("Current state of motors");

        this->addPort("jointstate",port_joint_state).doc("Measured joint state");
        this->addPort("joint_velocity_command",port_cmd_vel).doc("Commanded joint velocities");
        this->addPort("joint_position_command",port_cmd_pos).doc("Commanded joint positions");
        this->addPort("joint_effort_command",port_cmd_eff).doc("Commanded joint efforts");
        this->addPort("control_mode", port_control_mode).doc("Currently active control_mode");
        this->addPort("events",port_events).doc("Events");

        this->addOperation("start",&YoubotArmService::start,this);
        this->addOperation("update",&YoubotArmService::update,this);
        this->addOperation("configure",&YoubotArmService::configure,this);
        this->addOperation("configured",&YoubotArmService::configured,this);

        for(unsigned int i=0;i<YOUBOT_NR_OF_JOINTS;i++){
            m_joints[i] = ec_slave[++slave_nr];
            slave_nrs[i] = slave_nr;
            m_i2t_ex[i] = false;
            m_soft_limit_ex[i] = false;
        }

        //Initialize variables
        m_events.reserve(max_event_strlen);
        m_joint_state.name.assign(YOUBOT_NR_OF_JOINTS,"");
        m_joint_state.name[0]="arm_joint_1";
        m_joint_state.name[1]="arm_joint_2";
        m_joint_state.name[2]="arm_joint_3";
        m_joint_state.name[3]="arm_joint_4";
        m_joint_state.name[4]="arm_joint_5";
        m_joint_state.position.assign(YOUBOT_NR_OF_JOINTS,0);
        m_joint_state.velocity.assign(YOUBOT_NR_OF_JOINTS,0);
        m_joint_state.effort.assign(YOUBOT_NR_OF_JOINTS,0);
        m_cmd_vel.velocities.assign(YOUBOT_NR_OF_JOINTS,0);
        m_cmd_pos.positions.assign(YOUBOT_NR_OF_JOINTS,0);
        m_cmd_eff.efforts.assign(YOUBOT_NR_OF_JOINTS,0);

        m_motor_states.motor.resize(YOUBOT_NR_OF_JOINTS);
	module_init_status=0;
    }

    // rename to configure() ASAP!
    bool YoubotArmService::configure(){
        int32 val=0;

	log(Info) << "Calibrating arm service" << endlog();
        OperationCaller<bool(bool,uint8,uint8,uint8,int32&)> sendMBX(this->getParent()->getOperation("sendMBX"),
                                                                     this->getOwner()->engine());
	setDefParams();

        for(unsigned int i=0;i<YOUBOT_NR_OF_JOINTS;i++) {
            if(!sendMBX(false, GAP,MAX_CURRENT,slave_nrs[i],val)) {
                log(Error) << "configure: reading MAX_CURRENT failed." << endlog();
            }
            max_current[i] = val;
            log(Info) << "configure: setting axis " << i << MAX_CURRENT << " to " << val << endlog();
        }
        return true;
    }

    bool YoubotArmService::configured() {
	return module_init_status & ((1 << YOUBOT_NR_OF_ARM_SLAVES)-1);
    }


    bool YoubotArmService::start(){
        int32 dummy;
        OperationCaller<bool(bool,uint8,uint8,uint8,int32&)> sendMBX(this->getParent()->getOperation("sendMBX"),
                                                                     this->getOwner()->engine());

        for(int i=0; i<YOUBOT_NR_OF_JOINTS; i++) {
            if(!sendMBX(false,SAP,CLR_EC_TIMEOUT,slave_nrs[i],dummy)) {
                log(Error) << "arm: failed to clear EC_TIMEOUT flag" << endlog();
                return false;
            }
        }
	log(Info) << "YoubotArmService: started" << endlog();
        return true; //return setControlMode(MotorStop);
    }

    // check the soft joint limits and raise events
    void YoubotArmService::check_joint_limits()
    {
        double jntpos;

        for(unsigned int i=0; i < YOUBOT_NR_OF_JOINTS;i++){
            jntpos = m_joint_state.position[i];
            // first time in limit
            if ( !m_soft_limit_ex[i] &&
                 ( jntpos < (YOUBOT_ARM_LOWER_LIMIT[i] * YOUBOT_ARM_SOFT_LIMIT * (M_PI/180)) ||
                   jntpos > (YOUBOT_ARM_UPPER_LIMIT[i] * YOUBOT_ARM_SOFT_LIMIT * (M_PI/180)))) {
                port_events.write(make_event(m_events, "e_JNT_SOFT_LIMIT_EXEEDED,jointid:", i));
                m_soft_limit_ex[i] = true;
            }
            // first time out of limit
            if ( m_soft_limit_ex[i] &&
                 !( jntpos < (YOUBOT_ARM_LOWER_LIMIT[i] * YOUBOT_ARM_SOFT_LIMIT * (M_PI/180)) ||
                    jntpos > (YOUBOT_ARM_UPPER_LIMIT[i] * YOUBOT_ARM_SOFT_LIMIT * (M_PI/180)))) {
                port_events.write(make_event(m_events, "e_JNT_SOFT_LIMIT_EXEEDED_EXIT,jointid:", i));
                m_soft_limit_ex[i] = false;
            }
        }
    }

    // this is only done when we are in normal operation (ControlMode != Initialize)
    void YoubotArmService::copyJointStates()
    {
	double pos_conversion, vel_conversion, eff_conversion;

	for(unsigned int i=0; i < YOUBOT_NR_OF_JOINTS;i++) {  
	    pos_conversion = 2*M_PI/(YOUBOT_ARM_JOINT_GEAR_RATIOS[i]*YOUBOT_TICKS_PER_REVOLUTION);
	    vel_conversion = 2*M_PI/(60*YOUBOT_ARM_JOINT_GEAR_RATIOS[i]);
	    eff_conversion = YOUBOT_ARM_JOINT_TORQUE_CONSTANTS[i]*YOUBOT_ARM_JOINT_GEAR_RATIOS[i]/1000;

	    m_joint_state.position[i] = (m_in_motor[i]->position) * pos_conversion;
	    m_joint_state.velocity[i] = m_in_motor[i]->velocity * vel_conversion;
	    m_joint_state.effort[i] = m_in_motor[i]->current * eff_conversion; //current is in mA
	    m_joint_state.header.stamp=ros::Time::now();
	}
    }

    // low level status: always sent out.
    void YoubotArmService::copyMotorStates()
    {
	for(unsigned int i=0; i < YOUBOT_NR_OF_JOINTS;i++) {  
	    m_motor_states.motor[i].position = m_in_motor[i]->position;
	    m_motor_states.motor[i].current = m_in_motor[i]->current;
	    m_motor_states.motor[i].velocity = m_in_motor[i]->velocity;
	    m_motor_states.motor[i].error_flags = m_in_motor[i]->error_flags;
	    m_motor_states.motor[i].temperature = m_in_motor[i]->temperature;
	}
    }

    void YoubotArmService::process_data_handle() 
    {
        bool fatal_err;

	// tbd: do this only once during start 
	for(unsigned int i=0; i < YOUBOT_NR_OF_JOINTS;i++)
            m_in_motor[i] = (in_motor_t*) m_joints[i].inputs;

	fatal_err = check_status();

	// do this always so low level debug information as counters,
	// flags are available even during initialization.
	copyMotorStates();
	port_motor_states.write(m_motor_states);

	if(!configured()) {
	    __setControlMode(Initialize);
	} else {
	    copyJointStates();
	    port_joint_state.write(m_joint_state);
	    check_joint_limits();
	}

	// tbd: write only when changed.
	port_control_mode.write(control_mode2str(m_control_mode));

	for (unsigned int i = 0; i < YOUBOT_NR_OF_JOINTS; i++)
	    ((out_motor_t*) (m_joints[i].outputs))->controller_mode = m_control_modes[i];

	switch (m_control_mode) {
	case MotorStop:
	    break;
	case Positioning:
	    if (NoData != port_cmd_pos.read(m_cmd_pos)){
		if( m_cmd_pos.positions.size() == YOUBOT_NR_OF_JOINTS) {
		    for (unsigned int i = 0; i < YOUBOT_NR_OF_JOINTS; i++) {
			double pos_conversion = 2*M_PI/(YOUBOT_ARM_JOINT_GEAR_RATIOS[i]*YOUBOT_TICKS_PER_REVOLUTION);
			((out_motor_t*) (m_joints[i].outputs))->value	= m_cmd_pos.positions[i]/pos_conversion;
		    }
		} else {
		    log(Error) << "No valid port values." << endlog();
		}
	    }
	    break;
	case Velocity:
	    if (NoData != port_cmd_vel.read(m_cmd_vel)){
		if ( m_cmd_vel.velocities.size() == YOUBOT_NR_OF_JOINTS) {
		    for (unsigned int i = 0; i < YOUBOT_NR_OF_JOINTS; i++) {
			double vel_conversion = M_2_PI/(60*YOUBOT_ARM_JOINT_GEAR_RATIOS[i]);
			((out_motor_t*) (m_joints[i].outputs))->value = m_cmd_vel.velocities[i]/vel_conversion;
		    }
		} else {
		    log(Error) << "No valid port values." << endlog();
		}
	    }
	    break;
	case NoAction:
	    break;
	case Current:
	    if(NoData != port_cmd_eff.read(m_cmd_eff)){
		if ( m_cmd_eff.efforts.size() == YOUBOT_NR_OF_JOINTS) {
		    for (unsigned int i = 0; i < YOUBOT_NR_OF_JOINTS; i++) {
			double eff_conversion =
			    YOUBOT_ARM_JOINT_TORQUE_CONSTANTS[i]*YOUBOT_ARM_JOINT_GEAR_RATIOS[i]/1000;
			double out_cur = m_cmd_eff.efforts[i]/eff_conversion;
			((out_motor_t*) (m_joints[i].outputs))->value =
			    (out_cur > max_current[i]) ? max_current[i] : out_cur;
		    }
		} else {
		    log(Error) << "No valid port values." << endlog();
		}
	    }
	    break;
	case PWM:
	    log(Warning)<<"PWM mode not implemented yet"<<endlog();
	    break;
	case SetPositionToReference:
	    if (NoData != port_cmd_pos.read(m_cmd_pos)){
		if( m_cmd_pos.positions.size() == YOUBOT_NR_OF_JOINTS) {
		    for (unsigned int i = 0; i < YOUBOT_NR_OF_JOINTS; i++) {
			double pos_conversion = 2*M_PI/(YOUBOT_ARM_JOINT_GEAR_RATIOS[i]*YOUBOT_TICKS_PER_REVOLUTION);
			((out_motor_t*) (m_joints[i].outputs))->value	= m_cmd_pos.positions[i]/pos_conversion;
		    }
		} else {
		    log(Error) << "No valid port values." << endlog();
		}
	    }
	    break;
	case Initialize:
	    break;
	}
    }

    // Check the status flag, raise events and update counters.
    bool YoubotArmService::check_status()
    {
        bool fatal=false;
	int32 dummy;
	OperationCaller<bool(bool,uint8,uint8,uint8,int32&)> 
	    sendMBX(this->getParent()->getOperation("sendMBX"),this->getOwner()->engine());

        for(size_t i=0; i < YOUBOT_NR_OF_JOINTS; i++) {
            if (m_in_motor[i]->error_flags & OVERCURRENT) {
		m_motor_states.motor[i].counters.overcurrent++;
                port_events.write(make_event(m_events, "e_OVERCURRENT,jointid:", i));
                fatal=true;
            }
            if (m_in_motor[i]->error_flags & UNDERVOLTAGE) {
		m_motor_states.motor[i].counters.undervoltage++;
                port_events.write(make_event(m_events, "e_UNDERVOLTAGE,jointid:", i));
                m_control_modes[i]=MotorStop;
                fatal=true;
            }
            if (m_in_motor[i]->error_flags & OVERTEMP) {
		m_motor_states.motor[i].counters.overtemp++;
                port_events.write(make_event(m_events, "e_OVERTEMP,jointid:", i));
                m_control_modes[i]=MotorStop;
                fatal=true;
            }
            if (m_in_motor[i]->error_flags & HALL_ERR) {
		m_motor_states.motor[i].counters.hall_err++;
                port_events.write(make_event(m_events, "e_HALL_ERR,jointid:", i));
            }
            if (m_in_motor[i]->error_flags & ENCODER_ERR) {
		m_motor_states.motor[i].counters.encoder_err++;
                port_events.write(make_event(m_events, "e_ENCODER_ERR,jointid:", i));
            }
            if (m_in_motor[i]->error_flags & SINE_COMM_INIT_ERR) {
		m_motor_states.motor[i].counters.sine_comm_init_err++;
                port_events.write(make_event(m_events, "e_SINE_COMM_INIT_ERR,jointid:", i));
            }
            if (m_in_motor[i]->error_flags & EMERGENCY_STOP) {
		m_motor_states.motor[i].counters.emergency_stop++;
                port_events.write(make_event(m_events, "e_EMERGENCY_STOP,jointid:", i));
            }
	    
	    if (m_in_motor[i]->error_flags & MODULE_INIT) {
		if (! (module_init_status & (1 << i))) {
		    log(Info) << "arm: MODULE_INIT set for slave" <<i<< endlog();
		}
		module_init_status |= 1 << i;
	    }

	    // EC timeout
            if (m_in_motor[i]->error_flags & EC_TIMEOUT) {
		m_motor_states.motor[i].counters.ec_timeout++;
                port_events.write(make_event(m_events, "e_EC_TIMEOUT,jointid:", i));

                if(!sendMBX(false,SAP,CLR_EC_TIMEOUT,slave_nrs[i],dummy)) {
                    log(Warning) << "arm: failed to clear EC_TIMEOUT flag" << endlog();
                    fatal = true;
                }
            }

            // i2c exceeded rising edge
            if (m_in_motor[i]->error_flags & I2T_EXCEEDED && !m_i2t_ex[i]) {
		m_motor_states.motor[i].counters.i2t_exceeded++;
                port_events.write(make_event(m_events, "e_I2T_EXCEEDED,jointid:", i));
                m_control_mode=MotorStop;
                fatal=true;
                m_i2t_ex[i]=true;

		log(Warning) << "arm: i2t execeeded on wheel " << i << endlog();

		// reset while set (should probably be done externally)
                if(!sendMBX(false,SAP,CLEAR_I2T,slave_nrs[i],dummy))
                    log(Warning) << "arm: failed to clear I2T over flag" << endlog();

            } else if (!(m_in_motor[i]->error_flags & I2T_EXCEEDED) && m_i2t_ex[i]) {
                // i2c exceeded falling edge
                port_events.write(make_event(m_events, "e_I2T_EXCEEDED_EXIT,jointid:", i));
                m_i2t_ex[i]=false;
		log(Warning) << "arm: i2t execeeded reset on wheel " << i << endlog();
            }

        }
        return fatal;
    }

    // low level one
    bool YoubotArmService::__setControlMode(ControlMode mode)
    {
        m_control_mode = mode;
        for(unsigned int i = 0; i<YOUBOT_NR_OF_JOINTS ; i++)
            m_control_modes[i] = mode;
        return true;
    }
    
    // safe one available as operation.
    bool YoubotArmService::setControlMode(ControlMode mode){
	if (!configured()) {
	    log(Info) << "arm, setControlMode: refusing to set control mode before configured" << endlog();
	    return false;
	}
	return __setControlMode(mode);
    }

    void YoubotArmService::update(){
	process_data_handle();
    }
}
