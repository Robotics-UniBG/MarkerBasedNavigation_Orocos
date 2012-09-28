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

extern "C" {
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatconfig.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatprint.h"
}


#include "youbot_driver_component_complete.hpp"
#include "youbot_types.hpp"


#include <tf/tf.h>

namespace youbot_driver {

YoubotDriverComplete::YoubotDriverComplete(const std::string& name) :
	TaskContext(name, PreOperational),
	prop_ifname("eth0")
{
	this->addProperty("ifname", prop_ifname)
	    .doc("interface to which the ethercat device is connected");

	this->addPort("driver_state", drv_state_port).doc("driver lowlevel status information");
	this->addPort("events_out", events_out_port).doc("event out port");

	this->addOperation("sendMBX", &YoubotDriverComplete::sendMBX, this, OwnThread)
		.doc("send Trinamic Mailbox, check Trinamic documentation for values of the arguments")
	    .arg("gripper","boolean true if gripper").arg("instruction","instruction number")
	    .arg("type","parameter type")
	    .arg("slave","slave nr")
	    .arg("value","parameter value");

	this->addOperation("sendMBXGripper", &YoubotDriverComplete::sendMBXGripper, this, OwnThread)
	    .doc("send Trinamic Mailbox, check Trinamic documentation for values of the arguments")
	    .arg("instruction","instruction number")
	    .arg("type","parameter type")
	    .arg("slave","slave nr")
	    .arg("bank","bank nr")
	    .arg("value","parameter value");

	baseAddPortsAndOperations();

}

YoubotDriverComplete::~YoubotDriverComplete() {
}

bool YoubotDriverComplete::configureHook()
{
	Logger::In in(this->getName());
	// initialise Youbot, bind socket to ifname
	if (ec_init(prop_ifname.c_str()) <= 0) {
		log(Error) << "Slave configuration on "
				<< prop_ifname
				<< " failed in ec_init()! - Sufficient rights - correct interface?"
				<< endlog();
		return false;
	}

	log(Debug) << "ec_init on " << prop_ifname << " succeeded." << endlog();

	//Initialize default configuration, using the default config
	//table (see ethercatconfiglist.h)
	if (ec_config(true, &m_IOmap) <= 0) {
		log(Error) << "Configuration of slaves failed in ec_config()!" << endlog();
		return false;
	}

	log(Debug) << "Detected " << ec_slavecount << " slaves." << endlog();

	// Check the slaves that are attached
	bool success = false;
	int slave_nr=1;
	if (ec_slavecount >= YOUBOT_NR_OF_BASE_SLAVES){
		// Check the slave names: the first one is the power board
		success = baseCheckSlaves(slave_nr);
		if(success) {
			baseInitVariables(slave_nr);
			//this->provides()->addService(Service::shared_ptr(new YoubotBaseService("Base",this,slave_nr)));
			//update_ops.push_back(this->provides("Base")->getOperation("update"));
			slave_nr+=YOUBOT_NR_OF_BASE_SLAVES;
			//configure_ops.push_back(this->provides("Base")->getOperation("configure"));
			//start_ops.push_back(this->provides("Base")->getOperation("start"));
			log(Info) << "Detected youbot base, loading Base service" << endlog();
		}
	}

	// arm attached?
	if (ec_slavecount >= slave_nr+YOUBOT_NR_OF_ARM_SLAVES-1){
		success = armCheckSlaves(slave_nr);
		if(success) {
			armPresent = true;
//			armInitVariables(slave_nr);

			//this->provides()->addService(Service::shared_ptr(new YoubotArmService("Arm1",this,slave_nr)));
			//update_ops.push_back(this->provides("Arm1")->getOperation("update"));

			slave_nr+=YOUBOT_NR_OF_ARM_SLAVES;
			//configure_ops.push_back(this->provides("Arm1")->getOperation("configure"));
			//start_ops.push_back(this->provides("Arm1")->getOperation("start"));
			log(Info) << "Detected youbot arm, loading Arm1 service" << endlog();
		}
	}

	//	// 2nd arm?
	//	if (ec_slavecount >= slave_nr+YOUBOT_NR_OF_ARM_SLAVES-1){
	//		success = YoubotArm::check_slaves(slave_nr);
	//		if(success) {
	//			this->provides()->addService(Service::shared_ptr(new YoubotArmService("Arm2",this,slave_nr)));
	//			update_ops.push_back(this->provides("Arm2")->getOperation("update"));
	//			slave_nr+=YOUBOT_NR_OF_ARM_SLAVES;
	//			configure_ops.push_back(this->provides("Arm2")->getOperation("configure"));
	//			start_ops.push_back(this->provides("Arm2")->getOperation("start"));
	//			log(Info) << "Detected second youbot arm, loading Arm2 service" << endlog();
	//		}
	//	}

	// wait for all slaves to reach SAFE_OP state
	ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

	if (ec_slave[0].state == EC_STATE_SAFE_OP) {
		log(Debug) << "Safe operational state reached for all slaves." << endlog();
	} else {
		log(Error) << "Not all slaves reached safe operational state." << endlog();
		ec_readstate();
		//If not all slaves operational find out which one
		for (int i = 0; i <= ec_slavecount; i++) {
			if (ec_slave[i].state != EC_STATE_SAFE_OP) {
				log(Error) << "Slave " << i << " State= " <<
						to_string(ec_slave[i].state, std::hex) << " StatusCode="
						<< ec_slave[i].ALstatuscode << " : "
						<< ec_ALstatuscode2string(ec_slave[i].ALstatuscode) << endlog();
			}
		}
		return false;
	}
	// send (bogus) and receive (valid) process data. This is
	// necessary so the services have valid data available to
	// check wether they need to initialize or not.
	if (ec_send_processdata() <= 0)
		log(Warning) << "youbot_driver: failed to send boostrap process data" << endlog();

	if (ec_receive_processdata(EC_TIMEOUTRET) == 0)
		log(Warning) << "youbot_driver: failed to receive bootstrap process data" << endlog();

	// invoke all calibration operations
	// tbd: these could already check/set the Initialize control mode.
//	for(unsigned int i=0; i<configure_ops.size(); i++)
//		if(configure_ops[i].ready())
//			configure_ops[i]();
//		else
//			log(Error) << "configure ops << " << i << " not ready" << endlog();

	if(!baseConfigure()){
		return false;
	}
	if(armPresent){
		if(!armConfigure()){
			return false;
		}
	}

	return true;
}

bool YoubotDriverComplete::__sendMBX(bool gripper, uint8 instr_nr, uint8 param_nr,
		uint8 slave_nr, uint8 bank_nr, int32& value){
	ec_mbxbuft mbx_out, mbx_in;

	mbx_out[0] = (gripper) ? 1 : 0; // 1 for gripper, 0 for the rest
	mbx_out[1] = instr_nr; // Command number
	mbx_out[2] = param_nr; // Type number
	mbx_out[3] = bank_nr; // Always zero. Motor or Bank number
	mbx_out[4] = (uint32)value >> 24;
	mbx_out[5] = (uint32)value >> 16;
	mbx_out[6] = (uint32)value >> 8;
	mbx_out[7] = (uint32)value & 0xff;

	Logger::In in(this->getName()+"::sendMBX");

	if (!(ec_mbxsend(slave_nr, &mbx_out, EC_TIMEOUTSAFE)>0)){
		log(Error) <<"Could not send mailbox to slave "<<slave_nr<<endlog();
		return false;
	}

	if (!(ec_mbxreceive(slave_nr,&mbx_in,EC_TIMEOUTSAFE)>0)){
		log(Error) <<"Could not receive mailbox from slave "<<slave_nr<<endlog();
		return false;
	}

	if((int)mbx_in[2]==100){
		value = (mbx_in[4] << 24 | mbx_in[5] << 16 | mbx_in[6] << 8 | mbx_in[7]);
		log(Debug)<<"mbx received from host:" <<(int)mbx_in[0]
		                                                    <<", module:"<<(int)mbx_in[1]
		                                                                               <<", status:"<<(int)mbx_in[2]
		                                                                                                          <<", command:"<<(int)mbx_in[3]
		                                                                                                                                      <<", value:"<<value <<endlog();
		return true;
	} else {
		log(Error)<<"Setting parameter failed with status "<<(int)mbx_in[2]<<endlog();
		return false;
	}
}

bool YoubotDriverComplete::sendMBXGripper(uint8 instr_nr, uint8 param_nr,uint8 slave_nr,
		uint8 bank_nr, int32& value){
	return __sendMBX(true, instr_nr, param_nr, slave_nr, bank_nr, value);
}

bool YoubotDriverComplete::sendMBX(bool gripper, uint8 instr_nr, uint8 param_nr,
		uint8 slave_nr, int32& value){
	return __sendMBX(gripper, instr_nr, param_nr, slave_nr, 0, value);
}

bool YoubotDriverComplete::startHook() {
	log(Info) << "Request operational state for all slaves" << endlog();
	ec_slave[0].state = EC_STATE_OPERATIONAL;
	ec_writestate(0);

	// wait for all slaves to reach OP state
	ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
	if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
		log(Info) << "Operational state reached for all slaves." << endlog();
	} else {
		log(Error) << "Not all slaves reached operational state." << endlog();
		//If not all slaves operational find out which one
		for (int i = 1; i <= ec_slavecount; i++) {
			if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
				log(Error) << "Slave " << i << " State= " << to_string(ec_slave[i].state, std::hex)
			    																						   << " StatusCode=" << ec_slave[i].ALstatuscode << " : "
			    																						   << ec_ALstatuscode2string(ec_slave[i].ALstatuscode) << endlog();
			}
		}
		return false;
	}

	//	for(unsigned int i=0;i<start_ops.size();i++) {
	//		if(start_ops[i].ready()) {
	//			if(!start_ops[i]()) {
	//				log(Error) << "startHook calib_ops[" << i << "] returned false!" << endlog();
	//				return false;
	//			}
	//		} else {
	//			log(Warning) << "startHook calib_ops[" << i << "] not ready!" << endlog();
	//		}
	//	}

	if(!baseStart()){
		log(Error) << "base startHook calib_ops returned false!" << endlog();
		return false;
	}
	if(armPresent){
		if(!armStart()){
			log(Error) << "arm startHook calib_ops returned false!" << endlog();
			return false;
		}
	}

	// mk: get things rolling
	if (ec_send_processdata() <= 0) {
		drv_state.pd_send_err++;
		log(Warning) << "(Youbot) sending initial process data failed" << endlog();
		// todo: raise event
	}

	return true;
}

void YoubotDriverComplete::updateHook() {
	bool drv_state_updated = false;

	if (ec_receive_processdata(EC_TIMEOUTRET) == 0){
		drv_state.pd_recv_err++;
		drv_state_updated = true;
		// todo: raise event
	}

	//	for(unsigned int i=0;i<update_ops.size();i++) {
	//		if(update_ops[i].ready()) update_ops[i]();
	//		else log(Error) << "Youbot driver: update_ops " << i << " not ready" << endlog();
	//	}

	baseUpdate();
	if(armPresent){
		armUpdate();
	}

	/// Exchange data with ethercat slaves
	if (ec_send_processdata() <= 0){
		drv_state.pd_send_err++;
		drv_state_updated = true;
	}

	if(drv_state_updated)
		drv_state_port.write(drv_state);
}

void YoubotDriverComplete::stopHook() {
	ec_slave[0].state = EC_STATE_SAFE_OP;
	ec_writestate(0);
	// wait for all slaves to reach SAFE_OP state
	ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

}

void YoubotDriverComplete::cleanupHook() {
	//stop Youbot, close socket
	ec_slave[0].state = EC_STATE_PRE_OP;
	ec_writestate(0);
	ec_close();
}

/*******************************
 * Base Methods
 *******************************/

bool YoubotDriverComplete::baseCheckSlaves(unsigned int slave_nr) {
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
}

void YoubotDriverComplete::cartesianToWheelVelocities(geometry_msgs::Twist twist) {
	base_m_wheel_velocities[0] = ( -twist.linear.x + twist.linear.y + twist.angular.z *
			YOUBOT_ANGULAR_TO_WHEEL_VELOCITY ) * YOUBOT_CARTESIAN_VELOCITY_TO_RPM;

	base_m_wheel_velocities[1] = ( twist.linear.x + twist.linear.y + twist.angular.z *
			YOUBOT_ANGULAR_TO_WHEEL_VELOCITY ) * YOUBOT_CARTESIAN_VELOCITY_TO_RPM;

	base_m_wheel_velocities[2] = ( -twist.linear.x - twist.linear.y + twist.angular.z  *
			YOUBOT_ANGULAR_TO_WHEEL_VELOCITY ) * YOUBOT_CARTESIAN_VELOCITY_TO_RPM;

	base_m_wheel_velocities[3] = ( twist.linear.x - twist.linear.y + twist.angular.z *
			YOUBOT_ANGULAR_TO_WHEEL_VELOCITY ) * YOUBOT_CARTESIAN_VELOCITY_TO_RPM;
}

// Calculate odometry information: estimated velocity and travelled distance
void YoubotDriverComplete::calculateOdometry() {
	// wheel velocities to cartesian velocities
	base_m_odom.twist.twist.linear.x =
			(float) ( base_m_motor_states.motor[0].velocity - base_m_motor_states.motor[1].velocity
					+ base_m_motor_states.motor[2].velocity - base_m_motor_states.motor[3].velocity )
					/ (float) ( YOUBOT_NR_OF_WHEELS *  YOUBOT_CARTESIAN_VELOCITY_TO_RPM );

	base_m_odom.twist.twist.linear.y =
			(float) ( - base_m_motor_states.motor[0].velocity - base_m_motor_states.motor[1].velocity
					+ base_m_motor_states.motor[2].velocity + base_m_motor_states.motor[3].velocity )
					/ (float) ( YOUBOT_NR_OF_WHEELS * YOUBOT_CARTESIAN_VELOCITY_TO_RPM );

	base_m_odom.twist.twist.angular.z =
			(float) ( - base_m_motor_states.motor[0].velocity - base_m_motor_states.motor[1].velocity
					- base_m_motor_states.motor[2].velocity - base_m_motor_states.motor[3].velocity )
					/ (float) ( YOUBOT_NR_OF_WHEELS * YOUBOT_CARTESIAN_VELOCITY_TO_RPM * YOUBOT_ANGULAR_TO_WHEEL_VELOCITY);

	// wheel positions to cartesian positions
	// ugly hack: skip the first few samples - to make sure we start at 0 pose
	if(base_m_odom_started < 10) {
		for(size_t i = 0; i < YOUBOT_NR_OF_WHEELS; i++){
			base_m_last_wheel_pos[i] = base_m_motor_states.motor[i].position;
			base_m_delta_pos[i] = 0;
			base_m_odom_started++;
		}
	} else {
		for(size_t i = 0; i < YOUBOT_NR_OF_WHEELS; i++) {
			base_m_delta_pos[i] =
					(float) ( base_m_motor_states.motor[i].position - base_m_last_wheel_pos[i] )
					* (float) (YOUBOT_WHEEL_CIRCUMFERENCE)
					/ (float) (YOUBOT_TICKS_PER_REVOLUTION * YOUBOT_MOTORTRANSMISSION ) ;
			base_m_last_wheel_pos[i] = base_m_motor_states.motor[i].position;
		}

		base_m_odom_yaw += ( base_m_delta_pos[0] + base_m_delta_pos[1] + base_m_delta_pos[2] + base_m_delta_pos[3] )
																				/ ( YOUBOT_NR_OF_WHEELS * YOUBOT_ANGULAR_TO_WHEEL_VELOCITY );

		base_m_odom.pose.pose.position.x +=
				- ( ( ( base_m_delta_pos[0] - base_m_delta_pos[1] + base_m_delta_pos[2] - base_m_delta_pos[3] )
						/ YOUBOT_NR_OF_WHEELS ) * cos( base_m_odom_yaw )
						- ( ( - base_m_delta_pos[0] - base_m_delta_pos[1] + base_m_delta_pos[2] + base_m_delta_pos[3] )
								/ YOUBOT_NR_OF_WHEELS ) * sin( base_m_odom_yaw ));

		base_m_odom.pose.pose.position.y +=
				- ( ( ( base_m_delta_pos[0] - base_m_delta_pos[1] + base_m_delta_pos[2] - base_m_delta_pos[3] )
						/ YOUBOT_NR_OF_WHEELS ) * sin( base_m_odom_yaw )
						+ ( ( - base_m_delta_pos[0] - base_m_delta_pos[1] + base_m_delta_pos[2] + base_m_delta_pos[3] )
								/ YOUBOT_NR_OF_WHEELS ) * cos( base_m_odom_yaw ));

		base_m_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(base_m_odom_yaw);
	}
}

// Copy motor status information into ROS message
void YoubotDriverComplete::copyMotorStates()
{
	for(size_t i=0; i < YOUBOT_NR_OF_WHEELS; i++) {
		base_m_motor_states.motor[i].position = base_m_in_motor[i]->position;
		base_m_motor_states.motor[i].current = base_m_in_motor[i]->current;
		base_m_motor_states.motor[i].velocity = base_m_in_motor[i]->velocity;
		base_m_motor_states.motor[i].error_flags = base_m_in_motor[i]->error_flags;
		base_m_motor_states.motor[i].temperature = base_m_in_motor[i]->temperature;
	}
}

bool YoubotDriverComplete:: check_status_flag()
{
	bool fatal=false;
	for(size_t i=0; i < YOUBOT_NR_OF_WHEELS; i++) {
		if (base_m_in_motor[i]->error_flags & OVERCURRENT) {
			base_m_motor_states.motor[i].counters.overcurrent++;
			base_events.write(make_event(base_m_events, "e_OVERCURRENT,wheelid:", i));
			fatal=true;
		}
		if (base_m_in_motor[i]->error_flags & UNDERVOLTAGE) {
			base_m_motor_states.motor[i].counters.undervoltage++;
			base_events.write(make_event(base_m_events, "e_UNDERVOLTAGE,wheelid:", i));
			base_m_control_mode=MotorStop;
			fatal=true;
		}
		if (base_m_in_motor[i]->error_flags & OVERTEMP) {
			base_m_motor_states.motor[i].counters.overtemp++;
			base_events.write(make_event(base_m_events, "e_OVERTEMP,wheelid:",i));
			base_m_control_mode=MotorStop;
			fatal=true;
		}
		if (base_m_in_motor[i]->error_flags & HALL_ERR) {
			base_m_motor_states.motor[i].counters.hall_err++;
			base_events.write(make_event(base_m_events, "e_HALL_ERR,wheelid:", i));
		}
		if (base_m_in_motor[i]->error_flags & ENCODER_ERR) {
			base_m_motor_states.motor[i].counters.encoder_err++;
			base_events.write(make_event(base_m_events, "e_ENCODER_ERR,wheelid:", i));
		}
		if (base_m_in_motor[i]->error_flags & SINE_COMM_INIT_ERR) {
			base_m_motor_states.motor[i].counters.sine_comm_init_err++;
			base_events.write(make_event(base_m_events, "e_SINE_COMM_INIT_ERR,wheelid:", i));
		}
		if (base_m_in_motor[i]->error_flags & EMERGENCY_STOP) {
			base_m_motor_states.motor[i].counters.emergency_stop++;
			base_events.write(make_event(base_m_events, "e_EMERGENCY_STOP,wheelid:", i));
		}
		if (base_m_in_motor[i]->error_flags & MODULE_INIT) {
			if (! (base_module_init_status & (1 << i))) {
				log(Debug) << "base: MODULE_INIT set for slave" <<i<< endlog();
			}
			base_module_init_status |= 1 << i;
		}
		if (base_m_in_motor[i]->error_flags & EC_TIMEOUT) {
			base_m_motor_states.motor[i].counters.ec_timeout++;
			base_events.write(make_event(base_m_events, "e_EC_TIMEOUT,wheelid:", i));
			OperationCaller<bool(bool,uint8,uint8,uint8,int32&)>
			sendMBX(this->getOperation("sendMBX"),
					this->engine());
			int32 dummy;
			if(!sendMBX(false,SAP,CLR_EC_TIMEOUT,base_slave_nrs[i],dummy)) {
				log(Error) << "base: failed to clear EC_TIMEOUT flag" << endlog();
				fatal = true;
			}
		}

		// i2c exceeded rising edge
		if ((base_m_in_motor[i]->error_flags & I2T_EXCEEDED) && !base_m_i2t_ex[i]) {
			OperationCaller<bool(bool,uint8,uint8,uint8,int32&)>
			sendMBX(this->getOperation("sendMBX"),
					this->engine());
			int32 dummy;

			base_m_motor_states.motor[i].counters.i2t_exceeded++;
			base_events.write(make_event(base_m_events, "e_I2T_EXCEEDED,wheelid:", i));
			base_m_control_mode=MotorStop;
			log(Warning) << "base: i2t execeeded on wheel " << i << endlog();
			fatal=true;
			base_m_i2t_ex[i]=true;

			// reset while set (should probably be done externally)
			if(!sendMBX(false,SAP,CLEAR_I2T,base_slave_nrs[i],dummy))
				log(Warning) << "base: failed to clear I2T over flag" << endlog();

		} else if (!(base_m_in_motor[i]->error_flags & I2T_EXCEEDED) && base_m_i2t_ex[i]) {
			// i2c exceeded falling edge
			base_events.write(make_event(base_m_events, "e_I2T_EXCEEDED_EXIT,wheelid:", i));
			base_m_i2t_ex[i]=false;
			log(Warning) << "base: i2t exceeded reset on wheel " << i << endlog();
		}
	}
	return fatal;
}

// function to change base the control mode.
bool YoubotDriverComplete::setBaseControlMode(ControlMode mode)
{
	if (!configured()) {
		log(Warning) << "setControlMode: refusing to set control mode before configured" << endlog();
		return false;
	}

	if(mode==Velocity)
		resetWheelVel();

	base_m_control_mode = mode;
	return true;
}

bool YoubotDriverComplete::setDefParam(uint8 nr, int32 desval)
{
	int32 retval;
	bool succ = true;
	OperationCaller<bool(bool,uint8,uint8,uint8,int32&)>
	sendMBX(this->getOperation("sendMBX"),
			this->engine());

	for(unsigned int i=0;i<YOUBOT_NR_OF_WHEELS;i++) {
		log(Info) << "setDefParam: slave: " << base_slave_nrs[i] << "   param nr: " <<
				((int ) nr) << "   desval: " << desval << endlog();

		// read parameter value
		if(!sendMBX(false, GAP, nr, base_slave_nrs[i], retval)) {
			log(Error) << "setDefParam: failed to read param nr " << nr << endlog();
			return false;
		}

		// param is not set correctly, we need to set it.
		if (retval != desval) {
			log(Info) << "setDefParam: param nr not yet set. trying to set" << endlog();
			// set param
			if(!sendMBX(false, SAP, nr, base_slave_nrs[i], desval)) {
				log(Error) << "setDefParam: setting param nr " << nr << " to " << desval
						<< " failed. pwd?" << endlog();
				succ = false;
			}

			// re-read and check
			if(!sendMBX(false, GAP, nr, base_slave_nrs[i], retval)) {
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
bool YoubotDriverComplete::setDefParams()
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
void YoubotDriverComplete::check_wd(bool trigger)
{
	os::TimeService::nsecs cur_nsec = os::TimeService::Instance()->getNSecs();
	os::TimeService::nsecs diff_nsec = os::TimeService::Instance()->getNSecs(base_m_last_cmd_twist);

	if(diff_nsec >= base_m_timeout_nsec) {
		base_events.write("e_base_cmd_watchdog");
		log(Error) << "base: watchdog emergency stop. No base twist in "
				<< base_timeout << "s" << endlog();
		base_m_control_mode = MotorStop;
	}

	if(trigger)
		base_m_last_cmd_twist = cur_nsec;
}

void YoubotDriverComplete::resetWheelVel()
{
	for(int i=0; i<YOUBOT_NR_OF_WHEELS; i++)
		base_m_wheel_velocities[i] = 0;

	base_m_last_cmd_twist = os::TimeService::InfiniteNSecs;
}

bool YoubotDriverComplete::configured() {
	return base_module_init_status & ((1 << YOUBOT_NR_OF_BASE_SLAVES)-1);
}

bool YoubotDriverComplete::baseConfigure(){
	log(Info) << "Configuring base." << endlog();
	setDefParams();
	return true;
}

void YoubotDriverComplete::baseAddPortsAndOperations(){
	this->addPort("cmd_twist",base_cmd_twist)
		    		.doc("Commanded twist, expressed in the middle of the base");

	this->addPort("odometry",base_odom)
		    		.doc("Odometry, from wheel positions, expr. in middle of the base WRT to starting point");

	this->addOperation("setControlMode",&YoubotDriverComplete::setBaseControlMode,this)
				    		.doc("Sets control mode to the desired one")
				    		.arg("mode","Desired control mode");

	this->addPort("motor_states", base_motor_states).doc("Current state of motors");
	this->addPort("control_mode", base_control_mode).doc("Currently active control_mode");
	this->addPort("events", base_events).doc("Event outport");

}

void YoubotDriverComplete::baseInitVariables(unsigned int slave_nr){

	for(unsigned int i=0;i<YOUBOT_NR_OF_WHEELS;i++){
		base_m_wheels[i] = ec_slave[++slave_nr];
		base_slave_nrs[i] = slave_nr;
		base_m_i2t_ex[i] = false;
	}

	base_m_timeout_nsec = base_timeout * 1000*1000*1000; // sec -> nsec
	base_m_motor_states.motor.resize(YOUBOT_NR_OF_WHEELS);
	base_m_events.reserve(max_event_strlen);
	base_module_init_status=0;

	// odometry pose estimates frame
	base_m_odom.header.frame_id = "odom";
	// odometry twist estimates frame
	base_m_odom.child_frame_id = "base_link";
	// odometry estimates - set to zero
	base_m_odom.pose.pose.position.x = 0;
	base_m_odom.pose.pose.position.y = 0;
	base_m_odom.pose.pose.position.z = 0;
	base_m_odom.pose.pose.orientation.x = 0;
	base_m_odom.pose.pose.orientation.y = 0;
	base_m_odom.pose.pose.orientation.z = 0;
	base_m_odom.pose.pose.orientation.w = 0;
	base_m_odom.twist.twist.linear.x = 0;
	base_m_odom.twist.twist.linear.y = 0;
	base_m_odom.twist.twist.linear.z = 0;
	base_m_odom.twist.twist.angular.x = 0;
	base_m_odom.twist.twist.angular.y = 0;
	base_m_odom.twist.twist.angular.z = 0;
	base_m_odom_yaw = 0;
	base_m_odom_started = 0;

	base_m_control_mode = MotorStop; //Initialize;
}

bool YoubotDriverComplete::baseStart() {
	int32 dummy;

	OperationCaller<bool(bool,uint8,uint8,uint8,int32&)>
	sendMBX(this->getOperation("sendMBX"),
			this->engine());

	// set wheel velocity to zero and reset EC_TIMEOUT
	resetWheelVel();

	for(int i=0; i<YOUBOT_NR_OF_WHEELS; i++) {
		if(!sendMBX(false,SAP,CLR_EC_TIMEOUT,base_slave_nrs[i],dummy)) {
			log(Error) << "base: failed to clear EC_TIMEOUT flag" << endlog();
			return false;
		}
	}

	log(Info) << "YoubotBaseService: started" << endlog();
	return true;
}

void YoubotDriverComplete::baseUpdate() {
	geometry_msgs::Twist twist;
	bool fatal_err;
	FlowStatus fs;

	// Process received data
	// mk, todo: why one earth are we repeating this each time???
	for (size_t i = 0; i < YOUBOT_NR_OF_WHEELS; i++)
		base_m_in_motor[i] = (in_motor_t*) base_m_wheels[i].inputs;

	fatal_err = check_status_flag(); // raise events ASAP
	copyMotorStates();
	base_motor_states.write(base_m_motor_states);

	if (!configured()) {
		base_m_control_mode=Initialize;
	} else {
		calculateOdometry();
		base_odom.write(base_m_odom);
	}

	// tbd: only write when changed
	base_control_mode.write(control_mode2str(base_m_control_mode));

	for (size_t i = 0; i < YOUBOT_NR_OF_WHEELS; i++)
		((out_motor_t*) (base_m_wheels[i].outputs))->controller_mode = base_m_control_mode;

	switch (base_m_control_mode) {
	case MotorStop:
		break;
	case Velocity:
		fs = base_cmd_twist.read(twist);
		if ( fs == NewData ) {
			cartesianToWheelVelocities(twist); // this sets up m_wheel_velocities
			check_wd(true);
		} else if (base_m_wheel_velocities[0] == 0 && base_m_wheel_velocities[1] == 0 &&
				base_m_wheel_velocities[2] == 0 && base_m_wheel_velocities[3] == 0 ) {
			check_wd(true); // if velocity is zero don't timeout.
		} else {
			check_wd(false); // Not NewData and Velocity != 0
		}

		for (size_t i = 0; i < YOUBOT_NR_OF_WHEELS; i++)
			((out_motor_t*) (base_m_wheels[i].outputs))->value = base_m_wheel_velocities[i];

		break;
	case Initialize:
		break;
	default:
		log(Info) << "base::update: unexpected control mode: " << base_m_control_mode << endlog();
		break;
	}
}

bool YoubotDriverComplete::armCheckSlaves(unsigned int slave_nr){
	return false;
}

bool YoubotDriverComplete::armConfigure(){
	return true;
}
bool YoubotDriverComplete::armStart(){
	return true;
}
void YoubotDriverComplete::armUpdate(){
	return;
}


}//namespace

#include <rtt/Component.hpp>

ORO_CREATE_COMPONENT( youbot_driver::YoubotDriverComplete)
