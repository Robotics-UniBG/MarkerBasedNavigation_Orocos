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


#include "youbot_driver_component.hpp"
#include "youbot_types.hpp"
#include "youbot_base_service.hpp"
#include "youbot_arm_service.hpp"

namespace youbot_driver {

    using namespace RTT;

    YoubotDriver::YoubotDriver(const std::string& name) :
	TaskContext(name, PreOperational)
	,prop_ifname("eth0")
    {
	this->addProperty("ifname", prop_ifname)
	    .doc("interface to which the ethercat device is connected");

	this->addPort("driver_state", drv_state_port).doc("driver lowlevel status information");
	this->addPort("events_out", events_out_port).doc("event out port");

	this->addOperation("sendMBX",&YoubotDriver::sendMBX,this,OwnThread)
	    .doc("send Trinamic Mailbox, check Trinamic documentation for values of the arguments")
	    .arg("gripper","boolean true if gripper").arg("instruction","instruction number")
	    .arg("type","parameter type")
	    .arg("slave","slave nr")
	    .arg("value","parameter value");

	this->addOperation("sendMBXGripper", &YoubotDriver::sendMBXGripper, this, OwnThread)
	    .doc("send Trinamic Mailbox, check Trinamic documentation for values of the arguments")
	    .arg("instruction","instruction number")
	    .arg("type","parameter type")
	    .arg("slave","slave nr")
	    .arg("bank","bank nr")
	    .arg("value","parameter value");
    }

    YoubotDriver::~YoubotDriver() {
    }

    bool YoubotDriver::configureHook()
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
	    success = YoubotBase::check_slaves(slave_nr);
	    if(success) {
		this->provides()->addService(Service::shared_ptr(new YoubotBaseService("Base",this,slave_nr)));
		update_ops.push_back(this->provides("Base")->getOperation("update"));
		slave_nr+=YOUBOT_NR_OF_BASE_SLAVES;
		configure_ops.push_back(this->provides("Base")->getOperation("configure"));
		start_ops.push_back(this->provides("Base")->getOperation("start"));
		log(Info) << "Detected youbot base, loading Base service" << endlog();
	    }
	}

	// arm attached?
	if (ec_slavecount >= slave_nr+YOUBOT_NR_OF_ARM_SLAVES-1){
	    success = YoubotArm::check_slaves(slave_nr);
	    if(success) {
		this->provides()->addService(Service::shared_ptr(new YoubotArmService("Arm1",this,slave_nr)));
		update_ops.push_back(this->provides("Arm1")->getOperation("update"));
		slave_nr+=YOUBOT_NR_OF_ARM_SLAVES;
		configure_ops.push_back(this->provides("Arm1")->getOperation("configure"));
		start_ops.push_back(this->provides("Arm1")->getOperation("start"));
		log(Info) << "Detected youbot arm, loading Arm1 service" << endlog();
	    }
	}

	// 2nd arm?
	if (ec_slavecount >= slave_nr+YOUBOT_NR_OF_ARM_SLAVES-1){
	    success = YoubotArm::check_slaves(slave_nr);
	    if(success) {
		this->provides()->addService(Service::shared_ptr(new YoubotArmService("Arm2",this,slave_nr)));
		update_ops.push_back(this->provides("Arm2")->getOperation("update"));
		slave_nr+=YOUBOT_NR_OF_ARM_SLAVES;
		configure_ops.push_back(this->provides("Arm2")->getOperation("configure"));
		start_ops.push_back(this->provides("Arm2")->getOperation("start"));
		log(Info) << "Detected second youbot arm, loading Arm2 service" << endlog();
	    }
	}

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
	for(unsigned int i=0; i<configure_ops.size(); i++)
	    if(configure_ops[i].ready())
		configure_ops[i]();
	    else
		log(Error) << "configure ops << " << i << " not ready" << endlog();

	return true;
    }

    bool YoubotDriver::__sendMBX(bool gripper, uint8 instr_nr, uint8 param_nr,
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

    bool YoubotDriver::sendMBXGripper(uint8 instr_nr, uint8 param_nr,uint8 slave_nr,
					       uint8 bank_nr, int32& value){
	return __sendMBX(true, instr_nr, param_nr, slave_nr, bank_nr, value);
    }

    bool YoubotDriver::sendMBX(bool gripper, uint8 instr_nr, uint8 param_nr,
					uint8 slave_nr, int32& value){
	return __sendMBX(gripper, instr_nr, param_nr, slave_nr, 0, value);
    }

    bool YoubotDriver::startHook() {
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
	for(unsigned int i=0;i<start_ops.size();i++) {
	    if(start_ops[i].ready()) {
		if(!start_ops[i]()) {
		    log(Error) << "startHook calib_ops[" << i << "] returned false!" << endlog();
		    return false;
		}
	    } else {
		log(Warning) << "startHook calib_ops[" << i << "] not ready!" << endlog();
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

    void YoubotDriver::updateHook() {
	bool drv_state_updated = false;

	if (ec_receive_processdata(EC_TIMEOUTRET) == 0){
	    drv_state.pd_recv_err++;
	    drv_state_updated = true;
	    // todo: raise event
	}

	for(unsigned int i=0;i<update_ops.size();i++) {
	    if(update_ops[i].ready()) update_ops[i]();
	    else log(Error) << "Youbot driver: update_ops " << i << " not ready" << endlog();
	}

	/// Exchange data with ethercat slaves
	if (ec_send_processdata() <= 0){
	    drv_state.pd_send_err++;
	    drv_state_updated = true;
	}

	if(drv_state_updated)
	    drv_state_port.write(drv_state);
    }

    void YoubotDriver::stopHook() {
	ec_slave[0].state = EC_STATE_SAFE_OP;
	ec_writestate(0);
	// wait for all slaves to reach SAFE_OP state
	ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    }

    void YoubotDriver::cleanupHook() {
	//stop Youbot, close socket
	ec_slave[0].state = EC_STATE_PRE_OP;
	ec_writestate(0);
	ec_close();
    }
}//namespace

#include <rtt/Component.hpp>

ORO_CREATE_COMPONENT( youbot_driver::YoubotDriver)
