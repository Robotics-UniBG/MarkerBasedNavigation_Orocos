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

#ifndef YOUBOT_DRIVER_COMPONENT_H
#define YOUBOT_DRIVER_COMPONENT_H

#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/Timer.hpp>

#include "youbot_helpers.hpp"
#include <youbot_msgs/typekit/driver_state.h>

namespace youbot_driver {

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

        std::string prop_ifname;
        char m_IOmap[4096];
	youbot_msgs::driver_state drv_state;

        bool __sendMBX(bool gripper, uint8 instr_nr,uint8 param_nr, uint8 slave_nr, uint8 bank_nr, int32& value);
        bool sendMBX(bool gripper, uint8 instr_nr,uint8 param_nr, uint8 slave_nr, int32& value);
        bool sendMBXGripper(uint8 instr_nr,uint8 param_nr, uint8 slave_nr, uint8 bank_nr, int32& value);

        std::vector<RTT::OperationCaller<bool(void)> > configure_ops;
        std::vector<RTT::OperationCaller<bool(void)> > start_ops;
        std::vector<RTT::OperationCaller<void(void)> > update_ops;

	RTT::OutputPort<youbot_msgs::driver_state> drv_state_port;
	RTT::OutputPort<std::string> events_out_port;

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
