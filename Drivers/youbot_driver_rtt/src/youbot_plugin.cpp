/*
 * KUKA Youbot component for Orocos RTT
 *
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

#include <rtt/RTT.hpp>
#include <rtt/plugin/Plugin.hpp>
#include <rtt/types/GlobalsRepository.hpp>
#include "youbot_types.hpp"

using namespace RTT;
using namespace youbot_driver;

bool loadRTTPlugin( RTT::TaskContext* t )
{
  types::GlobalsRepository::shared_ptr globals = types::GlobalsRepository::Instance();

  globals->addConstant("MotorStop" , MotorStop );
  globals->addConstant("Positioning" , Positioning);
  globals->addConstant("Velocity" , Velocity);
  globals->addConstant("NoAction" , NoAction);
  globals->addConstant("SetPositionToReference" , SetPositionToReference);
  globals->addConstant("PWM" , PWM);
  globals->addConstant("Current" , Current);
  globals->addConstant("Initialize" , Initialize);

  globals->addConstant("ROR",ROR);
  globals->addConstant("ROL",ROL);
  globals->addConstant("MST",MST);
  globals->addConstant("MVP",MVP);
  globals->addConstant("SAP",SAP);
  globals->addConstant("GAP",GAP);
  globals->addConstant("STAP",STAP);
  globals->addConstant("RSAP",RSAP);
  globals->addConstant("SGP",SGP);
  globals->addConstant("GGP",GGP);
  globals->addConstant("STGP",STGP);
  globals->addConstant("RSGP",RSGP);

  globals->addConstant("TARGET_POS",TARGET_POS);
  globals->addConstant("ACTUAL_POS",ACTUAL_POS);
  globals->addConstant("TARGET_SPEED",TARGET_SPEED);
  globals->addConstant("ACTUAL_SPEED",ACTUAL_SPEED);
  globals->addConstant("MAX_RAMP_VEL",MAX_RAMP_VEL);
  globals->addConstant("PWM_LIMIT",PWM_LIMIT);
  globals->addConstant("MAX_CURRENT",MAX_CURRENT);
  globals->addConstant("MAX_VEL_SETPOS",MAX_VEL_SETPOS);
  globals->addConstant("VEL_PID_THRES",VEL_PID_THRES);
  globals->addConstant("CLR_TARGET_DIST",CLR_TARGET_DIST);
  globals->addConstant("MAX_DIST_SETPOS",MAX_DIST_SETPOS);
  globals->addConstant("ACCEL",ACCEL);
  globals->addConstant("POS_PID_THRES",POS_PID_THRES);
  globals->addConstant("RAMP_GEN_SPEED",RAMP_GEN_SPEED);
  globals->addConstant("INIT_BLDC",INIT_BLDC);
  globals->addConstant("CLR_EC_TIMEOUT",CLR_EC_TIMEOUT);

  return true;
}

std::string getRTTPluginName()
{
  return "youbot-plugin";
}

