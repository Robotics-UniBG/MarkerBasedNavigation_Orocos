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

#ifndef __YOUBOT_TYPES_HPP__
#define __YOUBOT_TYPES_HPP__

#include <youbot_msgs/typekit/motor_states.h>

namespace YoubotDriver {

static const unsigned int max_event_strlen = 100; // maxium length of events

#define YOUBOT_NR_OF_WHEELS 4
#define YOUBOT_NR_OF_JOINTS 5
#define YOUBOT_HAS_ARM 1
#define YOUBOT_NR_OF_BASE_SLAVES (YOUBOT_NR_OF_WHEELS + 1) //wheels + 1 power boards
#define YOUBOT_NR_OF_ARM_SLAVES (YOUBOT_NR_OF_JOINTS + 1) //arm joints, 1 power boards

#define YOUBOT_MOTOR_DEFAULT_MAX_CURRENT 8000 //[mA]
#define YOUBOT_MOTOR_DEFAULT_NOMINAL_SPEED  100//[r/s]

#define YOUBOT_WHEELCONTROLLER_SLAVENAME "TMCM-174"
#define YOUBOT_JOINTCONTROLLER_SLAVENAME "TMCM-KR-841"
#define YOUBOT_WHEELPOWERBOARD_SLAVENAME "KR-845"
#define YOUBOT_ARMPOWERBOARD_SLAVENAME "KR-843"



#define YOUBOT_WHEELRADIUS 0.052
#define YOUBOT_MOTORTRANSMISSION 26
#define YOUBOT_JOINT_INIT_VALUE 0
    /// @name Youbot base dimensions
    //@{
#define YOUBOT_FRONT_TO_REAR_WHEEL 0.47
#define YOUBOT_LEFT_TO_RIGHT_WHEEL 0.3
    //@}

#define YOUBOT_TICKS_PER_REVOLUTION 4096
#define YOUBOT_WHEEL_CIRCUMFERENCE (YOUBOT_WHEELRADIUS*2*M_PI)
    /// cartesian [ m/s ] to motor [ rpm ] velocity
#define YOUBOT_CARTESIAN_VELOCITY_TO_RPM (YOUBOT_MOTORTRANSMISSION*60)/(YOUBOT_WHEEL_CIRCUMFERENCE)
    /// angular [ rad/s ] to wheel [ m/s] velocity
#define  YOUBOT_ANGULAR_TO_WHEEL_VELOCITY (YOUBOT_FRONT_TO_REAR_WHEEL+YOUBOT_LEFT_TO_RIGHT_WHEEL)/2
#define PWM_VALUE_MAX 1799
    //@}

    // timeout after which base is stopped when no new twist is received.
    static const double base_timeout = 1;

    /// Possible control modes for the motors
    typedef uint8_t ControlMode;
    static const ControlMode MotorStop = 0;
    static const ControlMode Positioning = 1;
    static const ControlMode Velocity = 2;
    static const ControlMode NoAction = 3;
    static const ControlMode SetPositionToReference = 4;
    static const ControlMode PWM = 5;
    static const ControlMode Current = 6;
    static const ControlMode Initialize = 7;
    
    typedef uint8_t TMCLCommand;
    static const TMCLCommand ROR = 1;
    static const TMCLCommand ROL = 2;
    static const TMCLCommand MST = 3;
    static const TMCLCommand MVP = 4;
    static const TMCLCommand SAP = 5;
    static const TMCLCommand GAP = 6;
    static const TMCLCommand STAP = 7;
    static const TMCLCommand RSAP = 8;
    static const TMCLCommand SGP = 9;
    static const TMCLCommand GGP = 10;
    static const TMCLCommand STGP = 11;
    static const TMCLCommand RSGP = 12;

    typedef uint8_t AxisParam;
    static const AxisParam TARGET_POS = 0;
    static const AxisParam ACTUAL_POS = 1;
    static const AxisParam TARGET_SPEED = 2;
    static const AxisParam ACTUAL_SPEED = 3;
    static const AxisParam MAX_RAMP_VEL = 4;
    static const AxisParam PWM_LIMIT = 5;
    static const AxisParam MAX_CURRENT = 6;
    static const AxisParam MAX_VEL_SETPOS = 7;
    static const AxisParam VEL_PID_THRES = 8;
    static const AxisParam CLR_TARGET_DIST = 9;
    static const AxisParam MAX_DIST_SETPOS = 10;
    static const AxisParam ACCEL = 11;
    static const AxisParam POS_PID_THRES = 12;
    static const AxisParam RAMP_GEN_SPEED = 13;
    static const AxisParam INIT_BLDC = 15;

    static const AxisParam CLEAR_I2T = 29;
    // ..
    static const AxisParam  CLR_EC_TIMEOUT = 158;
    /* more to be added on demand... */

    /// @name Ethercat structs (documentation as given in the EtherCAT
    /// Manual posted at youbot@best-of-robotics.org on 23/12/2010 by
    /// Bischof Rainer)

    ///Output buffer protocol
    typedef struct {
        int32_t value; //Reference (Position, Velocity, PWM or Current)
        uint8_t controller_mode; //Controller Mode
        //0: Motor stop
        //1: Positioning mode
        //2: Velocity mode
        //3: no more action
        //4: set position to reference
        //5: PWM mode
        //6: Current mode
        //7: Initialize
    } out_motor_t;

    ///Input buffer protocol
    typedef struct {
        int32_t position;//Actual position, 32-bit Up-Down Counter, 4096 ticks/revolution
        int32_t current;//Actual current in mA
        int32_t velocity;//Actual velocity rpm motor axis
        uint32_t error_flags;//Error flags
        //bit 0: OverCurrent
        //bit 1: UnderVoltage
        //bit 2: OverVoltage
        //bit 3: OverTemperature
        //bit 4: Motor Halted
        //bit 5: Hall-Sensor error
        //bit 6: Encoder error
        //bit 7: Sine commutation initilization error
        //bit 8: PWM mode active
        //bit 9: Velocity mode active
        //bit 10: Position mode active
        //bit 11: Current mode active
        //bit 12: Emergency stop
        //bit 13: Freerunning
        //bit 14: Position end
        //bit 15: Module initialized
        //bit 16: EtherCAT timeout (reset with SAP 158)
        //bit 17: I2t exceeded (reset with SAP 29)
        uint16_t temperature;//Driver Temperature (on-board NTC), 0..4095
        //25C: 1.34V..1.43V..1.52V (Has to be configured)
        //1C: 4mV..4.3mV..4.6mV
        //Temp: 25.0 + ( ADC * 3.3V / 4096 - 1.43 ) / 0.0043
    } in_motor_t;


#define OVERCURRENT        1<<0
#define UNDERVOLTAGE       1<<1
#define OVERVOLTAGE        1<<2
#define OVERTEMP           1<<3
#define MOTOR_HALTED       1<<4
#define HALL_ERR           1<<5
#define ENCODER_ERR        1<<6
#define SINE_COMM_INIT_ERR 1<<7
#define PWM_MODE_ACT       1<<8
#define VEL_MODE_ACT       1<<9
#define POS_MODE_ACT       1<<10
#define CUR_MODE_ACT       1<<11
#define EMERGENCY_STOP     1<<12
#define FREERUNNING        1<<13
#define POSITION_END       1<<14
#define MODULE_INIT        1<<15
#define EC_TIMEOUT         1<<16
#define I2T_EXCEEDED       1<<17

    static const int YOUBOT_ARM_JOINT_GEAR_RATIOS[YOUBOT_NR_OF_JOINTS] = {156, 156, 100, 71, 71};

    static const double YOUBOT_ARM_JOINT_TORQUE_CONSTANTS[YOUBOT_NR_OF_JOINTS] = 
	{0.0335, 0.0335, 0.0335, 0.051, 0.049}; //Nm/A

    static const double YOUBOT_ARM_SOFT_LIMIT = 0.9; // legal yb soft limit range (%)
    // Joint limits in (deg) (from youBot manual v.82 pg. 7)
    static const double YOUBOT_ARM_LOWER_LIMIT[YOUBOT_NR_OF_JOINTS] = { -169, -90, -146, -102.5, -167.5 };
    static const double YOUBOT_ARM_UPPER_LIMIT[YOUBOT_NR_OF_JOINTS] = { 169, 65, 151, 102.5, 167.5 };
}

#endif
