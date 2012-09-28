--- TMCL command lua dsl.
-- This module ...
---

local ac = require "ansicolors"
local utils = require "utils"

local error = error
local rtt = rtt
local print = print
local pairs = pairs
local ipairs = ipairs
local table = table
local string = string
local tostring = tostring

module "TMCL"

local function sendMBX()
   error("sendMBX undefined")
end

local function sendMBXGripper()
   error("sendMBX undefined")
end

--- set_sendMBX
-- @param op RTT Operation that matches the sendMBX function
---
function set_sendMBX(op) sendMBX=op end
function set_sendMBXGripper(op) sendMBXGripper=op end

-- some usefull default slaves
-- to be used for the get_ and set_params functions.
def_slaves  = { 2, 3, 4, 5, 7, 8, 9, 10, 11 }
base_slaves = { 2, 3, 4, 5 }
arm_slaves  = { 7, 8, 9, 10, 11 }

--- Commands
ROR = 1   -- rotate right
ROL = 2   -- rotate left
MST = 3   -- stop motor
MVP = 4	  -- move to position
SAP = 5	  -- set axis param
GAP = 6	  -- get axis param
STAP = 7  -- store axis param
RSAP = 8  -- restore axis param
VERS = 136 -- get firmware version

--- Control Modes
CMODE_MOTOR_STOP	= 0 -- Motor stop
CMODE_POS		= 1 -- Positioning-Mode
CMODE_VEL		= 2 -- Velocity-Mode
CMODE_NO_ACT 		= 3 -- no more action
CMODE_SET_POS_TO_REF	= 4 -- set position to reference
CMODE_PWM	 	= 5 -- PWM-Mode
CMODE_CUR		= 6 -- Current-Mode
CMODE_INIT		= 7 -- Initialize

--- Internal configuration values.
-- encoder ticks to open and close gripper
local gripper_enc_width = 67000

--- Raw send command
function send(grip,comm, type, slave, value)
   return sendMBX(grip,comm,type, slave,value)
end

--- Open the gripper.
function gripper_open()
   sendMBXGripper(MVP, 1, 11, 0, -gripper_enc_width)
   sendMBXGripper(MVP, 1, 11, 1, -gripper_enc_width)
end

--- Close the gripper.
function gripper_close()
   sendMBXGripper(MVP, 1, 11, 0, gripper_enc_width)
   sendMBXGripper(MVP, 1, 11, 1, gripper_enc_width)
end

function version()
   return send(false, VERS, 0, 0, 0)
end

--- Set axis parameter
-- @param type parameter type
-- @param slave slave to set the parameter to
-- @param value desired parameter value
-- @return true if succesfull, false otherwise
function sap(type,slave,value)
   return sendMBX(false,SAP,type,slave,value)
end

--- Get axis parameter
-- @param type parameter type
-- @param slave slave to get the parameter from
-- @param value actual parameter value (output)
-- @return true if succesfull, false otherwise
-- @return value of the parameter
function gap(type,slave)
   local l_value = rtt.Variable("int32")
   local err = sendMBX(false,GAP,type,slave,l_value)
   return err, l_value:tolua()
end

--- Store axis parameter in EEPROM
-- @param type parameter type
-- @param slave slave to set the parameter to
-- @return true if succesfull, false otherwise
function stap(type,slave)
   return sendMBX(false,STAP,type,slave,0)
end

--- Restore axis parameter to value stored in EEPROM
-- @param type parameter type
-- @param slave slave to restore the parameter from
-- @return true if succesfull, false otherwise
function rsap(type,slave)
   local err = sendMBX(false,RSAP,type,slave,0)
end

axis_param = {
   TARGET_VEL={ num=0, desc='The target position of a currently executed ramp.' },
   ACTUAL_POS={ num=1, desc='Set/get the position counter without moving the motor.' },
   TARGET_VEL={ num=2, unit='rpm', desc='Set/get the desired target velocity.' },
   ACTUAL_VEL={ num=3, unit='rpm', desc='The actual velocity of the motor.' },
   MAX_RAMP_VEL={ num=4, unit='rpm', desc='The maximum velocity used for velocity ramp in velocity mode and positioning mode. Insensorless commutation mode used as [rpm] velocity threshold for hallFXTM.'  },
   PWM_LIMIT={ num=5, desc='Set/get PWM limit (0%... 100%).' },
   MAX_CURRENT={ num=6, unit='mA', desc='Set/get the max allowed motor current.' },
   MVP_TARGET_VEL={ num=7, unit='rpm', desc='Maximum velocity at which end position can be set. Prevents issuing of end position when the target is passed at high velocity.' },
   THRES_VEL_PID={ num=8, unit='rpm', desc="Threshold speed for velocity regulation to for velocity PID switch between first and second velocity PID parameter set." },
   CLR_TGT_DIST={ num=9, desc="Velocity is set to 0 if actual position differs distance from motor position for more than value, until the motor catches up." },

   MVP_TARGET_DIST={ num=10, desc="Maximum distance at which the position end reached distance flag is set." },

   ACCELERATION={ num=11, unit='rpm/s', desc="Acceleration parameter for ROL, ROR, and the velocity ramp of MVP. " },
   THRES_POS_PID={ num=12, unit='rpm', desc="Threshold speed for position regulation to for position PID switch between first and second position PID parameter set." },

   RAMP_GEN_VEL={ num=13, unit='rpm', desc="The actual speed of the velocity ramp used for positioning and velocity mode." },
   INIT_BLDC={ num=15, desc="SAP:do init and homing, GAP: get initalization state" },
   THERMAL_WIND={ num=25, unit='ms', desc="Thermal winding time constant for the used motor. Used for I2t monitoring" },
   I2T_LIMIT={ num=26, desc="An actual I2t sum that exceeds this limit leads to increasing the I2t exceed counter." },
   I2T_SUM={ num=27, desc="Actual sum of the I2t monitor." },
   I2T_EXCEED_CNT={num=28, desc="Counts how often an I2t sum was higher than the I2t limit." },
   CLEAR_I2T_EXCEED={ num=29, desc="Clear the flag that indicates that the I2t sum has exceeded the I2t limit" },
   MINUTE_CNT={ num=30, unit='min', desc="Counts the module operational time in minutes." },

   P_POS_PID_I={ num=130, desc="P parameter of position PID regulator (first position PID (I) parameter set)" },
   I_POS_PID_I={ num=131, desc="I parameter of position PID regulator (first position PID (I) parameter set)" },
   D_POS_PID_I={ num=132, desc="D parameter of position PID regulator (first position PID (I) parameter set)" },
   PID_LOOP_DELAY={ num=133, unit='ms', desc="PID calculation delay. Set PID operational frequency" },
   CUR_LOOP_DELAY={ num=134, unit='50us', desc="Delay of current limitation algorithm / PID current regulator" },
   I_CLIP_POS_PID_I={ num=135, desc="I-Clipping parameter of position PID regulator (first parameter set) (A too high value causes overshooting at positioning mode.) " },
   --- ...

   P_VEL_PID_I={ num=140, desc="P parameter of velocity PID regulator (first parameter set, used at lower speed)" },
   I_VEL_PID_I={ num=141, desc="I parameter of velocity PID regulator (first parameter set, used at lower speed)" },
   D_VEL_PID_I={ num=142, desc="D parameter of velocity PID regulator (first parameter set, used at lower speed)" },
   I_CLIP_VEL_PID_I={ num=143, desc="I-Clipping parameter of velocity PID (first parameter for parameter set, used at lower speed)" },

   ACTIVATE_RAMP={ num=146, desc="1: activate velocity ramp generator" },
   PID_TYPE={ num=147, desc="0: standard PID, 1: PID disabled, 2: Integrating PID algorith." },
   ACTUAL_CUR={ num=150, unit='mA', desc="Actual motor current"},
   ACTUAL_VOL={ num=151, desc="Actual supply voltage" },
   ACTUAL_TEMP={ num=152, desc="Actual temperature" },
   ACTUAL_PWM_CYCLE={num=153, desc="Get actual PWM duty cycle." },
   TARGET_PWM={ num=154, desc="Get desired target current or set target current to activate current regulation mode" },
   TARGET_CURRENT={ num=155, unit='mA', desc="Get desired target current or set target current to activate current regulation mode." },
   ERROR_STATUS={ num=156, desc="Error Status flag" },
   CLR_TC_TIMEOUT={ num=158, desc="Clear Ethercat timeout flag" },
   COMMUTATION_MODE={ num=159, desc="0: Block, 1: Sensorless Block (hallFX), 2: Sine comm w/ hall, 3: Sine comm w/ encoder, 4: controlled block comm, 5: Controlled Sine comm" },
   SINE_REINIT={ num=160, desc="0: still reinit'ing, 1: reinitalized" },

   --- [...]
   BLOCK_PWM_SCHEME={ num=167, desc="0: PWM chooper on high side, 1: PWM chopper on low side, 2: PWM chopper on low and high side" },
   P_CURR_PID_I={ num=168, desc="P parameter of current PID regulator (1)" },
   I_CURR_PID_I={ num=169, desc="I parameter of current PID regulator (1)" },
   D_CURR_PID_I={ num=170, desc="D parameter of current PID regulator (1)" },
   I_CLIP_CURR_PID_I={ num=171, desc="I Clipping parameter of current PID (1)" },

   P_CURR_PID_II={ num=172, desc="P parameter of current PID regulator (2)" },
   I_CURR_PID_II={ num=173, desc="I parameter of current PID regulator (2)" },
   D_CURR_PID_II={ num=174, desc="D parameter of current PID regulator (2)" },
   I_CLIP_CURR_PID_II={ num=175, desc="I Clipping parameter of current PID (2)" },
   THRES_CURR_PID={ num=176, unit='rpm', desc="Threshold speed for current regulation to switch between first and second current PID parameter set" },
   START_CURRENT={ num=177, unit='mA', desc="Motor current for controlled commutation. This parameter is used in commutation mode 1, 4, 5 and in initialization of sine." },
   ACT_ENCODER_POS={ num=209, desc="Actual encoder position / counter value" },
   GEAR_RATIO={ num=211, desc="Gear transmission ration" },

   --- ...

   P_POS_PID_II={ num=230, desc="P parameter of position PID regulator (2. PID (II) parameter set)" },
   I_POS_PID_II={ num=231, desc="I parameter of position PID regulator (2. PID (II) parameter set)" },
   D_POS_PID_II={ num=232, desc="D parameter of position PID regulator (2. PID (II) parameter set)" },
   I_CLIP_POS_PID_II={ num=233, desc="I-Clipping parameter of position PID regulator second parameter set) (A too high value causes overshooting at positioning mode.) " },

   --- Pos PID errors [...]
   P_VEL_PID_II={ num=234, desc="P parameter of velocity PID regulator (2. param set, used at higher speed)" },
   I_VEL_PID_II={ num=235, desc="I parameter of velocity PID regulator (2. param set, used at higher speed)" },
   D_VEL_PID_II={ num=236, desc="D parameter of velocity PID regulator (2. param set, used at higher speed)" },
   I_CLIP_VEL_PID_II={ num=237, desc="I-Clipping parameter of velocity PID (2. param set, used at higher speed)" },

   MASS_INERTIA_CONST={ num=238, desc="Mass inertia constant. Compensates the rotor inertia of the motor."},
   BEMF_CONST={num=239, unit="rpm/(10V)", desc="BEMF constant (used for current, position and velocity regulation"},
   MOTOR_COIL_RES={ num=240, unit="mOhm", desc="Resistance of motor coil"},

   INIT_SINE_SPEED={ num=241, unit='rpm', desc="Velocity for sine initialization"},
   INIT_SINE_BLOCK_OFF_CW={ num=242, desc="This parameter helps to tune hall sensor based initialization in a way, that the motor has the same velocity for left and right turn. It compensates for tolerance and hysteresis of the hall sensors. It is added to the Commutation offset upon CW turn initialization." },
   INIT_SINE_BLOCK_OFF_CCW={num=243, desc="This parameter helps to tune hall sensor based initialization in a way, that the motor has the same velocity for left and right turn.  It compensates for tolerance and hysteresis of the hall sensors. It is added to the Commutation offset upon CCW turn initialization." },
   INIT_SINE_DELAY={ num=244, unit='us', desc="Duration for sine initialization sequence. This parameter should be set in a way, that the motor has stopped mechanical oscillations after the specified time."},
   OVERVOLT_PROTECT={ num=245, desc="1: enable overvoltage protection" },
   MAX_PWM_CHANGE={ num=246, desc="Maximum PWM change per PID interval." },
   SINE_COMP_FACTOR={ num=247, desc="Compensates the propagation delay of the MPU" },
   INIT_SINE_MODE={ num=249, desc="0: Init in controlled sine comm, 1: Init in block comm using hall sensors" },
   -- [...]
}

--- Print all axis parameters of a given slave
-- @param slave slave number to print
-- @param nodesc don't print description
-- @param nocolor don't print using colors
function axis_param_print(slave, nodesc, nocolor)
   local pad = { param=24, val=10 }
   local res={}
   for p, pt in pairs(axis_param) do
      local unitstr = pt.unit or ""
      local descstr = ""
      if not nodesc then
	 descstr = ("// %s"):format(pt.desc)
	 descstr = utils.wrap(descstr, 80, " ", "")
	 descstr = string.gsub(descstr, "\n", "\n" ..string.rep(" ", pad.param+pad.val+5))
	 if not nocolor then descstr = ac.red(descstr) end
      end
      local valstr = "<READ ERR>"
      local paramstr = p
      local succ, val = gap(pt.num, slave)

      if succ then valstr = string.format("%d%s", val, unitstr) end
      valstr = utils.rpad(valstr, pad.val)

      if not nocolor then
	 paramstr = utils.rpad(paramstr, pad.param)
	 paramstr = ac.magenta(paramstr)
	 if valstr == "<READ ERR>" then
	    valstr = ac.bright(ac.red(valstr))
	 else
	    valstr = ac.blue(valstr)
	 end
      end
      res[#res+1] = { str=("%s= %s %s"):format(paramstr, valstr, descstr), num=pt.num }
   end
   -- restore the initial order
   table.sort(res, function (a,b) return a.num > b.num end)
   local str = ""
   for _,t in ipairs(res) do str = str .. t.str .. '\n' end
   print(str)
end


--- Get parameters for multiple axis
-- @param param string id of parameter
-- @param slaves table with slave numbers to return (default is only arm and base motors)
-- @return table with keys=slavenum, value is parameter.
function get_params(param, slaves)
   slaves = slaves or def_slaves
   local param_nr = axis_param[param].num
   local res = {}

   if not param_nr then return false end

   for _,s in ipairs(slaves) do
      err,res[s] = gap(param_nr, s)
   end
   return res
end


--- Pretty print a single parameter for a set of axis (or the default
-- axis if none given)
-- @param param string parameter to print
-- @param slaves list of slaves to print
function param_print(param, slaves)
   local params = get_params(param, slaves)
   local param_tab = axis_param[param]
   print("Parameter:   " .. param)
   print("Description: " .. param_tab.desc)

   if not params then print("failed!"); return false end

   for slid,val in pairs(params) do
      print(("   slave %d, value %s %s"):format(slid, tostring(val), param_tab.unit or "" ))
   end
end

--- Set parameters for multipe axis
-- @param params table with { paramA=value1, paramB=value2, ...} pairs
-- @param slaves table with slave numbers of axis to set (default is TMCL.def_slaves)
-- @param verbose if true then report what is being done.
-- @return true if suceeded, false otherwise.
function set_params(params, slaves, verbose)
   local function out(...)
      if verbose then print("set_params: ", ...) end
   end

   local ret = true
   slaves = slaves or def_slaves

   -- Check parameters
   for pname, val in pairs(params) do
      if not axis_param[pname] then
	 print("set_params: " .. tostring(pname) .. " is not a known parameter")
	 return false
      end
   end

   -- Set then
   for _,s in ipairs(slaves) do
      for pname, val in pairs(params) do
	 out("setting slave "..tostring(s)..", param "..tostring(pname).." to ".. tostring(val))
	 if not sap(axis_param[pname].num, s, val) then
	    print("set_params: failed to set " ..
		  tostring(pname) .. " to ".. tostring(val) .. " for slave " .. tostring(s))
	    ret = false
	 end
      end
   end
   return ret
end


--- Save parameters
--- Restore parameters
