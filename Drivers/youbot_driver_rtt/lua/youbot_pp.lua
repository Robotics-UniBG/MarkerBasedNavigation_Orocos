--- some basic pretty printing functions

local utils = require("utils")
local rttlib = require("rttlib")

local tostring = tostring
local ipairs = ipairs

module "youbot_pp"

local pad=6
local fmt='%g'

-- Convert number to string and pad with whitespace.
function padn(n) return utils.rpad((fmt):format(n), pad, ' ') end

function errcnt_tostr(ec)
   local function __tostr(str, cnt)
      if cnt > 0 then
	 return str..("=%s"):format(padn(cnt))
      end
      return ""
   end

   return __tostr("overcurr", ec.overcurrent) ..
	  __tostr("undervolt", ec.undervoltage) ..
	  __tostr("overvolt", ec.overvoltage) ..
	  __tostr("overtemp", ec.overtemp) ..
	  __tostr("hall_err", ec.hall_err) ..
	  __tostr("encoder_err", ec.encoder_err) ..
	  __tostr("sine_comm_init_err", ec.sine_comm_init_err) ..
	  __tostr("emergency_stop", ec.emergency_stop) ..
	  __tostr("ec_timeout", ec.ec_timeout) ..
	  __tostr("i2t_exceeded", ec.i2t_exceeded)
end

function motor_state_tostr(ms)
   return ("pos=%s vel=%s curr=%s errflag=%x %s"):format(padn(ms.position),
							 padn(ms.velocity),
							 padn(ms.current),
							 ms.error_flags,
							 errcnt_tostr(ms.counters))
end

function motor_states_tostr(mss)
   local res = ""
   local t = rttlib.var2tab(mss)
   for i,m in ipairs(t.motor) do
      res = res .. "Axis " .. tostring(i) .. ": " .. motor_state_tostr(m) .. "\n"
   end
   return res
end

function jointState_tostr(js)
   local t = rttlib.var2tab(js)
   local res = ""
   res = res .. "header:   " ..  utils.tab2str(t.header) .. "\n"
   res = res .. "names:    " .. utils.tab2str(t.name) .. "\n"
   res = res .. "position: " .. utils.tab2str(t.position) .. "\n"
   res = res .. "velocity: " .. utils.tab2str(t.velocity) .. "\n"
   res = res .. "effort:   " .. utils.tab2str(t.effort) .. "\n"
   return res
end

rttlib.var_pp["/youbot_msgs/motor_state"] = motor_state_tostr
rttlib.var_pp["/youbot_msgs/motor_states"] = motor_states_tostr
rttlib.var_pp["/sensor_msgs/JointState"] = jointState_tostr
