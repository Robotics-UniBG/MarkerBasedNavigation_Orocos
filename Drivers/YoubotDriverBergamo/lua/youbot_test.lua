--- Basic youbot deployment script.
-- This basic script starts up the youbot component, imports required
-- libraries and sets up a number of functions to support basic
-- operations. This script can be launched by other applications by
-- calling dofile("youbot_test.lua")

require "rttlib"	-- mandatory
require "rfsm_rtt"	-- rfsm rtt helper functions
require "rttros"	-- rospack_find
require "youbot_pp"	-- youbot types pretty printing
require "ansicolors"    -- colorful errors

--- Parameters
FUERTE_YOUBOT=true
ETHERCAT_IF='eth0'

rttlib.color=1
-- rtt.setLogLevel("Info")

--- helper, print a bright red errormeg
function errmsg(...)
   print(ansicolors.bright(ansicolors.red(table.concat({...}, ' '))))
end

-- get a handle to the current taskcontext
tc=rtt.getTC()

-- get a handle to the deployer component. This component permits
-- loading and instantiating other components, typekits, etc.
depl=tc:getPeer("deployer")

-- import the required packages
depl:import("youbot_driver_rtt")
depl:import("rtt_std_msgs")
depl:import("rtt_youbot_msgs")
depl:import("rtt_motion_control_msgs")

-- fire-up the Youbot component
depl:loadComponent("youbot", "youbot_driver::YoubotDriver")
yb=depl:getPeer("youbot")
yb:getProperty("ifname"):set(ETHERCAT_IF)

-- the youbot activity is running at 500Hz
depl:setActivity("youbot", 0.002, 50, rtt.globals.ORO_SCHED_RT)

-- Load the Lua service into the youbot. This essentially makes the
-- youbot component Lua scriptable. Used here to run the calibration
-- FSM in the youbot.
depl:loadService("youbot","Lua")

--- Configure the youbot component
if not yb:configure() then
   errmsg("Youbot configuration failed!")
   return -1
end

-- The youbot component is partitioned into one base service and one
-- or more Arm services (depending on the amount of arms added). The
-- following gets handles to these services.
arm=yb:provides("Arm1")
base=yb:provides("Base")

--- connect to base desired velocity
-- port_clone_conn creates an inverse and connected port to the given one.
base_cmd_vel = rttlib.port_clone_conn(base:getPort("cmd_twist"))

-- connect to arm desired velocity
arm_cmd_vel = rttlib.port_clone_conn(arm:getPort("joint_velocity_command"))
arm_vel=rtt.Variable("/motion_control_msgs/JointVelocities")
arm_vel.velocities:resize(5)
for axis = 0, arm_vel.velocities.size-1 do
   arm_vel.velocities[axis]=0.0
end

-- connect to arm desired torque
arm_cmd_torque = rttlib.port_clone_conn(arm:getPort("joint_effort_command"))
arm_torque = rtt.Variable("/motion_control_msgs/JointEfforts")
arm_torque.efforts:resize(5)
for axis = 0, arm_torque.efforts.size-1 do
   arm_torque.efforts[axis]=0.0
end

--- Some helper functions
function aset_cmode_vel() arm:setControlMode(2) end
function aset_cmode_cur() arm:setControlMode(rtt.Variable("uint8", 6)) end
function astop() arm:setControlMode(rtt.Variable("uint8", 0)) end

function bset_cmode_vel() base:setControlMode(2) end
function bstop() base:setControlMode(0) end

if not yb:start() then errmsg("Failed to start youbot") end

-- wait until arm and base are configured
timeout = 5 -- seconds
while true do
   local arm_calib = arm:configured()
   local base_calib = base:configured()
   if arm_calib and base_calib then break end

   if timeout <= 0 then
      if not arm_calib then print("Arm calibration failed.") end
      if not base_calib then print("Base calibration failed.") end
      break
   end
   io.stderr:write('.')
   rtt.sleep(1,0); timeout=timeout - 1
end

-- Launch calibration FSM.
-- This state machine is run in a service executing in the context of
-- the youbot component.
-- fsmfile = "/home/mk/src/stacks/youbot_hardware/youbot_driver_rtt/lua/fsm.lua"
fsmfile = rttros.find_rospack("youbot_driver_rtt") .. "/lua/calibration_fsm.lua"
execstr_op = yb:provides("Lua"):getOperation("exec_str")
execstr_op("FUERTE_YOUBOT="..tostring(FUERTE_YOUBOT))     -- propagate Youbot version to calibration FSM
rfsm_rtt.service_launch_rfsm(fsmfile, execstr_op, true)
calib_event = rttlib.port_clone_conn(yb:getPort("cal_events"))

arm_cmd_vel:write(arm_vel)
aset_cmode_vel()

function load_tmcl()
   require "TMCL"
   TMCL.set_sendMBX(yb:getOperation("sendMBX"))
   TMCL.set_sendMBXGripper(yb:getOperation("sendMBXGripper"))
end

--- Print arm status information.
function ainf() rttlib.portstats(arm) end

--- Print base status information
function binf() rttlib.portstats(base) end

--- Print driver status information
function dinf() rttlib.portstats(yb) end

--- Move arm in velocity mode for a certain time.
-- @param axis axis number (0-5)
-- @param dur duration to move in seconds.
-- @param vel desired velocity in rad/s
function amove(axis, dur, vel)

   local function set_vel(axis, vel)
      if axis<=5 or axis > 0 then arm_vel.velocities[axis]=vel
      else error("unknown axis nr ".. tostring(axis)) end
   end

   local function set_vels(axis, vels)
      if not type(vel)=='table' then error("if axis is table then vel must be too") end
      if not #vel==#axis then error("size of table axis not equal size of table vel") end
      for i,ax in ipairs(axis) do set_vel(ax, vels[i]) end
   end

   if type(axis) == 'number' then set_vel(axis, vel)
   elseif type(axis) == 'table' then set_vels(axis, vel) end

   local starts, startns = rtt.getTime()
   local endtime = starts + dur
   arm_cmd_vel:write(arm_vel)

   while true do
      local ts,tns = rtt.getTime()
      if ts>=endtime and tns >= startns then break end
   end

   set_vels({0,1,2,3,4}, {0,0,0,0,0})
   arm_cmd_vel:write(arm_vel)
end

--- Move arm in velocity mode for a certain time.
-- @param axis axis number (0-5)
-- @param dur duration to move in seconds.
-- @param vel desired velocity in rad/s
function aforce(axis, torque)
   if axis<=5 or axis > 0 then
      arm_torque.efforts[axis]=torque
   else
      error("unknown axis nr ".. tostring(axis))
   end
   arm_cmd_torque:write(arm_torque)
end

--- Move base in velocity mode.
-- @param dir direction ('th', 'x', 'y')
-- @param dur duration to move in seconds
-- @param vel desired velocity [m/s]
function bmove(dir, dur, vel)
   t=rtt.Variable("/geometry_msgs/Twist")

   if dir=='x' then t.linear.x=vel
   elseif dir=='y' then t.linear.y=vel
   elseif dir=='th' then t.angular.z=vel
   else error("unknown direction " .. tostring(dir)); end

   local starts, startns = rtt.getTime()
   local endtime = starts + dur
   base_cmd_vel:write(t)

   while true do
      base_cmd_vel:write(t)
      local ts,tns = rtt.getTime()
      if ts>=endtime and tns >= startns then break end
   end

   t.linear.x=0; t.linear.y=0; t.angular.z=0;
   base_cmd_vel:write(t)
end

--- shake ya legs to see if healthy
function test()
   local avel=0.03
   local bvel=0.1
   aset_cmode_vel()
   bset_cmode_vel()
   bmove('th', 2, bvel)
   bmove('th', 2, -bvel)
   amove({0,1,2,3,4}, 2, {avel, avel, avel, avel, avel})
   amove({0,1,2,3,4}, 2, {-avel, -avel, -avel, -avel, -avel})
end


--- Start the calibration state machine.
-- This is done by sending it the 'e_start_calib' event.
function start_calib() calib_event:write("e_start_calib") end

function youbot_test_help()
   print [=[
      ainf()			print status information about arm
      binf()			print status information about base
      dinf()                    print status information about driver in general
      amove(axis, t, vel)	move axis [0..5] for t seconds with velocity vel.
      bmove(dir, t, vel)	move base in dir ('th', 'x', 'y') with for t seconds with vel [m/s]
      aset_cmode_vel()	 	set arm ControlMode to velocity
      aset_cmode_cur()		set arm ControlMode to current
      astop		     	stop arm motors.
      bset_cmode_vel()          set base ControlMode to velocity
      bstop(f        )          stop base motors
      start_calib()		start arm calibration.
 ]=]
end

print('Please run "youbot_test_help()" for information on available functions')
