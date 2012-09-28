--- Calibration FSM.
-- Basic operation: move into limits, set this as the reference
-- position and then move to zero position (home).

require "rttlib"
require "rfsm_rtt"
require "rfsm_timeevent"

--- Parameters
-- Axis limit is reached when msr currents are higher than
-- current_high:
local current_high
if FUERTE_YOUBOT then
   current_high = {[0]=1000,1500,1000,500,400} --mA
else
   current_high = {[0]=1000,1500,1000,300,400} --mA
end

--- Reference position.
local ref_pos = {[0]=2.8793,1.1414,2.50552,1.76662,2.8767}

--- velocities with which we move into the limit and back again.
local move_in_vel = 0.005
local move_out_vel = -0.01


--- Setup starts here
local arm = rtt.getTC():provides("Arm1")
local cmd_vel = rttlib.port_clone_conn(arm:getPort("joint_velocity_command"))
local cmd_pos = rttlib.port_clone_conn(arm:getPort("joint_position_command"))
local joint_state_port = rttlib.port_clone_conn(arm:getPort("jointstate"))

local vel=rtt.Variable("/motion_control_msgs/JointVelocities")
local pos=rtt.Variable("/motion_control_msgs/JointPositions")
local joint_state=rtt.Variable("/sensor_msgs/JointState")
vel.velocities:resize(5)
pos.positions:resize(5)
joint_state.position:resize(5)
joint_state.velocity:resize(5)
joint_state.effort:resize(5)
local motor_states_port = rttlib.port_clone_conn(arm:getPort("motor_states"))
local motor_states=rtt.Variable("/youbot_msgs/motor_states")
motor_states:resize(5)
local motion_done = {}
local events_in = rtt.InputPort("string")
rtt.getTC():addPort(events_in,"cal_events","event port")

--- configure time triggered events
rfsm_timeevent.set_gettime_hook(rtt.getTime)

local function set_cmode_vel()
   arm:setControlMode(2)
end

local function set_reference_pos()
   for axis = 0, pos.positions.size-1 do
      pos.positions[axis]=ref_pos[axis]
   end
   cmd_pos:write(pos)
   arm:setControlMode(4)
end

return rfsm.csta{
   getevents = rfsm_rtt.gen_read_events(events_in),

   -- Empty state. We wait here until receiving the start calibration
   -- event.
   idle = rfsm.sista{ },
   

   -- Move all joints into the limits.
   -- In the doo check for high currents and stop the respective axis.
   -- Done when all axis have reached the limit.
   move_in = rfsm.sista{
      entry = function()
		 rtt.logl("Info","Start move in")
		 set_cmode_vel()
		 for axis = 0, vel.velocities.size-1 do
		    vel.velocities[axis]=move_in_vel
		    motion_done[axis]=false
		 end
		 cmd_vel:write(vel)
	      end,
      doo = function()
	       while true do 
		  local fs = motor_states_port:read(motor_states)
		  fs = joint_state_port:read(joint_state)
		  local ret = true
		  for axis = 0, vel.velocities.size-1 do
		     if not motion_done[axis] and motor_states.motor[axis].current>current_high[axis] then
			vel.velocities[axis]=0
			motion_done[axis]=true
			rtt.logl("Info","Axis ",axis," finished: ",joint_state.position[axis])
		     end
		     ret = ret and motion_done[axis]
		  end
		  cmd_vel:write(vel)
		  if ret then break end
		  rfsm.yield(true)
	       end
	    end,
   },

   --- Set this position as the reference position.
   set_ref_pos = rfsm.sista{
      entry = function()
		 set_reference_pos()
	      end,
   },
   
   --- Move to zero (home)
   -- In doo, stop axis if zero position reached.
   move_zero = rfsm.sista{
      entry = function()
		 rtt.logl("Info","Move close to zero")
		 set_cmode_vel()
		 for axis = 0, vel.velocities.size-1 do
		    vel.velocities[axis]=move_out_vel
		    motion_done[axis]=false
		 end
		 cmd_vel:write(vel)
	      end,
      doo = function()
	       while true do 
		  local fs = joint_state_port:read(joint_state)
		  local ret = true
		  for axis = 0, vel.velocities.size-1 do
		     if not motion_done[axis] and joint_state.position[axis]<0.0 then
			vel.velocities[axis]=0
			motion_done[axis]=true
			rtt.logl("Info","Axis ",axis," finished: ",joint_state.position[axis])
		     end
		     ret = ret and motion_done[axis]
		  end
		  cmd_vel:write(vel)
		  if ret then break end
		  rfsm.yield(true)
	       end
	    end,
   },

   rfsm.trans{ src="initial", tgt="idle"},
   rfsm.trans{ src="idle", tgt="move_in",events={"e_start_calib"}},
   rfsm.trans{ src="move_in", tgt="set_ref_pos", events={"e_done"} },
   rfsm.trans{ src="set_ref_pos", tgt="move_zero", events={"e_done"} },
   rfsm.trans{ src="move_zero", tgt="idle", events={"e_done"} },

}
