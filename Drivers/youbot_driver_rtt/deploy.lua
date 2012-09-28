t=rtt:getTC()
d=t:getPeer("deployer")

d:import("youbot_driver_rtt")

SCHED_RT=1

-- Set up youbot component
d:loadComponent("youbot","youbot_driver::YoubotDriver")
youbot=d:getPeer("youbot")
d:setActivity("youbot", 0.001, 99, SCHED_RT)

-- Set up timer component
d:loadComponent("timer","OCL::TimerComponent")
timer=d:getPeer("timer")
d:setActivity("timer", 0.01, 99, SCHED_RT)
d:addPeer("youbot","timer")

-- Set youbot Properties
youbot:getProperty("ifname"):set("eth15")

-- Create connections
cp=rtt.Variable("ConnPolicy")
d:connect("youbot.watchdog","timer.timeout",cp)
cp.transport = 3
cp.name_id = "odometry"
d:stream("youbot.odometry",cp)
cp.name_id = "cmd_vel"
d:stream("youbot.cmd_twist",cp)

-- Configuring components
timer:configure()
youbot:configure()

-- Starting components
timer:start()
youbot:start()

-- Set up timers
timer:startTimer(youbot:getProperty("state_timer"):get(),0.1)
timer:startTimer(youbot:getProperty("command_timer"):get(),10)
