#!/usr/bin/env rttlua-gnulinux

require "rttlib"
require "TMCL"
require "utils"

local opttab=utils.proc_args(arg)

if opttab['-h'] then
   print( [=[
Youbot param tool v1.0
Usage:
  dump_param.lua <option>
	-s <number>	print slave nr <number>. If not given all slaves are printed
	-nc 		don't use colors
	-nd		omit parameter description
	-h		print this help
	  ]=] )
   os.exit(0)
end

local nocolor = false
local nodesc = false

rttlib.color=1
tc=rtt.getTC()
depl=tc:getPeer("deployer")
depl:import("youbot_driver_rtt")

rtt.setLogLevel("Error")
depl:loadComponent("youbot", "youbot_driver::YoubotDriver")
yb=depl:getPeer("youbot")
yb:getProperty("ifname"):set("eth0")

local res = yb:configure()
print("configured youbot: ", res)

TMCL.set_sendMBX(yb:getOperation("sendMBX"))

if opttab['-nc'] then nocolor = true end
if opttab['-nd'] then nodesc = true end

if opttab['-s'] then
   local slavenr = tonumber(opttab['-s'][1])
   print(string.rep("-", 80))
   print("Dumping slave #", slavenr)
   TMCL.axis_param_print(slavenr, nodesc, nocolor)
else
   -- print all
   for _,i in ipairs(TMCL.def_slaves) do
      print(string.rep("-", 80))
      print("Dumping slave #", i)
      TMCL.axis_param_print(i, nodesc, nocolor)
   end
end
