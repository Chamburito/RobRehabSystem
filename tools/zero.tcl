#!/usr/bin/tclsh

# send zeros to each motor for 4 minutes, read encoder angles.

# Copyright 2003-2005 Interactive Motion Technologies, Inc.
# Cambridge, MA, USA
# http://www.interactive-motion.com
# All rights reserved

global ob

set ob(crobhome) $::env(CROB_HOME)

source $ob(crobhome)/shm.tcl

proc spin {motor volts} {
	if {![string match {[se]} $motor]} {
		return
	}
	if {$::run($motor)} {
		wshm raw_torque_volts_$motor $volts
	} else {
		wshm raw_torque_volts_$motor 0.0
	}
}

# start the Linux Kernel Modules, shared memory, and the control loop
start_lkm
start_shm
start_loop

# get the party started
wshm pfotest 2.0
wshm test_raw_torque 1
wshm no_safety_check 1

# sleep for .1 sec, gives robot chance to start
after 100

set ::run(s) 1
set ::run(e) 1

spin s 0.0
spin e 0.0

set date [clock format [clock seconds] -format %a.%H%M]
set logname zero.$date.dat

puts "zero test begin"
start_log $logname 6
after 240000 {set done 1}
vwait done
stop_log
after 1000
puts "zero test done"
stop_loop
stop_shm
stop_lkm

