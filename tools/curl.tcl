#!/usr/bin/tclsh

# Copyright 2003-2005 Interactive Motion Technologies, Inc.
# Cambridge, MA, USA
# http://www.interactive-motion.com
# All rights reserved

# run a curl controller
# you can modify the curl constant with wshm curl 10.0 or whatever.
# or by hand with ./shm.

# if vex is already running, then just star/stop shm, not lkm or loop.

global ob

set ob(crobhome) $::env(CROB_HOME)

source $ob(crobhome)/shm.tcl

# start the Linux Kernel Modules, shared memory, and the control loop
puts "loading linux kernel module"
if {[is_lkm_loaded]} {
	puts "lkm already loaded."
} else {
	start_lkm
}
start_shm

set curl 20.0
set damp 10.0

puts "Curl controller"
puts "(+curl rotates cw, -curl rotates ccw)"
puts "curl = $curl, hit enter to start"
gets stdin

wshm curl $curl
wshm damp $damp


start_loop

# sleep for .1 sec, gives robot chance to start
after 100

# movebox to move a box slot
# e.g.: movebox 0 0 {0 1000 1} {0.0 0.0 0.005 0.005} {0.15 0.15 0.005 0.005}

movebox 0 9 {0 1 0} {0.0 0.0 0.0 0.0} {0.0 0.0 0.0 0.0}

puts "hit enter to stop"
gets stdin a

puts "unloading linux kernel module"

stop_loop
stop_shm
stop_lkm
