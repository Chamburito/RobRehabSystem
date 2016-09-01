#! /usr/bin/tclsh

# example print a line in a timing loop controlled by ntickfifo.
# /dev/rtf5 is the tick fifo, it gets written every ntickfifo samples.
# if you are sampling at 200Hz, set ntickfifo to 200 to print once
# per second.

# (for this demo, run "go", then run this script,
# then use shm to adjust ntickfifo.)

# use fileevent to notify got_a_tick that the tickfifo has been written.

# note that this code is async/event-driven,
# we wait in vwait, not in gets.

set ob(crobhome) $::env(CROB_HOME)

source $ob(crobhome)/shm.tcl

puts "start kernel module"
start_rtl

wshm ntickfifo 200

global tickfd i

set tickfd [open "/proc/xenomai/registry/pipes/crob_tick" r]
set i 0

# set up the event

fileevent $tickfd readable got_a_tick

set done 0
fileevent stdin readable {set ::done 1}

# call this when we get a tick, 200x/sec.
# actually prints 1x/sec.

proc got_a_tick {} {
	global tickfd i

	incr i
	gets $tickfd x
	puts "$i sec, [rshm time_since_start] nsec"
}

# set up Tcl event loop
vwait done

close $tickfd
puts "stop kernel module"
stop_rtl
exit

