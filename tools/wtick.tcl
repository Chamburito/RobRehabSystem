#! /usr/bin/tclsh

# wtick var1 var2 ...
# a simple texty version of display

# for each var, print its value once per second.

# (run "go", then run this script)

# this code is async/event-driven, wait in vwait

set ob(crobhome) $::env(CROB_HOME)

source $ob(crobhome)/shm.tcl

start_shm

set done 0
fileevent stdin readable {set ::done 1}
set i 0

proc loop1sec {} {
	global i argv

	incr i
	puts -nonewline $i:
	foreach a $argv {
		set v [rshm $a]
		puts -nonewline "  $a $v"
	}
	puts ""
	after 1000 loop1sec
}

loop1sec

# set up Tcl event loop
vwait done

puts "done"

stop_shm
exit
