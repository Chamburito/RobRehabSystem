#!/usr/bin/wish

# Copyright 2005 Interactive Motion Technologies, Inc.
# Cambridge, MA, USA
# http://www.interactive-motion.com
# All rights reserved

# mvbox - form entry to set up a moving box slot controller.

package require Tk

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
start_loop

proc mb_entry {f} {
	global ob

	frame $f

	label $f.ltype -text "move type:"
	set ob(mb,type,$f) 0
	entry $f.etype -width 6 -textvariable ob(mb,type,$f)

	label $f.ltime -text "time: "
	set ob(mb,time,$f) 5
	entry $f.etime -width 6 -textvariable ob(mb,time,$f)

	label $f.lsrc_x -width 6 -text "src x: "
	set ob(mb,src_x,$f) 0.0
	entry $f.esrc_x -width 6 -textvariable ob(mb,src_x,$f)
	label $f.lsrc_y -text "y: "
	set ob(mb,src_y,$f) 0.0
	entry $f.esrc_y -width 6 -textvariable ob(mb,src_y,$f)
	label $f.lsrc_w -text "w: "
	set ob(mb,src_w,$f) 0.0
	entry $f.esrc_w -width 6 -textvariable ob(mb,src_w,$f)
	label $f.lsrc_h -text "h: "
	set ob(mb,src_h,$f) 0.0
	entry $f.esrc_h -width 6 -textvariable ob(mb,src_h,$f)

	label $f.ldest_x -text "dest x: "
	set ob(mb,dest_x,$f) 0.05
	entry $f.edest_x -width 6 -textvariable ob(mb,dest_x,$f)
	label $f.ldest_y -text "y: "
	set ob(mb,dest_y,$f) 0.1
	entry $f.edest_y -width 6 -textvariable ob(mb,dest_y,$f)
	label $f.ldest_w -text "w: "
	set ob(mb,dest_w,$f) 0.0
	entry $f.edest_w -width 6 -textvariable ob(mb,dest_w,$f)
	label $f.ldest_h -text "h: "
	set ob(mb,dest_h,$f) 0.0
	entry $f.edest_h -width 6 -textvariable ob(mb,dest_h,$f)

	button $f.go -command [list mb_go $f] -text "go"
	button $f.bgo -command [list mb_bgo $f] -text "go back"
	button $f.show -command [list mb_show $f] -text "show"

	grid $f.ltype $f.etype $f.ltime $f.etime
	grid $f.lsrc_x $f.esrc_x $f.lsrc_y $f.esrc_y $f.lsrc_w $f.esrc_w $f.lsrc_h $f.esrc_h 
	grid $f.ldest_x $f.edest_x $f.ldest_y $f.edest_y $f.ldest_w $f.edest_w $f.ldest_h $f.edest_h 
	grid $f.go $f.bgo $f.show
}

proc mb_go {f} {
	global ob
	set ticks [expr {int($ob(mb,time,$f) * 200.0)}]
	set type [slot_str $ob(mb,type,$f)]
	movebox 0 $type {0 $ticks 1} \
		{$ob(mb,src_x,$f) $ob(mb,src_y,$f) $ob(mb,src_w,$f) $ob(mb,src_h,$f)} \
		{$ob(mb,dest_x,$f) $ob(mb,dest_y,$f) $ob(mb,dest_w,$f) $ob(mb,dest_h,$f)}
}

proc mb_bgo {f} {
	global ob
	set ticks [expr {int($ob(mb,time,$f) * 200.0)}]
	set type [slot_str $ob(mb,type,$f)]
	movebox 0 $type {0 $ticks 1} \
		{$ob(mb,dest_x,$f) $ob(mb,dest_y,$f) $ob(mb,dest_w,$f) $ob(mb,dest_h,$f)} \
		{$ob(mb,src_x,$f) $ob(mb,src_y,$f) $ob(mb,src_w,$f) $ob(mb,src_h,$f)}
}

proc mb_show {f} {
	global ob
	set ticks [expr {int($ob(mb,time,$f) * 200.0)}]
	set type [slot_str $ob(mb,type,$f)]
	puts "movebox 0 $type {0 $ticks 1} \
		{$ob(mb,src_x,$f) $ob(mb,src_y,$f) $ob(mb,src_w,$f) $ob(mb,src_h,$f)} \
		{$ob(mb,dest_x,$f) $ob(mb,dest_y,$f) $ob(mb,dest_w,$f) $ob(mb,dest_h,$f)}"
}

proc center {} {
	center_arm
}

proc stopall {} {
	stop_movebox
}

proc quit {} {
	stop_rtl
	exit
}

proc slot_str {str} {
	set ret 0
	if {[string is integer $str] && \
	    ($str >= 0) && ($str < 32)} {
		return $str
	}
	switch $str {
	"simple" {set ret 0}
	"point" {set ret 2}
	"damp" {set ret 3}
	"rotate" {set ret 4}
	"adap" {set ret 5}
	"sine" {set ret 6}
	"curl" {set ret 9}
	}
	return $ret
}

mb_entry .mb1
mb_entry .mb2
button .center -command center -text "center"
button .stopall -command stopall -text "stop all"
button .quit -command quit -text quit

grid .mb1 - -
grid .mb2 - -
grid .center .stopall .quit

