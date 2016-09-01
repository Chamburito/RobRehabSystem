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
after 100
wshm no_safety_check 1
wshm stiff 100
wshm damp 5
wshm slot_max 4

proc mb_entry {f n} {
	global ob

	frame $f

	label $f.ltype -text "move type:"
	set ob(mb,type,$f) 0
	entry $f.etype -width 6 -textvariable ob(mb,type,$f)

	label $f.ltime -text "time: "
	set ob(mb,time,$f) 5.0
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
	set ob(mb,dest_y,$f) 0.0
	entry $f.edest_y -width 6 -textvariable ob(mb,dest_y,$f)

	label $f.ldest_w -text "w: "
	set ob(mb,dest_w,$f) 0.0
	entry $f.edest_w -width 6 -textvariable ob(mb,dest_w,$f)

	label $f.ldest_h -text "h: "
	set ob(mb,dest_h,$f) 0.0
	entry $f.edest_h -width 6 -textvariable ob(mb,dest_h,$f)

	button $f.osc -command [list osc $f $n] -text "oscillate"
	button $f.go -command [list mb_go $f $n] -text "go $n"
	button $f.bgo -command [list mb_bgo $f $n] -text "go back"
	button $f.show -command [list mb_show $f $n] -text "show"

	grid $f.ltype $f.etype $f.ltime $f.etime
	grid $f.lsrc_x $f.esrc_x $f.lsrc_y $f.esrc_y $f.lsrc_w $f.esrc_w $f.lsrc_h $f.esrc_h 
	grid $f.ldest_x $f.edest_x $f.ldest_y $f.edest_y $f.ldest_w $f.edest_w $f.ldest_h $f.edest_h 
	# grid with adjustments to spacing
	grid $f.go $f.bgo - $f.osc - - $f.show - -sticky w
}

proc mb_go {f n} {
	global ob
	set ticks [expr {int($ob(mb,time,$f) * 200.0)}]
	set type [slot_str $ob(mb,type,$f)]
	movebox $n $type {0 $ticks 1} \
		{$ob(mb,src_x,$f) $ob(mb,src_y,$f) $ob(mb,src_w,$f) $ob(mb,src_h,$f)} \
		{$ob(mb,dest_x,$f) $ob(mb,dest_y,$f) $ob(mb,dest_w,$f) $ob(mb,dest_h,$f)}
}

proc mb_bgo {f n} {
	global ob
	set ticks [expr {int($ob(mb,time,$f) * 200.0)}]
	set type [slot_str $ob(mb,type,$f)]
	movebox $n $type {0 $ticks 1} \
		{$ob(mb,dest_x,$f) $ob(mb,dest_y,$f) $ob(mb,dest_w,$f) $ob(mb,dest_h,$f)} \
		{$ob(mb,src_x,$f) $ob(mb,src_y,$f) $ob(mb,src_w,$f) $ob(mb,src_h,$f)}
}

proc osc {f n} {
	global ob
	set ms [expr {int($ob(mb,time,$f) * 1000)}]
	set ms2 [expr $ms * 2]
	mb_go $f $n
	after $ms mb_bgo $f $n

	after $ms2 osc $f $n
}

proc mb_show {f n} {
	global ob
	set ticks [expr {int($ob(mb,time,$f) * 200.0)}]
	set type [slot_str $ob(mb,type,$f)]
	puts "movebox $n $type {0 $ticks 1} \
		{$ob(mb,src_x,$f) $ob(mb,src_y,$f) $ob(mb,src_w,$f) $ob(mb,src_h,$f)} \
		{$ob(mb,dest_x,$f) $ob(mb,dest_y,$f) $ob(mb,dest_w,$f) $ob(mb,dest_h,$f)}"
}

proc center {} {
	center_arm
}

proc stopall {} {
    # stop oscillations
    foreach id [after info] {after cancel $id}
    stop_movebox
    stop_loop
    after 20
    foreach id [after info] {after cancel $id}
    start_loop
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
	"wrist" {set ret 7}
	"ankle" {set ret 8}
	"curl" {set ret 9}
	"wristcurl" {set ret 11}
	"wristps" {set ret 12}
	"linear" {set ret 16}
	"linearadap" {set ret 19}
	"wristadap" {set ret 20}
	"wristpsadap" {set ret 21}
	"hand" {set ret 22}
	}
	return $ret
}

mb_entry .mb1 0
mb_entry .mb2 1
button .center -command center -text "center"
button .stop -text stop -command stopall
button .quit -command quit -text quit

grid .mb1 - - -
grid .mb2 - - -
grid .center .stop .quit -
