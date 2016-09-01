#!/usr/bin/tclsh

# Copyright 2003-2005 Interactive Motion Technologies, Inc.
# Cambridge, MA, USA
# http://www.interactive-motion.com
# All rights reserved

# run a damping controller
# you can modify the damping with wshm damp 10.0 or whatever.
# or by hand with ./shm.

# if vex is already running, then just star/stop shm, not lkm or loop.

global ob

set ob(crobhome) $::env(CROB_HOME)

source $ob(crobhome)/shm.tcl

no_arm

# start the Linux Kernel Modules, shared memory, and the control loop
puts "loading linux kernel module"
if {[is_lkm_loaded]} {
        puts "lkm already loaded."
} else {
        start_lkm
}
start_shm

# handle window resize
proc chwin {c w h} {
	global ob
	set ob(can,w) $w
	set ob(can,h) $h
        set w2 [expr {$w / 2}]
        set h2 [expr {$h / 2}]
	set ob(can,w2) $w2
	set ob(can,h2) $h2
        # translate from 0,0 in upper left to 0,0 in center
        $c config -scrollregion [list -$w2 -$h2 $w2 $h2]
}

# display to robot world coords
proc d2w {x y} {
	global ob
	set cx [expr {($x - $ob(can,w2)) / $ob(scale)}]
	set cy [expr {-($y - $ob(can,h2)) / $ob(scale)}]
	return [y_up $cx $cy]
}

# robot world to display coords
proc w2d {cx cy} {
	global ob
	set x [expr {$cx * $ob(scale) + $ob(can,w2)}]
	set y [expr {$cy * $ob(scale) + $ob(can,h2)}]
	return [y_up $x $y]
}

proc d2d {args} {
    global ob
    set ret ""
    if {[llength $args]==1} {set args [lindex $args 0]}
    foreach {x y} $args {
	lappend ret [expr {$x - $ob(can,w2)}]
	lappend ret [expr {$y - $ob(can,h2)}]
    }
    return $ret
}

# new title
proc wtitle {x y} {
	foreach {cx cy} [d2w $x $y] break
	wm title . "($cx $cy) ($x $y)"
}

# handle button1
proc dob1 {w x y} {
	global ob
	if {![info exists ob(linestart)]} {
		# start a line
		set ob(linestart) 1
		set ob(line,x1) $x
		set ob(line,y1) $y
		$w create oval [eval centxy [d2d $x $y] 3] -tag start,$x,$y -width 3
		puts "start $x $y"
	} else {
		# end a line
		set x1 $ob(line,x1) 
		set y1 $ob(line,y1)
		set ob(line,x2) $x
		set ob(line,y2) $y
		unset ob(linestart)
		puts "line $x1,$y1 $x,$y"
	$w create line [d2d $x1 $y1 $x $y] -tag line,$x1,$y1 -fill blue -width 3
	}
}

set c [canvas .c -height 600 -width 600]
$c config -highlightthickness 0
chwin $c 600 600
pack $c
set ob(scale) 1000.0
$c scale all 0 0 $ob(scale) $ob(scale)

proc done {} {
	stop_loop
	stop_shm
	stop_lkm
	exit
}

proc init {c} {
	global ob
	# bind $c <Configure> [list after idle chwin $c %w %h]
	bind $c <Motion> {wtitle %x %y}
	bind $c <Button-1> {dob1 %W %x %y}

	bind . <Key-q> done

	# dampzone
	$c create rect [centxy 0.0 0.0 .2] -tag rect -outline red
	$c scale rect 0 0 $ob(scale) $ob(scale)
	# circle
	$c create oval [centxy 0.0 0.0 .14] -tag ring -outline green
	$c scale ring 0 0 $ob(scale) $ob(scale)
	# center
	$c create oval [centxy 0.0 0.0 .005] -tag center
	$c scale center 0 0 $ob(scale) $ob(scale)
}

init $c

