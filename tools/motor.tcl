#!/usr/bin/wish

# apply absolute voltages to motors
# separate voltage for each motor by .5's -5.0 to 5.0

# Copyright 2003-2005 Interactive Motion Technologies, Inc.
# Cambridge, MA, USA
# http://www.interactive-motion.com
# All rights reserved

# (no voltage bias mul or add)
# 
# read Mz from FT
# 
# Apply each force for 60 seconds.
# 
# read sampling at 200 Hz

# commands that talk to robot's shared memory buffer

package require Tk

global ob

set ob(crobhome) $::env(CROB_HOME)

source $ob(crobhome)/shm.tcl

proc wrote_entry_var {vPtr win callback} {
	upvar #0 $vPtr v
	set v [$win get]
	namespace eval :: $callback
} 

proc spin {motor volts} {
	if {![string match {[se]} $motor]} {
		return
	}
        set volts [expr {$volts * $ob(voltsmult)}]
	if {$::run($motor)} {
		wshm raw_torque_volts_$motor $volts
	} else {
		wshm raw_torque_volts_$motor 0.0
	}
}

proc mzavg {} {
	set ::mz(i) 0
	set ::mz(avg) 0
	set ::mz(nsamples) 10
	every 1000 {
		set ::mz($::mz(i)) [rshm ft_zmoment]
		if {$::mz(i) > $::mz(nsamples)} {
			after cancel $::mzav_id
			for {set i 0} {$i < $::mz(nsamples)} {incr i} {
				set ::mz(avg) [expr $::mz(avg) + $::mz($i)]
			}
			puts "voltage $::volts mz [expr {$::mz(avg) / 10.0}]"
			puts $::mz(fd) "voltage $::volts mz [expr {$::mz(avg) / 10.0}]"
			return
		}
		# puts "$::mz(i) : voltage $::volts mz $::mz($::mz(i))"
		incr ::mz(i)
	} ::mzav_id
	
}

proc calib {} {
	start
	.state_l configure -text "calibrate"

	set ::calib_i 0
	set ::calib_list {
		-1.0 -.9 -.7 -.5 -.3 -.1 
		  .1  .3  .5  .7  .9 1.0
	}
	set ::len [llength $::calib_list]
	
	set mwhich ""
	set date [clock format [clock seconds] -format %a.%H%M]
	if {$::run(s)} {set mwhich ${mwhich}s}
	if {$::run(e)} {set mwhich ${mwhich}e}
	set clogname /tmp/mcal$mwhich.$date.dat
	set mzname /tmp/$mwhich.$date.mz

	start_log $clogname 6
	set ::mz(fd) [open $mzname w]
	every 60000 {
		set ::volts [lindex $::calib_list $::calib_i]

		spin s 0.0
		spin e 0.0
		after 1000 {
			spin s $::volts
			spin e $::volts
		}
		after 10000 mzavg
		.shoulder_l configure -text "shoulder V $::volts"
		.elbow_l configure -text "elbow V $::volts"
		incr ::calib_i
		if {$::calib_i > $::len} {
			after cancel $::calib_id
			close $::mz(fd)
			unset ::mz(fd)
			stop_log
			.state_l configure -text "calibrate done"
		}
	} ::calib_id
}

proc cycle {} {
	start
	.state_l configure -text "cycle"

	set ::cyclei 0
	set ::cycle_list {
		 .1  .3  .5  .7  .9  1.0
		 .9  .7  .3  .5  .1 0
		 -.1 -.3 -.5 -.7 -.9 -1.0
		 -.9 -.7 -.3 -.5 -.1 0
	}
	set ::len [llength $::cycle_list]

	every 100 {
		set i [expr [incr ::cyclei] % $::len]
		set ::volts [lindex $::cycle_list $i]

		spin s $::volts
		spin e $::volts
		.shoulder_l configure -text "sh V $::volts"
		.elbow_l configure -text "el V $::volts"
		
	} ::cycle_id
}

proc const {} {
	start
	.state_l configure -text "const"

	every 100 {
		spin s $::shoulder_var
		spin e $::elbow_var
		.shoulder_l configure -text "sh V $::shoulder_var"
		.elbow_l configure -text "el V $::elbow_var"
	} ::const_id
}

proc start {} {
	wshm test_raw_torque 1
	wshm no_safety_check 1
	wshm have_thermal_model 0
	start_loop
	after 100
	.state_l configure -text "running"
}

proc stop {} {
	if [info exists ::calib_id] {
		after cancel $::calib_id
	}
	if [info exists ::cycle_id] {
		after cancel $::cycle_id
	}
	if [info exists ::const_id] {
		after cancel $::const_id
	}
	set ::shoulder_var 0.0
	set ::elbow_var 0.0
	spin s $::shoulder_var
	spin e $::elbow_var
	stop_loop
	stop_log
	after 100
	.state_l configure -text "paused"
}

proc quit {} {
	stop_loop
	stop_log
	stop_shm
	stop_lkm
	after 100 exit
}

# start the Linux Kernel Modules, shared memory, and the control loop
start_lkm
start_shm
start_loop

# get the party started
# wshm pfotest 2.0
wshm test_raw_torque 1
wshm no_safety_check 1

set ob(pfovolts) 0.
set ob(voltsmult) 1.0
set ob(pfovolts) [rshm pfomax]
if {$ob(pfovolts) == 10.0} {set ob(voltsmult) 2.0}

# sleep for .1 sec, gives robot chance to start
after 100

button .start_b -text "start" -command start
button .cycle_b -text "cycle" -command cycle
button .calib_b -text "calib" -command calib
button .const_b -text "const" -command const
checkbutton .s_cb -text shoulder -variable ::run(s)
checkbutton .e_cb -text elbow -variable ::run(e)

set ::run(s) 1
set ::run(e) 1

label .state_l -text "running"

label .shoulder_l -text "shoulder V"
entry .shoulder_e -textvariable ::shoulder_var

bind .shoulder_e <Return> [list wrote_entry_var ::shoulder_var %W {
	spin s $::shoulder_var
	.shoulder_l configure -text "shoulder V $::shoulder_var"
}]

bind .shoulder_e <Tab> [list wrote_entry_var ::shoulder_var %W {
	spin s $::shoulder_var
	.shoulder_l configure -text "shoulder V $::shoulder_var"
}]

label .elbow_l -text "elbow V"
entry .elbow_e -textvariable ::elbow_var

bind .elbow_e <Return> [list wrote_entry_var ::elbow_var %W {
	spin e $::elbow_var
	.elbow_l configure -text "elbow V $::elbow_var"
}]

bind .elbow_e <Tab> [list wrote_entry_var ::elbow_var %W {
	spin e $::elbow_var
	.elbow_l configure -text "elbow V $::elbow_var"
}]

label .mz_l -text ""

button .stop_b -text "stop" -command stop
button .quit_b -text "quit" -command quit

pack .start_b .cycle_b .calib_b .const_b
pack .s_cb .e_cb
pack .state_l .shoulder_l .shoulder_e .elbow_l .elbow_e
pack .mz_l .stop_b .quit_b

every 50 {
	.mz_l configure -text "mz [rshm ft_zmoment]"
} ::mz_id
