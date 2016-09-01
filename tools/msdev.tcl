#! /usr/bin/tclsh
# read a 7-wide .asc file and calculate mean and stdev on each column.
# assume 1 is i, we don't need that.

# see: tcl wiki - additional math functions
# http://mini.net/tcl/819
#
proc getit {} {
    global dlist

    while {[gets stdin line] >= 0} {
	scan $line "%f %f %f %f %f %f %f" v(1) v(2) v(3) v(4) v(5) v(6) v(7)
	foreach i {2 3 4 5 6 7} {
	    lappend dlist($i) $v($i)
	}
    }

    foreach i {2 3 4 5 6 7} {
	    puts "mean $i [mean $dlist($i)]"
	    puts "stddev $i [stddev $dlist($i)]"
    }
}

proc mean  L {expr ([join $L +])/[llength $L].}
proc gmean L {expr pow([join $L *],1./[llength $L])}
proc qmean L {expr sqrt((pow([join $L ,2)+pow(],2))/[llength $L])}
proc hmean L {expr [llength $L]/(1./[join $L +1./])}

proc mean2 list {
    set sum 0
	foreach i $list {set sum [expr {$sum+$i*$i}]}
    expr {double($sum)/[llength $list]}
}
proc stddev list {
    set m [mean $list] ;# see below for [mean]
	expr {sqrt([mean2 $list]-$m*$m)}
}

getit
