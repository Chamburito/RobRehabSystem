#!/bin/sh

# gnuplot run in a shell script "here document,
# to give it string variables.
# creates a tmp file dir which is deleted on exit

# Copyright 2003-2005 Interactive Motion Technologies, Inc.
# Cambridge, MA, USA
# http://www.interactive-motion.com
# All rights reserved

file=none
CROB=$CROB_HOME

case $# in
	1)
		file=$1
		;;
	2)
		file=$1
		using=$2
		;;
	*)
		echo usage: $0 filename [cols]
		exit 1
		;;
esac

if [ ! -r $file ] ; then
	echo no such file $file
	exit 1
fi


TMP=/tmp/gp$$
# don't use the variable, that could get munched somehow.
trap "rm -rf /tmp/gp$$" EXIT
mkdir -p $TMP

case $file in
	*.dat)
		$CROB/ta.tcl $file > $TMP/asc
		asc=$TMP/asc
		;;
	*.asc)
		asc=$file
		;;
	*.dat.gz)
		gunzip -c $file > $TMP/dat
		$CROB/ta.tcl $TMP/dat > $TMP/asc
		asc=$TMP/asc
		;;
	*.asc.gz)
		gunzip -c $file > $TMP/asc
		asc=$TMP/asc
		;;
	*)
		echo bad file type $file
		exit
		;;
esac

# remove tmp file on exit
# seems we must trap INT for exit to be called, at least on QNX
# trap "echo exiting gnuplot shell script; rm -f $tmp.*" EXIT
# trap "rm -f $tmp.*" EXIT
# trap true INT

# some variables for the script
# designate the seven column titles and their positions

txp="Time (0)"
xp=1
txp="X Position (1)"
xp=2
typ="Y Position (2)"
yp=3
txv="X Velocity (3)"
xv=4
tyv="Y Velocity (4)"
yv=5
txf="X Force (5)"
xf=6
tyf="Y Force (6)"
yf=7

############### gnuplot script starts here

# replace %f with ascii name

plotcmd=${2:-"plot \"$asc\" using 2:3 with lines"}
plotcmd=`echo $plotcmd | sed -e s@%f@$asc@g`

cat > $TMP/sh << END_OF_GP_SCRIPT
# tmp file created by gnuplot sh script

set size square
set xzeroaxis
set yzeroaxis
set grid

$plotcmd

END_OF_GP_SCRIPT

############### gnuplot script ends here

gnuplot -persist $TMP/sh &
# make sure gnuplot starts before tmp files disappear.
sleep 3
