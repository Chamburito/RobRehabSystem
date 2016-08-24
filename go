#! /bin/sh

# set -x

# go - load the linux kernel modules
# load calibration data from imt2.cal too

# InMotion2 robot system software for RTLinux

# Copyright 2003-2005 Interactive Motion Technologies, Inc.
# Cambridge, MA, USA
# http://www.interactive-motion.com
# All rights reserved

PATH=/usr/local/bin:/bin:/usr/bin:/usr/X11R6/bin:/sbin:/usr/sbin

if [ -z "$CROB_HOME" ]; then
        echo "CROB_HOME is not set, imt_config/imt.rc must be sourced"
	exit 1
fi

CROB=$CROB_HOME
cd $CROB

if ! ./checkexist; then
	exit 1
fi

$CROB/tools/plc check-plc
ret=$?
if [ $ret -ne 0 ]; then
        echo "go: PLC is not running, exiting."
        exit 1
fi

$CROB/tools/plc set-drive-en

# TODO: do we really want to keep loading/unloading the modules?
# if our modules are already loaded, unload them first.

lsmod | grep -E -q 'xeno_|pwrdaq|pci4e' && {
	echo "some modules already loaded, reloading."
	./stop || { 
		echo "couldn't unload, exiting."; exit 1
	}
}

if pkill -0 -x robot; then
    echo "old robot still running, stopping."
    ./stop || { 
	echo "couldn't stop, exiting."; exit 1
    }
fi

# flush disk buffers, could help in case something crashes.
sync;sync

# TODO: delete # echo inserting RTLinux modules...
# TODO: delete insmod /usr/src/linux-2.4.18-rtl3.1/rtlinux-3.1/drivers/mbuff/mbuff.o
# TODO: delete # echo inserting RTLinux modules...
# TODO: delete (cd /usr/src/linux/rtlinux; scripts/insrtl)
# TODO: delete ## echo inserting UEI powerDAQ module...
# TODO: delete #insmod /lib/modules/2.4.18-w4l-rtl3.1/misc/pwrdaq.o
# TODO: delete ## echo inserting IMT robot module...
# TODO: delete #insmod ./robot.o

./loadmodules.sh

# /proc/pwrdaq has been loaded, and should be present.

I=0
while [ ! -r /proc/pwrdaq ]; do
    if [ $I -gt 20 ]; then
	echo "go: could not load UEI PowerDAQ module."
	./stop
	exit 1
    fi
    I=`expr $I + 1`
    sleep 0.1
done

# ./robot &
# 
# I=0
# while ! pkill -0 -x robot ; do
#     if [ $I -gt 20 ]; then
# 	echo "go: could not start robot"
# 	./stop
# 	exit 1
#     fi
#     I=`expr $I + 1`
#     sleep 0.1
# done
# 
# sleep 0.1
# 
# ./shm < $IMT_CONFIG/robots/`cat $IMT_CONFIG/current_robot`/imt2.cal
# 
# sleep 0.1

exit 0
