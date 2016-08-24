#!/bin/sh

# set -x
lsmod | grep -q pwrdaq_dummy && sudo rmmod /lib/modules/`uname -r`/kernel/drivers/misc/pwrdaq_dummy.ko
lsmod | grep -q pwrdaq && sudo rmmod /lib/modules/`uname -r`/kernel/drivers/misc/pwrdaq.ko
sudo rmmod /usr/realtime/modules/xeno_rtdm.ko
sudo rmmod /usr/realtime/modules/xeno_native.ko
sudo rmmod /usr/realtime/modules/xeno_nucleus.ko
sudo rmmod /usr/realtime/modules/xeno_hal.ko
cd /opt/imt/robot/pci4e
./pci4e_unload
