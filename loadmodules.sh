#!/bin/sh

# make sure the /dev/rtp files from the previous run are gone,
# in case we are iterating very quickly

I=1
while [ -e /dev/rtp31 ]; do
      sleep 0.1
      if [ $I -gt 20 ]; then
          break
      fi
      I=`expr $I + 1`
done

# set -x
sudo insmod /usr/realtime/modules/xeno_hal.ko
sudo insmod /usr/realtime/modules/xeno_nucleus.ko
sudo insmod /usr/realtime/modules/xeno_native.ko
sudo insmod /usr/realtime/modules/xeno_rtdm.ko

sudo insmod /lib/modules/`uname -r`/kernel/drivers/misc/pwrdaq.ko > /dev/null 2>&1 || {
    sudo insmod /lib/modules/`uname -r`/kernel/drivers/misc/pwrdaq_dummy.ko
}
cd /opt/imt/robot/pci4e 
./pci4e_load vendor_id=0x1892 device_id=0x5747

# make sure the /dev/rtp files exist before running chmod.

I=1
while [ ! -e /dev/rtp31 ]; do
      sleep 0.1
      if [ $I -gt 20 ]; then
          break
      fi
      I=`expr $I + 1`
done

sudo chmod 666 /dev/rtp*
