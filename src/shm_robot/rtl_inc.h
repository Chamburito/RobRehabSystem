#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/io.h>
#include <sys/mman.h>
//#include <rtai/task.h>
//#include <rtai/queue.h>
//#include <rtai/intr.h>
#include <native/task.h>
#include <native/queue.h>
#include <native/intr.h>
#include <native/pipe.h>

#include <sys/mman.h>

#include <sys/ipc.h>
#include <sys/shm.h>

#include <fcntl.h>

// linux kernel mod includes

// InMotion2 robot system software for RTLinux

// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

// for multi-file.
//#define __NO_VERSION__

//#include <linux/module.h>
//#include <linux/kernel.h>
//#include <linux/version.h>
//#include <linux/sched.h>

// rtlinux includes
//#include <rtl_sched.h>  // for threading funcs
//#include <rtl_fifo.h>

//#include <mbuff.h>

