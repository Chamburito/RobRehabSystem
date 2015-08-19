//===========================================================================
//
// NAME:    powerdaq_kernel.h
//
// DESCRIPTION:
//
//          PowerDAQ Linux driver
//          
//          This file includes the headers required to compile the PowerDAQ
//          driver for various versions of the Linux kernel as well as
//          real-time extensions for Linux.
//          This file also defines a few global variables shared by the driver
//          source files.
//
//
// AUTHOR: Frederic Villeneuve 
//
//---------------------------------------------------------------------------
//      Copyright (C) 2005 United Electronic Industries, Inc.
//      All rights reserved.
//---------------------------------------------------------------------------



// When compiling for RTLinuxPro (only 2.1 or 2.2 supported)
// use RTLinuxPro headers
#ifdef _PD_RTLPRO
   #define __RTCORE_POLLUTED_APP__
   #define __KERNEL__
   
   #include <linux/pci.h>
   #include <linux/proc_fs.h>

   #include <rtl_pthread.h>
   #include <rtl_semaphore.h>
   #include <rtl_sched.h>
   #include <rtl_time.h>
   #include <rtl_profile.h>
   #include <rtl_core.h>
   #include <rtl_sync.h>
   #include <rtl_posixio.h>
   #include <sys/rtl_ioctl.h>
   #include <rtl_unistd.h>
   #include <rtl_stdio.h>
   #include <sys/rtl_stat.h>
   #include <sys/rtl_types.h>
   #include <rtl_fcntl.h>
   #include <rtl/rtl_string.h>
   #include <rtl/rtl_stdlib.h>
#else
   // For Linux, RTAI and Free RTLinux, we use regular kernel headers
   // in addition to some specific RTAI or RTLinux free headers
   #include <linux/version.h>
   #if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
      #include <generated/autoconf.h>
   #else
      #include <linux/autoconf.h>
   #endif
   #include <linux/kernel.h>
   #include <linux/module.h>
   #include <linux/types.h>
   #include <linux/sched.h>
   #include <linux/interrupt.h>
   #include <linux/init.h>
   #include <linux/pci.h>
   #include <linux/errno.h>
   #include <linux/mm.h>
   #include <linux/vmalloc.h>
   
   #if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 4, 0)
      #include <linux/slab.h>
   #else
      #include <linux/malloc.h>
   #endif
   
   #include <linux/delay.h>
   
   #if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 5, 0)
      #include <linux/moduleparam.h>
      #include <linux/device.h>
      #include <linux/workqueue.h>
   #else
      #include <linux/tqueue.h>
   #endif
   
   #include <linux/proc_fs.h>
   #include <linux/fs.h>
   
   #include <asm/io.h>
   #if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
       #include <asm/system.h>
   #endif
   #include <asm/uaccess.h>
   #include <asm/page.h>


   #ifdef _PD_RTL
      #include <rtl_mutex.h>
      #include <rtl_spinlock.h>
      #include <pthread.h>
      #include <time.h>
   #endif
   
   #ifdef _PD_RTAI
      #include <rtai_config.h>
      #include <rtai.h>
      #include <rtai_sched.h>
      #include <rtai_sem.h>
   #endif
   
   #ifdef _PD_XENOMAI
      #include <rtdm/rtdm_driver.h>
   #endif
   
   #include "kernel_compat.h"
#endif

#include "win_ddk_types.h"
#include "pdfw_def.h"
#include "powerdaq-internal.h"
#include "powerdaq-extension.h"
#include "powerdaq.h"
#include "pdfw_if.h"
#include "kvmem.h"

#ifndef PD_GLOBAL_PREFIX
   #define PD_GLOBAL_PREFIX extern
   extern const char *pd_devices_by_minor[];
   extern const PD_SUBSYSTEM pd_subsystems_by_minor[];
#else
   const char *pd_devices_by_minor[] = {
      "ain",
      "aout",
      "din",
      "dout",
      "uct",
      "drv",
      "dspct"
   };
   
   const PD_SUBSYSTEM pd_subsystems_by_minor[] = {
      AnalogIn,
      AnalogOut,
      DigitalIn,
      DigitalOut,
      CounterTimer,
      BoardLevel,
      DSPCounter
   };
#endif

extern pd_board_t pd_board[PD_MAX_BOARDS];
extern int num_pd_boards;

// Spinlocks to make driver SMP-safe
// Programming the board operations consists of several
// writes/reads to board registers. Those sequences
// of writes and reads need to be atomic or the board's
// firmware will get very confused.
// This spinlock ensure that only one thread can access
// one given board.
#if defined(_PD_RTL)
   PD_GLOBAL_PREFIX pthread_spinlock_t pd_fw_lock;
   
   #define _fw_spinlock pthread_spin_lock(&pd_fw_lock);
   #define _fw_spinunlock pthread_spin_unlock(&pd_fw_lock);
#elif defined(_PD_RTLPRO)
   PD_GLOBAL_PREFIX rtl_pthread_spinlock_t pd_fw_lock;

   #define _fw_spinlock rtl_pthread_spin_lock(&pd_fw_lock);
   #define _fw_spinunlock rtl_pthread_spin_unlock(&pd_fw_lock);
#elif defined(_PD_RTAI)
   PD_GLOBAL_PREFIX spinlock_t pd_fw_lock;
   PD_GLOBAL_PREFIX unsigned long pd_fw_flags;

   #define _fw_spinlock pd_fw_flags = rt_spin_lock_irqsave(&pd_fw_lock);
   #define _fw_spinunlock rt_spin_unlock_irqrestore(pd_fw_flags, &pd_fw_lock);
#elif defined(_PD_XENOMAI)
   PD_GLOBAL_PREFIX rtdm_lock_t pd_fw_lock;
   PD_GLOBAL_PREFIX rtdm_lockctx_t pd_fw_lock_ctx;

   #define _fw_spinlock rtdm_lock_get_irqsave(&pd_fw_lock, pd_fw_lock_ctx);
   #define _fw_spinunlock rtdm_lock_put_irqrestore(&pd_fw_lock, pd_fw_lock_ctx);
#else
   PD_GLOBAL_PREFIX spinlock_t pd_fw_lock;
   PD_GLOBAL_PREFIX unsigned long pd_fw_flags;

   #define _fw_spinlock spin_lock_irqsave(&pd_fw_lock, pd_fw_flags);
   #define _fw_spinunlock spin_unlock_irqrestore(&pd_fw_lock, pd_fw_flags);
#endif

// global variables for deferred processing of interrupts
// Under Linux we use bottom-halves
// Under RT-Linux we use a thread that wakes-up after the ISR is called
// Under RTAI we use a task that waits on a semaphore unlocked from the ISR
#if defined(_PD_RTL)
   PD_GLOBAL_PREFIX pthread_t       rt_bh_thread[PD_MAX_BOARDS];
#elif defined(_PD_RTLPRO)
   PD_GLOBAL_PREFIX rtl_pthread_t   rt_bh_thread[PD_MAX_BOARDS];
   PD_GLOBAL_PREFIX rtl_sem_t       rt_bh_sem[PD_MAX_BOARDS];
#elif defined(_PD_RTAI)
   PD_GLOBAL_PREFIX RT_TASK         rt_bh_task[PD_MAX_BOARDS];
   PD_GLOBAL_PREFIX SEM             rt_bh_sem[PD_MAX_BOARDS];
#endif

#if defined(_PD_RTL) || defined(_PD_RTAI) || defined(_PD_RTLPRO) || defined(_PD_XENOMAI)
   PD_GLOBAL_PREFIX pd_board_ext_t pd_board_ext[PD_MAX_BOARDS];
   #define GET_BOARD_EXT(X)  (pd_board_ext_t*)(pd_board[X].extension)
#endif

void pd_bottom_half(void *data);

#if defined(_PD_RTL) || defined(_PD_RTLPRO)
   void* pd_rt_bottom_half(void *data);
#elif defined(_PD_RTAI)
   void pd_rt_bottom_half(int board);
#endif


int pd_isr_serve_board(int board);
#if defined(_PD_RTL)
   unsigned int pd_rt_isr(unsigned int irq, struct pt_regs *regs);
#elif defined(_PD_RTLPRO)
   unsigned int pd_rt_isr(unsigned int irq, struct rtl_frame *regs);
#elif defined(_PD_RTAI)
   void pd_rt_isr(int irq);
#else
   #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
      irqreturn_t pd_isr(int trigger_irq, void *dev_id);
   #elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
      irqreturn_t pd_isr(int trigger_irq, void *dev_id, struct pt_regs * regs);
   #else
      void pd_isr(int trigger_irq, void *dev_id, struct pt_regs * regs);
   #endif
#endif

void pd_kill_fasync(struct fasync_struct *fa, int sig);

unsigned long pd_copy_from_user32(u32* to, u32* from, u32 len);
unsigned long pd_copy_to_user32(u32* to, u32* from, u32 len);

unsigned long pd_copy_from_user16(u16* to, u16* from, u32 len);
unsigned long pd_copy_to_user16(u16* to, u16* from, u32 len);

unsigned long pd_copy_from_user8(u8* to, u8* from, u32 len);
unsigned long pd_copy_to_user8(u8* to, u8* from, u32 len);

void pd_udelay(u32 usecs);
void pd_mdelay(u32 msecs);
void* pd_alloc_bigbuf(u32 size);
void pd_free_bigbuf(void* mem, u32 size);
        
int pd_event_create(int board, TSynchSS **sync);
int pd_event_wait(int board, TSynchSS *sync, int timeoutms);
int pd_event_signal(int board, TSynchSS *sync);
int pd_event_destroy(int board, TSynchSS *synch);

// Operating system abstraction layer function calls
typedef int (* tPdIrqHandler)(int board);
int pd_driver_register(int board, int minor);
int pd_driver_unregister(int board, int minor);
int pd_driver_open(int board, int minor);
int pd_driver_close(int board, int minor);
int pd_driver_ioctl(int board, int board_minor, int command, tCmd* argcmd);
int pd_driver_request_irq(int board, tPdIrqHandler handler);
int pd_driver_release_irq(int board);

