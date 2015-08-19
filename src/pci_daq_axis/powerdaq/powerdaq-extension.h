//===========================================================================
//
// NAME:    powerdaq-extension.h
//
// DESCRIPTION:
//
//          PowerDAQ header file
//
//
// AUTHOR: Frederic Villeneuve 
//         Alex Ivchenko
//
//---------------------------------------------------------------------------
//      Copyright (C) 2000,2004 United Electronic Industries, Inc.
//      All rights reserved.
//---------------------------------------------------------------------------
// For more informations on using and distributing this software, please see
// the accompanying "LICENSE" file.
//
#ifndef __POWERDAQ_EXTENSION_H__
#define __POWERDAQ_EXTENSION_H__

// common data structure used to synchronize access
// to each sub-system
struct _synchSS
{
#if defined(_PD_RTL)
   pthread_cond_t   event;
   pthread_mutex_t  event_lock;
#elif defined (_PD_RTLPRO)
   //rtl_pthread_cond_t   event;
   //rtl_pthread_mutex_t  event_lock;
   rtl_sem_t   event;
#elif defined(_PD_RTAI)
   CND event;
   SEM event_lock;
#else
   spinlock_t lock;
   unsigned long lock_flags;
   struct semaphore sem;         // syncronization object

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,0)
   struct wait_queue *wait_q;    // wait queue for blocking I/O
#else
   wait_queue_head_t wait_q;
#endif
#endif // _PD_RTL
   PD_SUBSYSTEM subsystem;     // Id of the subsystem that owns this structure
   int wakeupEvents;           // Events that will wake-up the subsystem
   int notifiedEvents;         // Events that were received
};

// This extension is specific to RTLinux
// it contains the user ISR data
typedef struct _board_extension
{
   TUser_isr user_isr;
   void *user_param;

#if defined(_PD_RTL)
   pthread_mutex_t user_isr_lock;
#elif defined(_PD_RTLPRO)
   rtl_pthread_mutex_t user_isr_lock;
#elif defined(_PD_RTAI)
   SEM user_isr_lock;
#elif defined(_PD_XENOMAI)
   rtdm_mutex_t user_isr_lock;
   struct rtdm_device* rtdmdev[PD_MAX_SUBSYSTEMS];
#endif
} pd_board_ext_t;

#if defined(_PD_XENOMAI)
typedef struct _pd_rtdm_ctx
{
   int board;
   PD_SUBSYSTEM ss;
} pd_rtdm_ctx_t;
#endif

#endif
