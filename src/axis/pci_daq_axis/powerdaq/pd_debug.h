//===========================================================================
//
// NAME:    pdcmd.h
//
// DESCRIPTION:
//
//          PowerDAQ Linux debug statements definitions
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
#ifndef __PD_DEBUG_H__
#define __PD_DEBUG_H__

                                                 
// COMMENT IT OUT TO WRITE TO THE HERCULES SCREEN
#define USE_KERNEL_LOG    

// Itemized debug messages
#ifdef PD_DEBUG

#ifndef USE_KERNEL_LOG
    #include "PrintH.h"
#endif

// Event debug messages
#define PD_DEBUG_E

// FW calls errors (FW call FAILS)
#define PD_DEBUG_F

// I/O debug messages
#define PD_DEBUG_I

// Information messages (initialize/release)
#define PD_DEBUG_N

// IRQ debug messages
#define PD_DEBUG_Q

// FW calls parameters
#define PD_DEBUG_P

// Signaling debug messages
#define PD_DEBUG_S

// FW calls trace
#define PD_DEBUG_T

#endif

#if defined(MODULE) && defined(USE_KERNEL_LOG)
  #if defined(_PD_RTL)
     #define PRINTK(fmt, args...) rt_printk(fmt, ## args)
  #elif defined(_PD_RTLPRO)
     #define PRINTK(fmt, args...) rtl_printf(fmt, ## args)
  #elif defined(_PD_RTAI)
     #define PRINTK(fmt, args...) rt_printk(fmt, ## args)
  #else
     #define PRINTK(fmt, args...) printk(KERN_DEBUG PD_ID fmt, ## args)
  #endif

  #ifdef PD_DEBUG
     #define DPRINTK(fmt, args...) PRINTK(fmt, ## args)
     #define DPRINTK_IF(cond, fmt, args...) if(cond == 0) PRINTK(fmt, ## args)
  #else
     #define DPRINTK(fmt, args...)
     #define DPRINTK_IF(cond, fmt, args...)
  #endif

  // Event debug statements
  #ifdef PD_DEBUG_E
     #define DPRINTK_E(fmt, args...) PRINTK(fmt, ## args)
  #else
     #define DPRINTK_E(fmt, args...)
  #endif

  // FW calls errors
  #ifdef PD_DEBUG_F
     #define DPRINTK_F(fmt, args...) PRINTK(fmt, ## args)
  #else
     #define DPRINTK_F(fmt, args...)
  #endif

  // I/O debug statements
  #ifdef PD_DEBUG_I
     #define DPRINTK_I(fmt, args...) PRINTK(fmt, ## args)
  #else
     #define DPRINTK_I(fmt, args...)
  #endif

  // Information messages (initialize/release)
  #ifdef PD_DEBUG_N
     #define DPRINTK_N(fmt, args...) PRINTK(fmt, ## args)
  #else
     #define DPRINTK_N(fmt, args...)
  #endif

  // FW calls parameters
  #ifdef PD_DEBUG_P
     #define DPRINTK_P(fmt, args...) PRINTK(fmt, ## args)
  #else
     #define DPRINTK_P(fmt, args...)
  #endif

  // IRQ debug statements
  #ifdef PD_DEBUG_Q
     #define DPRINTK_Q(fmt, args...) PRINTK(fmt, ## args)
  #else
     #define DPRINTK_Q(fmt, args...)
  #endif

  // Signaling debug statements
  #ifdef PD_DEBUG_S
     #define DPRINTK_S(fmt, args...) PRINTK(fmt, ## args)
  #else
     #define DPRINTK_S(fmt, args...)
  #endif

  // FW calls trace
  #ifdef PD_DEBUG_T
     #define DPRINTK_T(fmt, args...) PRINTK(fmt, ## args)
  #else
     #define DPRINTK_T(fmt, args...)
  #endif

#elif defined(MODULE)
  #ifdef PD_DEBUG
     #define DPRINTK(fmt, args...) hercmon_printf(fmt, ## args)
     #define DPRINTK_IF(cond, fmt, args...) if(cond == 0) hercmon_printf(fmt, ## args)
  #else
     #define DPRINTK(fmt, args...)
     #define DPRINTK_IF(cond, fmt, args...)
  #endif

  // Event debug statements
  #ifdef PD_DEBUG_E
     #define DPRINTK_E(fmt, args...) hercmon_printf(fmt, ## args)
  #else
     #define DPRINTK_E(fmt, args...)
  #endif

  // FW calls errors
  #ifdef PD_DEBUG_F
     #define DPRINTK_F(fmt, args...) hercmon_printf(fmt, ## args)
  #else
     #define DPRINTK_F(fmt, args...)
  #endif

  // I/O debug statements
  #ifdef PD_DEBUG_I
     #define DPRINTK_I(fmt, args...) hercmon_printf(fmt, ## args)
  #else
     #define DPRINTK_I(fmt, args...)
  #endif

  // Information messages (initialize/release)
  #ifdef PD_DEBUG_N
     #define DPRINTK_N(fmt, args...) hercmon_printf(fmt, ## args)
  #else
     #define DPRINTK_N(fmt, args...)
  #endif

  // FW calls parameters
  #ifdef PD_DEBUG_P
     #define DPRINTK_P(fmt, args...) hercmon_printf(fmt, ## args)
  #else
     #define DPRINTK_P(fmt, args...)
  #endif

  // IRQ debug statements
  #ifdef PD_DEBUG_Q
     #define DPRINTK_Q(fmt, args...) hercmon_printf(fmt, ## args)
  #else
     #define DPRINTK_Q(fmt, args...)
  #endif

  // Signaling debug statements
  #ifdef PD_DEBUG_S
     #define DPRINTK_S(fmt, args...) hercmon_printf(fmt, ## args)
  #else
     #define DPRINTK_S(fmt, args...)
  #endif

  // FW calls trace
  #ifdef PD_DEBUG_T
     #define DPRINTK_T(fmt, args...) hercmon_printf(fmt, ## args)
  #else
     #define DPRINTK_T(fmt, args...)
  #endif
#else
  
  #ifdef PD_DEBUG
    #define DPRINTK(fmt, args...) printf(fmt, ## args)
  #else
    #define DPRINTK(fmt, args...)
  #endif

  // Event debug statements
  #ifdef PD_DEBUG_E
     #define DPRINTK_E(fmt, args...) printf(fmt, ## args)
  #else
     #define DPRINTK_E(fmt, args...)
  #endif

  // FW calls errors
  #ifdef PD_DEBUG_F
     #define DPRINTK_F(fmt, args...) printf(fmt, ## args)
  #else
     #define DPRINTK_F(fmt, args...)
  #endif

  // I/O debug statements
  #ifdef PD_DEBUG_I
     #define DPRINTK_I(fmt, args...) printf(fmt, ## args)
  #else
     #define DPRINTK_I(fmt, args...)
  #endif

  // Information messages (initialize/release)
  #ifdef PD_DEBUG_N
     #define DPRINTK_N(fmt, args...) printf(fmt, ## args)
  #else
     #define DPRINTK_N(fmt, args...)
  #endif

  // FW calls parameters
  #ifdef PD_DEBUG_P
     #define DPRINTK_P(fmt, args...) printf(fmt, ## args)
  #else
     #define DPRINTK_P(fmt, args...)
  #endif

  // IRQ debug statements
  #ifdef PD_DEBUG_Q
     #define DPRINTK_Q(fmt, args...) printf(fmt, ## args)
  #else
     #define DPRINTK_Q(fmt, args...)
  #endif

  // Signaling debug statements
  #ifdef PD_DEBUG_S
     #define DPRINTK_S(fmt, args...) printf(fmt, ## args)
  #else
     #define DPRINTK_S(fmt, args...)
  #endif

  // FW calls trace
  #ifdef PD_DEBUG_T
     #define DPRINTK_T(fmt, args...) printf(fmt, ## args)
  #else
     #define DPRINTK_T(fmt, args...)
  #endif
   
#endif // USE_KERNEL_LOG





#endif // __PD_DEBUG_H__
