//===========================================================================
//
// NAME:    pd_dsp_ct.h
//
// DESCRIPTION:
//          Definitions for PowerDAQ PCI Device Driver.
//
// NOTES:   This is DSP counter-timer register definitions
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
//
//===========================================================================
//
// PD2-DIO UCT Support
//
// General Information
//
// PowerDAQ DIO board has three 24-bit counter/timers.
// Also there is an additional 21-bit counter called prescaler which can be
// used as a pre-divider or for the programmable cascading purposes.
// Each counter can be clocked from the one of the three sources
// ·	External TIOx pin (Note: That when external signal source attached
//     100-200 Ohm series resistor should be used to protect DSP TIOx pin)
// ·	Internal DSP Clock/2 (33 MHz for the all PD2-DIO boards)
// ·	Prescaler output
// Please note: That each counter/timer has programmable polarity and can generate
// interrupt on the any of the two events - compare and overflow.
// Each timer has an associated compare register that is used in some modes to
// generate PWM signal and to generate compare interrupt in all modes.
// DSP Counters are count-up type counters.
//
// Prescaler counter can use any of the four following sources (software selectable)
// ·	Internal DSP Clock/2 (33 MHz for the all PD2-DIO boards)
// ·	External TIO0/1/2 pin
//
// Please refer to the Motorola DSP56301 User Manual for the details
// (Motorola P/N DSP56301UM/AD)
//
// PowerDAQ Implementation
//
// DIO Interrupts are supported by revision 3 of the PowerDAQ driver that is in the
// beta stage right now. Following algorithm should be used:
//
// 1)	All timers should be programmed using the _PdDspRegWrite  and _PdDSPRegRead
//   functions and proper constants. Note that using of the wrong addresses can
//   cause system problems.
// 2)	Timer1 should not be used with DIn asynchronous operations and Timer2 with
//   DOut asynchronous operations.
// 3)	UCT Interrupts should be enabled first by _PdSetUserEvents function, after
//   that using the _PdDspRegWrite  function user should enable interrupt for the
//   selected counter (M_TOIE and M_TCIE bits in M_TCSRx). Note that because only one
//   bit is dedicated in the events word for the each UCT interrupt only one type of
//   the interrupt should be enabled at the time.
// 4)	After host is interrupted because of the UCT interrupt it should be
//   re-enabled if necessary using the same algorithm.
// 5)	Before program is closed each DSP Timer used should be reset using
//   the _PdDspRegWrite function (write zero to the corresponding TCSR).
// 6)	The functions below are not supported and should NEVER used with DSP UCTs
//
// PDUCTCFG
// PDUCTSTATUS
// PDUCTWRITE
// PDUCTREAD
// PDUCTSWGATE
// PDUCTSWCLK
// PDUCTRESET
//
//       Register Addresses Of TIMER0
#define M_TCSR0    0xFFFF8F      // TIMER0 Control/Status Register
#define M_TLR0     0xFFFF8E      // TIMER0 Load Reg
#define M_TCPR0    0xFFFF8D      // TIMER0 Compare Register
#define M_TCR0     0xFFFF8C      // TIMER0 Count Register
//
//       Register Addresses Of TIMER1
#define M_TCSR1    0xFFFF8B      // TIMER1 Control/Status Register
#define M_TLR1     0xFFFF8A      // TIMER1 Load Reg
#define M_TCPR1    0xFFFF89      // TIMER1 Compare Register
#define M_TCR1     0xFFFF88      // TIMER1 Count Register
//
//       Register Addresses Of TIMER2
#define M_TCSR2    0xFFFF87      // TIMER2 Control/Status Register
#define M_TLR2     0xFFFF86      // TIMER2 Load Reg
#define M_TCPR2    0xFFFF85      // TIMER2 Compare Register
#define M_TCR2     0xFFFF84      // TIMER2 Count Register
//
//       Prescaler Registers
#define M_TPLR     0xFFFF83      // TIMER Prescaler Load Register
#define M_TPCR     0xFFFF82      // TIMER Prescalar Count Register
//
//       Timer Control/Status Register Bit Flags
#define M_TE       (1 << 0)      // Timer Enable
#define M_TOIE     (1 << 1)      // Timer Overflow Interrupt Enable
#define M_TCIE     (1 << 2)      // Timer Compare Interrupt Enable
#define M_TC0      (1 << 4)      // Timer Control 0
#define M_TC1      (1 << 5)      // Timer Control 1
#define M_TC2      (1 << 6)      // Timer Control 2
#define M_TC3      (1 << 7)      // Timer Control 3
#define M_TC       0xF0          // Timer Control Mask   (TC0-TC3)
#define M_INV      (1 << 8)      // Inverter Bit
#define M_TRM      (1 << 9)      // Timer Restart Mode
#define M_DIR      (1 << 11)     // Direction Bit
#define M_DI       (1 << 12)     // Data Input
#define M_DO       (1 << 13)     // Data Output
#define M_PCE      (1 << 15)     // Prescaled Clock Enable
#define M_TOF      (1 << 20)     // Timer Overflow Flag
#define M_TCF      (1 << 21)     // Timer Compare Flag
//
//       Timer Prescaler Register Bit Flags
#define M_PS       0x600000      // Prescaler Source Mask
#define M_PS0      (1 << 21)
#define M_PS1      (1 << 22)
#define M_PS_CLK   0
#define M_PS_CLK2  0                 // Use internal clock / 2 
#define M_PS_TIO0  (M_PS0)           // Use timer IO 0
#define M_PS_TIO1  (M_PS1)           // Use timer IO 1
#define M_PS_TIO2  (M_PS0 | M_PS1)   // Use timer IO 2

//
//       Timer Mode Masks
#define DCT_Timer              0x0
#define DCT_TimerPulse         0x10
#define DCT_TimerToggle        0x20
#define DCT_EventCounter       0x30
#define DCT_InputWidth         0x40
#define DCT_InputPeriod        0x50
#define DCT_Capture            0x60
#define DCT_PWM                0x70
#define DCT_WatchdogPulse      0x90
#define DCT_WatchdogToggle     0xA0
//
//       DCT_Status bits
#define DCTB_TE0       (1 << 0)   // Timer 0 Enable
#define DCTB_TE1       (1 << 1)   // Timer 1 Enable
#define DCTB_TE2       (1 << 2)   // Timer 2 Enable
#define DCTB_TA0       (1 << 3)   // Timer 0 Available
#define DCTB_TA1       (1 << 4)   // Timer 1 Available
#define DCTB_TA2       (1 << 5)   // Timer 2 Available
#define DCTB_FMODE     (1 << 6)   // Frequency Mode On
#define DCTB_FDONE     (1 << 7)   // Frequency Measurement Done
#define DCTB_TFME0     (1 << 8)   // Timer 0 Frequency Mode Enable
#define DCTB_TFME1     (1 << 9)   // Timer 1 Frequency Mode Enable
//
// Counter IDs
#define DCT_UCT0       0x0    // counter 0
#define DCT_UCT1       0x1    // counter 1
#define DCT_UCT2       0x2    // counter 2




