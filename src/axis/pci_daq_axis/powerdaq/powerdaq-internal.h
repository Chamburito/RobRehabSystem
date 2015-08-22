//===========================================================================
//
// NAME:    powerdaq-internal.h
//
// DESCRIPTION:
//
//          PowerDAQ Linux driver definitions needed for kernel driver only
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
#ifndef __POWER_DAQ_INTERNAL_H__
#define __POWER_DAQ_INTERNAL_H__

#include "powerdaq.h"
#include "pdfw_def.h"
#include "win_ddk_types.h"
#include "pd_debug.h"

//  The major number of this device driver.  Dont change this unless you
//  know what you're doing!
#define PD_MAJOR 61


//#define USECOMMONFW 1

// Uncomment this if you are using Red-Hat 6.2 or 7.0
// Red-Hat 6.2 is based on a 2.2.x kernel with patches
// back ported from the kernel 2.4.x
//#define PD_USING_REDHAT_62

// Some ID strings for the driver.
#define PD_ID "pd: "

//
//  Minor numbers...
//
#define PD_MINOR_RANGE 7 

#define PD_MINOR_AIN   0
#define PD_MINOR_AOUT  1
#define PD_MINOR_DIN   2
#define PD_MINOR_DOUT  3
#define PD_MINOR_UCT   4
#define PD_MINOR_DRV   5
#define PD_MINOR_DSPCT 6


// Subsystem States (State):
typedef enum {
    ssConfig            = 0x0,      // configuration state (default)
    ssStandby           = 0x1,      // on standby ready to start
    ssRunning           = 0x2,      // running
    ssPaused            = 0x4,      // paused
    ssStopped           = 0x8,      // stopped
    ssDone              = 0x10,     // operation done, stopped
    ssError             = 0x20,     // error condition, stopped
    ssActive            = 0x3       // subsystem either running or standby
} PDSubsystemState;


// DaqBuf Get DaqBuf Status Info Struct
typedef struct
{
    u32   SubsysState;      // current subsystem state
    u32   BufSizeInBytes;
    u32   ScanValues;       // maximum number of scans in buffer
    u32   MaxValues;        // maximum number of samples in buffer
    u32   FrameValues;      // maximum number of frames in buffer
    u32   SampleSize;       // size of the sample
    u32   DataWidth;        // size of single value in bytes
    u32   ScanIndex;        // buffer index of first scan
    u32   NumValidValues;   // number of valid values available
    u32   NumValidScans;    // number of valid scans available
    u32   NumValidFrames;   // number of valid frames available
    u32   bFixedDMA;        // Fixed DMA mode
    u32   bDWValues;        // DWORD values shall be used instead of WORD
    u32   WrapCount;        // total num times buffer wrapped
    u32   ScanSize;         // scan size in samples (2 bytes each)
    u32   FrameSize;        // frame size in scans
    u32   NumFrames;        // number of scans
    u16*  databuf;          // pointer to the buffer contains samples
    u16*  userdatabuf;      // pointer to the user buffer contains samples
    u32   Head;             // head of the buffer
    u32   Tail;             // tail of the buffer
    u32   Count;            // current position in the buffer
    u32   ValueCount;       // number of samples
    u32   ValueIndex;       // sample index
    u32   BufMode;          // mode of buffer usage
    u32   bWrap;            // buffer is in the "WRAPPED" mode
    u32   bRecycle;         // buffer is in the "RECYCLED" mode
    u32   FirstTimestamp;   // first sample timestamp
    u32   LastTimestamp;    // last sample timestamp
} TBuf_Info, * PTBuf_Info;

// set up run parameters (set parameter to -1 to keep default):
typedef struct _XFERPARAM 
{
    int XFerMode;       // transfer mode
    int InXFerCyc;      // number of input blocks to xfer (in 512 samples chunks)
    int OutXFerCyc;     // number of output block to xfer (in 1024 samples chunks)
    int DoGetSamples;   // Do GetSamples after XFer when flush FIFO
    int BmFHFXFers;     // number of BM transfers from FIFO (in 512 samples chunks)
    int BmPageXFers;    // number of BM transfers to the page (in 512 samples chunks)
} XFERPARAM, *pXFERPARAM;

// input structure to IOCTL_PWRDAQ_ALLOC_BMBUF
typedef struct _AllocContigMem
{
   u32 idx;
   u32 size;    // # of 4K pages (to be used later, currently defaults to 4)
   u32 linearAddr;
   u32 physAddr;
   u32 status;
} tAllocContigMem;

struct _synchSS;

typedef struct _synchSS TSynchSS, *PTSynchSS;


// this structure holds information about AIn subsystem
typedef struct
{
    u32   SubsysState;            // current subsystem state
    u32   SubsysConfig;           // keeps latest AIn configuration
    u32   bAsyncMode;
    u32   dwAInCfg;               // AIn configuration word
    u32   dwAInPreTrigCount;      // pre-trigger scan count
    u32   dwAInPostTrigCount;     // post-trigger scan count
    u32   dwCvRate;               // conversion start clock divider
    u32   dwClRate;               // channel list start clock divider
    u32   dwEventsNotify;         // subsystem user events notification
    u32   dwEventsStatus;         // subsystem user events status
    u32   dwEventsNew;            // new events
    u32   dwChListChan;           // number of channels in list
    u32   ChList[PD_MAX_CL_SIZE]; // channel list data buffer
    TBuf_Info BufInfo;            // buffer information
    tScanInfo ScanInfo;          // scan information
    u32   FifoValues;             // ???
    u32   bInUse;                 // TRUE -> SS is in use
    u32   bCheckHalfDone;         // check FHF/HalfDone event
    u32   bCheckFifoError;        // check Overrun/Underrun Error event
    u32   bImmUpdate;             // Immediate update enable

    u32   XferBufValues;          // number of values in Xfer buffer
    u32*  pXferBuf;               // DMA ready driver buffer
    u32   XferBufValueCount;      // number of samples written to Xfer buffer
    u32   BlkXferValues;          // number of values to transfer using DSP DMA
    u32   FifoXFerCycles;         // blocks of 512 samples
    u32   DoGetSamples;           // perform get samples after XFer

    u32   BmFHFXFers;             // size of 1/2 FIFO in 512 samples chunks
    u32   BmPageXFers;            // size of cont physical page in 512 samples chunks
    u32   AIBMTXSize;             // DSP internal BM buffer size, samples

    struct _synchSS *synch;
} TAinSS, *PTAinSS;


// this structure holds information about AOut subsystem
typedef struct
{
    u32   SubsysState;
    u32   SubsysConfig;           // keeps latest AOut configuration
    u32   dwMode;
    u32   dwAOutPreTrigCount;     // pre-trigger scan count
    u32   dwAOutPostTrigCount;    // post-trigger scan count
    u32   dwCvRate;               // conversion start clock divider
    u32   dwClRate;               // channel list start clock divider
    u32   dwAoutValue;
    u32   dwEventsNotify;
    u32   dwEventsStatus;
    u32   dwEventsNew;
    u32   dwChListChan;           // number of channels in list
    u32   ChList[PD_MAX_CL_SIZE]; // channel list data buffer
    TBuf_Info BufInfo;
    tScanInfo ScanInfo;
    u32   FifoValues;             // ???
    u32   bInUse;                 // TRUE -> SS is in use
    u32   bAsyncMode;
    u32   bCheckHalfDone;         // check FHF/HalfDone event
    u32   bCheckFifoError;        // check Overrun/Underrun Error event
    u32   bImmUpdate;             // Immediate update enable
    u32   dwWakeupEvents;         // events to wake up blocked request
    u32   dwNotifyEvents;         // events to notify on wakeup
    u32   timeout;

    u32   XferBufValues;          // number of values in Xfer buffer
    u32*  pXferBuf;               // DMA ready driver buffer
    u32   XferBufValueCount;      // number of samples written to Xfer buffer
    u32   BlkXferValues;          // number of values to transfer using DSP DMA
    u32   FifoXFerCycles;         // blocks of 512 samples
    u32   TranSize;

    u32   BmFHFXFers;             // size of 1/2 FIFO in 512 samples chunks
    u32   BmPageXFers;            // size of cont physical page in 512 samples chunks
    u32   AIBMTxSize;             // DSP internal BM buffer size, samples

    u32   bRev3Mode;              // Select between old way/new way of doing data transfer

    struct _synchSS *synch;
} TAoutSS, *PTAoutSS;

// this structure holds information about UCT subsystem
typedef struct
{
    u32   SubsysState;
    u32   bAsyncMode;
    u32   dwMode;
    u32   dwCvRate;
    u32   dwEventsNotify;
    u32   dwEventsStatus;
    u32   dwEventsNew;
    u32   bInUse;                 // TRUE -> SS is in use
    u32   dwWakeupEvents;         // events to wake up blocked request
    u32   dwNotifyEvents;         // events to notify on wakeup
    u32   timeout;
    struct _synchSS *synch;
} TUctSS, * PTUctSS;

// this structure holds information about DIO subsystem
typedef struct
{
    u32   SubsysState;
    u32   bAsyncMode;
    u32   dwMode;
    u32   dwCvRate;
    u32   dwEventsNotify;
    u32   dwEventsStatus;
    u32   dwEventsNew;
    u32   bInUse;                 // TRUE -> SS is in use
    u32   dwWakeupEvents;         // events to wake up blocked request
    u32   dwNotifyEvents;         // events to notify on wakeup
    u32   timeout;
    u32   intrData[16];
    u32   intrMask[8];
    struct _synchSS *synch;
} TDioSS, * PTDioSS;

// Prototype of a routine that a user can install to be notified
// when the board receives an interrupt.
// This is only used with RTLinux.
typedef void (* TUser_isr)(void *user_param);

//
// this structure holds everything the driver knows about a board
//
typedef struct
{
   struct pci_dev *dev;
   void *address;
   u32 size;
   int index;
   int irq;
   int open;
   int bTestInt;     // TRUE for interrupt test

   u32 HI32CtrlCfg;
   u16 caps_idx;    // board type index in pd_hcaps.h
   u32 dwXFerMode;

   struct fasync_struct *fasync;  // for asynchronous notification (SIGIO)

   // Subsystem-related variables
   PD_EEPROM Eeprom;
   u32 fwTimestamp[2];
   u32 logicRev;
   PD_PCI_CONFIG PCI_Config;
   int bImmUpdate;
   int bUseHeavyIsr;
   int fd[PD_MAX_SUBSYSTEMS];

   // Event-related variables
   tEvents FwEventsConfig;   // current adapter's events config
   tEvents FwEventsNotify;   // firmware events to notify
   tEvents FwEventsStatus;   // firmware event status saved
      
   u32 intMutex;

   TAinSS  AinSS;
   TAoutSS AoutSS;
   TUctSS  UctSS;
   TDioSS  DinSS;
   TDioSS  DoutSS;

   dma_addr_t DMAHandleBMB[4]; // DMA handle that contains a physical address
   			       // usable by a PCI device with 32 or 64 bits CPUs
   void*  pSysBMB[4];          // virtual addresses of BM buffer
   void*  pLinBMB[4];          // linear, i.e. user space address
   u32    SizeBMB[4];          // size of BM buffer
   u32    cp;                  // current AIn page

#ifdef MEASURE_LATENCY
   unsigned int tscLow;
   unsigned int tscHigh;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
   struct work_struct worker;
#else
   struct tq_struct worker;
#endif

   void *extension;
} pd_board_t;



//  The max number of times to poll the board before we declare it dead.
#define MAX_PCI_BUSY_WAIT 0xFFFFFF

//  The max number of milliseconds to wait for the board to act before we
//  declare it dead.  This is not currently used.
#define MAX_PCI_WAIT 1000

// AIn/AOut temporary storage data transfer buffer definition.
//
// Size set to be sufficient to store AIn/AOut FIFO samples.
//      (AIn/AOut FIFO samples * 2 bytes/sample)
//
#define PD_AIN_MAX_FIFO_VALUES  0x2000   //AI00310: max xfer buffer size
#define PD_AIN_FIFO_VALUES      0x400    //AI00310: minimal safe transfer
#define PD_AIN_MAX_XFERSIZE     0x1000   //AI00310: maximum xfer size
#define PD_BM_PAGES             4        // number of contigous pages
#define PD_BM_SPP               0x400    // samples per page

#define ANALOG_XFERBUF_VALUES   0x10000
#define ANALOG_XFERBUF_SIZE     (ANALOG_XFERBUF_VALUES * sizeof(ULONG))
#define PD_AOUT_MAX_FIFO_VALUES 0x800   // Maximum number of samples in DAC FIFO

#define XFERMODE_NORMAL     0       // use read/write with status register check
#define XFERMODE_FAST       1       // use read/write without status register check
#define XFERMODE_BM         2       // use bus master for downstream transfers
#define XFERMODE_BM8WORD    3       // use bus master with shorter bursts
#define XFERMODE_RTBM       4
#define XFERMODE_NOAIMEM    0xe     // error: no AI contigous memory
#define XFERMODE_NOAOMEM    0xf     // error: no AO contigous memory
#define XFERMODE_BRDDIS     0xff    // error: board disabled

// bus mastering pages
#define BMPAGES     4
#define AI_PAGE0    0
#define AI_PAGE1    1
#define AO_PAGE0    2
#define AO_PAGE1    3


#endif


