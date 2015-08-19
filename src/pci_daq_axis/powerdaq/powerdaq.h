/*===========================================================================*/
/*                                                                           */    
/* NAME:    powerdaq.h                                                       */
/*                                                                           */
/* DESCRIPTION:                                                              */
/*                                                                           */
/*          PowerDAQ Linux driver definitions needed to include into the     */
/*          library and applications                                         */
/*                                                                           */
/* AUTHOR: Frederic Villeneuve                                               */
/*         Alex Ivchenko                                                     */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*      Copyright (C) 2000,2004 United Electronic Industries, Inc.           */
/*      All rights reserved.                                                 */
/*---------------------------------------------------------------------------*/
/* For more informations on using and distributing this software, please see */
/* the accompanying "LICENSE" file.                                          */
/*===========================================================================*/
#ifndef __POWER_DAQ_H__
#define __POWER_DAQ_H__


#include "pdfw_def.h"
#include "pd_types.h"

/* AIn modes definition */
#define AIN_SINGLE_ENDED   0
#define AIN_DIFFERENTIAL   AIB_INPMODE

#define AIN_UNIPOLAR       0
#define AIN_BIPOLAR        AIB_INPTYPE

#define AIN_RANGE_5V       0
#define AIN_RANGE_10V      AIB_INPRANGE

#define AIN_CV_CLOCK_SW          0
#define AIN_CV_CLOCK_INTERNAL    AIB_CVSTART0
#define AIN_CV_CLOCK_EXTERNAL    AIB_CVSTART1
#define AIN_CV_CLOCK_CONTINUOUS  (AIB_CVSTART0 | AIB_CVSTART1)

#define AIN_CV_EXT_CLOCK_RISE    0
#define AIN_CV_EXT_CLOCK_FALL    AIB_EXTCVS

#define AIN_CL_CLOCK_SW          0
#define AIN_CL_CLOCK_INTERNAL    AIB_CLSTART0
#define AIN_CL_CLOCK_EXTERNAL    AIB_CLSTART1
#define AIN_CL_CLOCK_CONTINUOUS  (AIB_CLSTART0 | AIB_CLSTART1)

#define AIN_CL_EXT_CLOCK_RISE    0
#define AIN_CL_EXT_CLOCK_FALL    AIB_EXTCLS

#define AIN_CV_INT_CLOCK_BASE_11MHZ  0
#define AIN_CV_INT_CLOCK_BASE_33MHZ  AIB_INTCVSBASE

#define AIN_CL_INT_CLOCK_BASE_11MHZ  0
#define AIN_CL_INT_CLOCK_BASE_33MHZ  AIB_INTCLSBASE

#define AIN_START_TRIGGER_NONE       0
#define AIN_START_TRIGGER_RISE       AIB_STARTTRIG0
#define AIN_START_TRIGGER_FALL       (AIB_STARTTRIG0 | AIB_STARTTRIG1)

#define AIN_STOP_TRIGGER_NONE        0
#define AIN_STOP_TRIGGER_RISE        AIB_STOPTRIG1
#define AIN_STOP_TRIGGER_FALL        (AIB_STOPTRIG0 | AIB_STOPTRIG1)

#define AIN_SCANRETMODE_MMAP      0
#define AIN_SCANRETMODE_RAW       1
#define AIN_SCANRETMODE_VOLTS     2

#define SCANRETMODE_MMAP      0
#define SCANRETMODE_RAW       1
#define SCANRETMODE_VOLTS     2


/*---------------------------------------------------------------------------*/
/*                                                                           */
/* Macros for constructing Channel List entries.                             */
/*                                                                           */
#define CHAN(c)     ((c) & 0x3f)
#define GAIN(g)     (((g) & 0x3) << 6)
#define SLOW        (1<<8)
#define CHLIST_ENT(c,g,s)   (CHAN(c) | GAIN(g) | ((s) ? SLOW : 0))

/*---------------------------------------------------------------------------*/
/*                                                                           */
/* Macro to convert A/D samples from 2's complement to straight binary.      */
/*                                                                           */
/*Alex::No longer needed. Use pd_hcaps.h for conversion                      */
/* ...                                                                       */
/*#define AIN_BINARY(a)   (((a) & ANDWORD) ^ XORWORD)                        */


/*---------------------------------------------------------------------------*/
/* Definitions for mapdsp.sys Device Driver.                                 */
/*---------------------------------------------------------------------------*/
#define TRUE        1
#define FALSE       0

#define STATUS_SUCCESS      1
#define STATUS_UNSUCCESSFUL 0

#ifndef PCI_TYPE0_ADDRESSES
  #define PCI_TYPE0_ADDRESSES     6
#endif

/*---------------------------------------------------------------------------*/
/*                                                                           */
/* PCI Configuration Space:                                                  */
/*                                                                           */
typedef struct _PD_PCI_CONFIG
{
    u16    VendorID;
    u16    DeviceID;
    u16    Command;
    u16    Status;
    u8     RevisionID;
    u8     CacheLineSize;
    u8     LatencyTimer;
    ptr_t       BaseAddress0;
    u16    SubsystemVendorID;
    u16    SubsystemID;
    u8     InterruptLine;
    u8     InterruptPin;
    u8     MinimumGrant;
    u8     MaximumLatency;
} PD_PCI_CONFIG, *PPD_PCI_CONFIG;


#define PD_MAX_BOARDS 8         /* The max number of boards the driver can deal with.*/
#define PD_MAX_CL_SIZE 256      /* Longest channel list*/
#define MAX_PWRDAQ_ADAPTERS PD_MAX_BOARDS
#define PD_MAX_AIN_CHLIST_ENTRIES PD_MAX_CL_SIZE
#define PD_MAX_BUFFER_SIZE 1024         /* Maximum Buffer size for transfer through the ioctl interface*/
#define PD_MAX_SUBSYSTEMS 8 /* max number of subsystems in PowerDAQ PCI cards*/
#define MAX_PCI_SPACES      1   /* number of PCI Type 0 base address registers*/

/*---------------------------------------------------------------------------*/
/*                                                                           */
/* Interface Data Structures:                                                */
/*                                                                           */
   
#define PD_EEPROM_SIZE          312     /* EEPROM size in 16-bit words*/
#define PD_SERIALNUMBER_SIZE    10      /* serial number length in bytes*/
#define PD_DATE_SIZE            12      /* date string length in bytes*/
#define PD_CAL_AREA_SIZE        32      /* EEPROM calibration area size in 16-bit words*/
#define PD_SST_AREA_SIZE        96      /* EEPROM start-up state area size in 16-bit words*/

typedef struct _PD_EEPROM
{
   union
   {
      struct _Header
      {
         UCHAR   ADCFifoSize;
         UCHAR   CLFifoSize;
         UCHAR   SerialNumber[PD_SERIALNUMBER_SIZE];
         UCHAR   ManufactureDate[PD_DATE_SIZE];
         UCHAR   CalibrationDate[PD_DATE_SIZE];
         ULONG   Revision;
         USHORT  FirstUseDate;
         USHORT  CalibrArea[PD_CAL_AREA_SIZE];
         USHORT  FWModeSelect;
         USHORT  StartupArea[PD_SST_AREA_SIZE];
         USHORT  PXI_Config[5];                 
         UCHAR   DACFifoSize;
      } Header;

      USHORT WordValues[PD_EEPROM_SIZE];
   } u;
} PD_EEPROM, *PPD_EEPROM;

#define PWRDAQ_EEPROM PD_EEPROM

typedef enum
{
    BoardLevel  = 0,
    AnalogIn    = 1,
    AnalogOut,
    DigitalIn,
    DigitalOut,
    CounterTimer,
    CalDiag,
    DSPCounter
} PD_SUBSYSTEM, * PPD_SUBSYSTEM;

#define EdgeDetect    0x8000  /* prohibit subsystem substitution in DLL*/


/* Subsystem Events (Events):*/

typedef enum {
    eStartTrig          = (1L<<0),   /* start trigger / operation started*/
    eStopTrig           = (1L<<1),   /* stop trigger / operation stopped*/
    eInputTrig          = (1L<<2),   /* subsystem specific input trigger*/
    eDataAvailable      = (1L<<3),   /* new data / points available*/

    eScanDone           = (1L<<4),   /* scan done*/
    eFrameDone          = (1L<<5),   /* logical frame done*/
    eFrameRecycled      = (1L<<6),   /* cyclic buffer frame recycled*/
    eBlockDone          = (1L<<7),   /* logical block done (FUTURE)*/

    eBufferDone         = (1L<<8),   /* buffer done*/
    eBufListDone        = (1L<<9),   /* buffer list done (FUTURE)*/
    eBufferWrapped      = (1L<<10),  /* cyclic buffer / list wrapped*/
    eConvError          = (1L<<11),  /* conversion clock error*/

    eScanError          = (1L<<12),  /* scan clock error*/
    eDataError          = (1L<<13),  /* data error (out-of-range)*/
    eBufferError        = (1L<<14),  /* buffer over/under run error*/
    eTrigError          = (1L<<15),  /* trigger error*/

    eStopped            = (1L<<16),  /* operation stopped*/
    eTimeout            = (1L<<17),  /* operation timed-out*/
    eAllEvents          = (0xFFFFF) /* set/clear all events*/
} PDEvent;

typedef enum {
    eDInEvent           = (1L<<0),   /* Digital Input event*/
    eUct0Event          = (1L<<1),   /* Uct0 countdown event*/
    eUct1Event          = (1L<<2),   /* Uct1 countdown event*/
    eUct2Event          = (1L<<3),   /* Uct2 countdown event*/
} PDDigEvent;

#define     AIB_BUFFERWRAPPED   0x1
#define     AIB_BUFFERRECYCLED  0x2
#define     BUF_BUFFERWRAPPED   0x1  /* now applied to AIn/AOut/DIn/DOut/CT buffers*/
#define     BUF_BUFFERRECYCLED  0x2
#define     BUF_DWORDVALUES     0x10
#define     BUF_FIXEDDMA        0x20


/*---------------------------------------------------------------------------*/
/*                                                                           */
/* PowerDAQ IOCTL:                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
#define PWRDAQX_CONTROL_CODE(request, method) request+0x100
#define METHOD_BUFFERED 0

/*---------------------------------------------------------------------------*/

#define IOCTL_PWRDAQ_PRIVATE_MAP_DEVICE   PWRDAQX_CONTROL_CODE(0, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_PRIVATE_UNMAP_DEVICE PWRDAQX_CONTROL_CODE(1, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_PRIVATE_GETCFG       PWRDAQX_CONTROL_CODE(2, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_PRIVATE_SETCFG       PWRDAQX_CONTROL_CODE(3, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_PRIVATE_SET_EVENT    PWRDAQX_CONTROL_CODE(4, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_PRIVATE_CLR_EVENT    PWRDAQX_CONTROL_CODE(5, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_PRIVATE_SLEEP_ON     PWRDAQX_CONTROL_CODE(6, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_PRIVATE_WAKE_UP      PWRDAQX_CONTROL_CODE(7, METHOD_BUFFERED)

#define IOCTL_PWRDAQ_GET_NUMBER_ADAPTER PWRDAQX_CONTROL_CODE(9, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_ACQUIRESUBSYSTEM   PWRDAQX_CONTROL_CODE(0xA, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_REGISTER_BUFFER    PWRDAQX_CONTROL_CODE(0xB, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_UNREGISTER_BUFFER  PWRDAQX_CONTROL_CODE(0xC, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_REGISTER_EVENTS    PWRDAQX_CONTROL_CODE(0xD, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_UNREGISTER_EVENTS  PWRDAQX_CONTROL_CODE(0xE, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_GET_EVENTS         PWRDAQX_CONTROL_CODE(0xF, METHOD_BUFFERED)

#define IOCTL_PWRDAQ_GETADCFIFOSIZE     PWRDAQX_CONTROL_CODE(0x10, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_GETSERIALNUMBER    PWRDAQX_CONTROL_CODE(0x11, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_GETMANFCTRDATE     PWRDAQX_CONTROL_CODE(0x12, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_GETCALIBRATIONDATE PWRDAQX_CONTROL_CODE(0x13, METHOD_BUFFERED)

#define IOCTL_PWRDAQ_SET_USER_EVENTS    PWRDAQX_CONTROL_CODE(0x14, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_CLEAR_USER_EVENTS  PWRDAQX_CONTROL_CODE(0x15, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_GET_USER_EVENTS    PWRDAQX_CONTROL_CODE(0x16, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_IMMEDIATE_UPDATE   PWRDAQX_CONTROL_CODE(0x17, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_SET_TIMED_UPDATE   PWRDAQX_CONTROL_CODE(0x18, METHOD_BUFFERED)

/* PowerDAQ Asynchronous Buffered AIn/AOut Operations.*/
#define IOCTL_PWRDAQ_AIN_ASYNC_INIT     PWRDAQX_CONTROL_CODE(0x1E, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AIN_ASYNC_TERM     PWRDAQX_CONTROL_CODE(0x1F, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AIN_ASYNC_START    PWRDAQX_CONTROL_CODE(0x20, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AIN_ASYNC_STOP     PWRDAQX_CONTROL_CODE(0x21, METHOD_BUFFERED)

#define IOCTL_PWRDAQ_AO_ASYNC_INIT      PWRDAQX_CONTROL_CODE(0x22, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AO_ASYNC_TERM      PWRDAQX_CONTROL_CODE(0x23, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AO_ASYNC_START     PWRDAQX_CONTROL_CODE(0x24, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AO_ASYNC_STOP      PWRDAQX_CONTROL_CODE(0x25, METHOD_BUFFERED)

#define IOCTL_PWRDAQ_GET_DAQBUF_STATUS  PWRDAQX_CONTROL_CODE(0x28, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_GET_DAQBUF_SCANS   PWRDAQX_CONTROL_CODE(0x29, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_CLEAR_DAQBUF       PWRDAQX_CONTROL_CODE(0x2A, METHOD_BUFFERED)

/* Low Level PowerDAQ Board Level Commands.*/
#define IOCTL_PWRDAQ_BRDRESET           PWRDAQX_CONTROL_CODE(0x64, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_BRDEEPROMREAD      PWRDAQX_CONTROL_CODE(0x65, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_BRDEEPROMWRITEDATE PWRDAQX_CONTROL_CODE(0x66, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_BRDENABLEINTERRUPT PWRDAQX_CONTROL_CODE(0x67, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_BRDTESTINTERRUPT   PWRDAQX_CONTROL_CODE(0x68, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_BRDSTATUS          PWRDAQX_CONTROL_CODE(0x69, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_BRDSETEVNTS1       PWRDAQX_CONTROL_CODE(0x6A, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_BRDSETEVNTS2       PWRDAQX_CONTROL_CODE(0x6B, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_BRDFWLOAD          PWRDAQX_CONTROL_CODE(0x6C, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_BRDFWEXEC          PWRDAQX_CONTROL_CODE(0x6D, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_BRDREGWR           PWRDAQX_CONTROL_CODE(0x6E, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_BRDREGRD           PWRDAQX_CONTROL_CODE(0x6F, METHOD_BUFFERED)

/* Low Level PowerDAQ AIn Subsystem Commands.*/
#define IOCTL_PWRDAQ_AISETCFG           PWRDAQX_CONTROL_CODE(0xC8, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AISETCVCLK         PWRDAQX_CONTROL_CODE(0xC9, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AISETCLCLK         PWRDAQX_CONTROL_CODE(0xCA, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AISETCHLIST        PWRDAQX_CONTROL_CODE(0xCB, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AISETEVNT          PWRDAQX_CONTROL_CODE(0xCC, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AISTATUS           PWRDAQX_CONTROL_CODE(0xCD, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AICVEN             PWRDAQX_CONTROL_CODE(0xCE, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AISTARTTRIG        PWRDAQX_CONTROL_CODE(0xCF, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AISTOPTRIG         PWRDAQX_CONTROL_CODE(0xD0, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AISWCVSTART        PWRDAQX_CONTROL_CODE(0xD1, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AISWCLSTART        PWRDAQX_CONTROL_CODE(0xD2, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AICLRESET          PWRDAQX_CONTROL_CODE(0xD3, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AICLRDATA          PWRDAQX_CONTROL_CODE(0xD4, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AIRESET            PWRDAQX_CONTROL_CODE(0xD5, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AIGETVALUE         PWRDAQX_CONTROL_CODE(0xD6, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AIGETSAMPLES       PWRDAQX_CONTROL_CODE(0xD7, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AISETSSHGAIN       PWRDAQX_CONTROL_CODE(0xD8, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AIGETXFERSAMPLES   PWRDAQX_CONTROL_CODE(0xD9, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AISETXFERSIZE      PWRDAQX_CONTROL_CODE(0xDA, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AIXFERSIZE         PWRDAQX_CONTROL_CODE(0xDB, METHOD_BUFFERED) 
#define IOCTL_PWRDAQ_AIRETRIEVE         PWRDAQX_CONTROL_CODE(0xDC, METHOD_BUFFERED) 


#define IOCTL_PWRDAQ_AIENABLECLCOUNT    PWRDAQX_CONTROL_CODE(0x78, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AIENABLETIMER      PWRDAQX_CONTROL_CODE(0x79, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AIFLUSHFIFO        PWRDAQX_CONTROL_CODE(0x7A, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AIGETSAMPLECOUNT   PWRDAQX_CONTROL_CODE(0x7B, METHOD_BUFFERED)

/* Low Level PowerDAQ AOut Subsystem Commands.*/
#define IOCTL_PWRDAQ_AOSETCFG           PWRDAQX_CONTROL_CODE(0x12C, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AOSETCVCLK         PWRDAQX_CONTROL_CODE(0x12D, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AOSETEVNT          PWRDAQX_CONTROL_CODE(0x12E, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AOSTATUS           PWRDAQX_CONTROL_CODE(0x12F, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AOCVEN             PWRDAQX_CONTROL_CODE(0x130, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AOSTARTTRIG        PWRDAQX_CONTROL_CODE(0x131, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AOSTOPTRIG         PWRDAQX_CONTROL_CODE(0x132, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AOSWCVSTART        PWRDAQX_CONTROL_CODE(0x133, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AOCLRDATA          PWRDAQX_CONTROL_CODE(0x134, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AORESET            PWRDAQX_CONTROL_CODE(0x135, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AOPUTVALUE         PWRDAQX_CONTROL_CODE(0x136, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AOPUTBLOCK         PWRDAQX_CONTROL_CODE(0x137, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_AODMASET           PWRDAQX_CONTROL_CODE(0x13A, METHOD_BUFFERED) 
#define IOCTL_PWRDAQ_AOSETWAVE          PWRDAQX_CONTROL_CODE(0x13B, METHOD_BUFFERED)


/* Low Level PowerDAQ DIn Subsystem Commands.*/
#define IOCTL_PWRDAQ_DISETCFG           PWRDAQX_CONTROL_CODE(0x190, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_DISTATUS           PWRDAQX_CONTROL_CODE(0x191, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_DIREAD             PWRDAQX_CONTROL_CODE(0x192, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_DICLRDATA          PWRDAQX_CONTROL_CODE(0x193, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_DIRESET            PWRDAQX_CONTROL_CODE(0x194, METHOD_BUFFERED)

/* Low Level PowerDAQ DOut Subsystem Commands.*/
#define IOCTL_PWRDAQ_DOWRITE            PWRDAQX_CONTROL_CODE(0x1F4, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_DORESET            PWRDAQX_CONTROL_CODE(0x1F5, METHOD_BUFFERED)

/* Low Level PowerDAQ DIO-256 Subsystem Commands.*/
#define IOCTL_PWRDAQ_DIO256CMDWR        PWRDAQX_CONTROL_CODE(0x1F6, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_DIO256CMDRD        PWRDAQX_CONTROL_CODE(0x1F7, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_DIO256SETINTRMASK  PWRDAQX_CONTROL_CODE(0x1F8, METHOD_BUFFERED) 
#define IOCTL_PWRDAQ_DIO256GETINTRDATA  PWRDAQX_CONTROL_CODE(0x1F9, METHOD_BUFFERED) 
#define IOCTL_PWRDAQ_DIO256INTRREENABLE PWRDAQX_CONTROL_CODE(0x1FA, METHOD_BUFFERED) 
#define IOCTL_PWRDAQ_DIODMASET          PWRDAQX_CONTROL_CODE(0x1FB, METHOD_BUFFERED) 
#define IOCTL_PWRDAQ_DIO256CMDWR_ALL    PWRDAQX_CONTROL_CODE(0x1FC, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_DIO256CMDRD_ALL    PWRDAQX_CONTROL_CODE(0x1FD, METHOD_BUFFERED)



/* Low Level PowerDAQ UCT Subsystem Commands.*/
#define IOCTL_PWRDAQ_UCTSETCFG          PWRDAQX_CONTROL_CODE(0x258, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_UCTSTATUS          PWRDAQX_CONTROL_CODE(0x259, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_UCTWRITE           PWRDAQX_CONTROL_CODE(0x25A, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_UCTREAD            PWRDAQX_CONTROL_CODE(0x25B, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_UCTSWGATE          PWRDAQX_CONTROL_CODE(0x25C, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_UCTSWCLK           PWRDAQX_CONTROL_CODE(0x25D, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_UCTRESET           PWRDAQX_CONTROL_CODE(0x25E, METHOD_BUFFERED)

/* Low Level PowerDAQ Cal Subsystem Commands.*/
#define IOCTL_PWRDAQ_CALSETCFG          PWRDAQX_CONTROL_CODE(0x2BC, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_CALDACWRITE        PWRDAQX_CONTROL_CODE(0x2BD, METHOD_BUFFERED)

/* Low Level PowerDAQ Diag Subsystem Commands.*/
#define IOCTL_PWRDAQ_DIAGPCIECHO        PWRDAQX_CONTROL_CODE(0x320, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_DIAGPCIINT         PWRDAQX_CONTROL_CODE(0x321, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_BRDEEPROMWRITE     PWRDAQX_CONTROL_CODE(0x322, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_DIAGSETPRM         PWRDAQX_CONTROL_CODE(0x323, METHOD_BUFFERED)

/* Linux driver test only ioctl's*/
#define IOCTL_PWRDAQ_TESTEVENTS         PWRDAQX_CONTROL_CODE(0x385, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_TESTBUFFERING      PWRDAQX_CONTROL_CODE(0x386, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_OPENSUBSYSTEM      PWRDAQX_CONTROL_CODE(0x387, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_CLOSESUBSYSTEM     PWRDAQX_CONTROL_CODE(0x388, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_RELEASEALL         PWRDAQX_CONTROL_CODE(0x389, METHOD_BUFFERED)

/* RT-specific ioctls*/
#define IOCTLRT_PWRDAQ_GETKERNELBUFPTR  PWRDAQX_CONTROL_CODE(0x3E9, METHOD_BUFFERED)
#define IOCTL_PWRDAQ_GETKERNELBUFSIZE    PWRDAQX_CONTROL_CODE(0x3EA, METHOD_BUFFERED)

/* Latency measurement ioctl*/
#define IOCTL_PWRDAQ_MEASURE_LATENCY    PWRDAQX_CONTROL_CODE(0x3EB, METHOD_BUFFERED)


/*                                                                           */
/* File contains structures to perform requests to the driver                */
/* structures from the PowerDAQ user-level library                           */
/*                                                                           */


/* Analog Input synchronous operation structure                              */
/* return averaged channel values in dwChList                                */
typedef struct
{
   u32 Subsystem;
   u32 dwMode;
   u32 dwChListSize;
   u32 dwChList[PD_MAX_CL_SIZE];
} tSyncCfg;

typedef struct
{
   u32 Subsystem;
   u32 dwAInCfg;
   u32 dwEventsNotify;
   u32 dwAInPreTrigCount;
   u32 dwAInPostTrigCount;
   u32 dwCvRate;
   u32 dwClRate;
   u32 dwChListSize;
   u32 dwChList[PD_MAX_CL_SIZE];
} tAsyncCfg;

/* Board hardware event words.*/
typedef struct
{
   u32   Board;
   u32   ADUIntr;
   u32   AIOIntr;
   u32   AInIntr;
   u32   AOutIntr;
} tEvents;

/* DaqBuf Get Scan Info Struct*/
typedef struct
{
   u32   Subsystem; 
   u32   NumScans;       /* number of scans to get*/
   u32   ScanIndex;      /* buffer index of first scan*/
   u32   NumValidScans;  /* number of valid scans available*/
   u32   ScanRetMode;    /* how to copy scans into user buffer*/
} tScanInfo;

typedef struct _PD_DAQBUF_STATUS_INFO
{
   u32           dwAdapterId;        /* Adapter ID*/
   PD_SUBSYSTEM  Subsystem;          /* subsystem*/
   /*-----------------------*/
   u32           dwSubsysState;      /* OUT: current subsystem state*/
   u32           dwScanIndex;        /* OUT: buffer index of first scan*/
   u32           dwNumValidValues;   /* OUT: number of valid values available*/
   u32           dwNumValidScans;    /* OUT: number of valid scans available*/
   u32           dwNumValidFrames;   /* OUT: number of valid frames available*/
   u32           dwWrapCount;        /* OUT: total num times buffer wrapped*/

   u32           dwFirstTimestamp;   /* OUT: first sample timestamp*/
   u32           dwLastTimestamp;    /* OUT: last sample timestamp*/
} tDaqBufStatusInfo, PD_DAQBUF_STATUS_INFO;


typedef struct
{
   u32 MaxSize;
   u32 WordsRead;
   u16 Buffer[PD_EEPROM_SIZE];
} tEepromAcc;

typedef struct
{
   u32   fileDescriptor;
   u32   subSystem;
} tAcqSS;

typedef struct
{
   u32 size;
   u8  buffer[PD_MAX_BUFFER_SIZE];
} tBuffer;


/* Main command structure                                                    */
/* union contains ioctl-specific information needed to communicate           */
/* with the driver                                                           */
/*                                                                           */
typedef union 
{
   tSyncCfg     SyncCfg;
   tEvents      Event;
   u32          dwParam[8];
   tBuffer      bufParam;
   tEepromAcc   EepromAcc;
   tAsyncCfg    AsyncCfg;
   tAcqSS       AcqSS; 
   tScanInfo    ScanInfo;
   PD_PCI_CONFIG PciConfig;
} tCmd;


#endif


