/*===========================================================================*/
/*                                                                           */
/* NAME:    powerdaq32.h                                                     */
/*                                                                           */
/* DESCRIPTION:                                                              */
/*                                                                           */
/*          PowerDAQ Linux library header file                               */
/*                                                                           */
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
#ifndef __POWERDAQ32_H__
#define __POWERDAQ32_H__

/* Do update this #define for each version !!!------------------------*/
#define _PDDLL32VER     00523

/*#include <signal.h>*/

/* supress name-mangling if this file is included from a C++ file*/
#ifdef __cplusplus
extern "C" {
#endif

/* Uncomment next line if you want to compile library for RT*/
/*#define _PD_RTL*/

/*--------------------------------------------------------------------*/
/* EEPROM data structure*/
/*--------------------------------------------------------------------*/
#define PD_EEPROM_SIZE          312     /* EEPROM size in 16-bit WORDs*/
#define PD_SERIALNUMBER_SIZE    10      /* serial number length in BYTEs*/
#define PD_DATE_SIZE            12      /* date string length in BYTEs*/
#define PD_CAL_AREA_SIZE        32      /* EEPROM calibration area size in 16-bit WORDs*/
#define PD_SST_AREA_SIZE        96      /* EEPROM start-up state area size in 16-bit WORDs*/

/* This is an entry point if powerdaq32.c is compiled to work from within*/
/* the kernel space (compiled with _PD_RTL)*/
#ifdef _PD_RTL
    int pd_rt_ioctl(int board, unsigned int command, unsigned long argument)
#endif

/*                                                                           */
/* PCI Configuration Space info (only partial)                               */
/*                                                                           */
typedef struct _PWRDAQ_PCI_CONFIG
{
    WORD    VendorID;
    WORD    DeviceID;
    WORD    Command;
    WORD    Status;
    BYTE    RevisionID;
    BYTE    CacheLineSize;
    BYTE    LatencyTimer;
    ULONG   BaseAddress0;
    WORD    SubsystemVendorID;
    WORD    SubsystemID;
    BYTE    InterruptLine;
    BYTE    InterruptPin;
    BYTE    MinimumGrant;
    BYTE    MaximumLatency;
} PWRDAQ_PCI_CONFIG, * PPWRDAQ_PCI_CONFIG;

/*                                                                           */
/* Driver version and timestamp, along with some system facts                */
/*                                                                           */
typedef struct _PWRDAQ_VERSION
{
    DWORD   SystemSize;
    BOOL    NtServerSystem;
    DWORD   NumberProcessors;
    DWORD   MajorVersion;
    DWORD   MinorVersion;
    BYTE    BuildType;
    BYTE    BuildTimeStamp[40];
} PWRDAQ_VERSION, * PPWRDAQ_VERSION;

/* Capabilities structure and table ---------------------------------------- */
#define PD_BRD          0
#define PD_BRD_BASEID   0x101

#define MAXRANGES   8
#define MAXGAINS    4
#define MAXSS   PD_MAX_SUBSYSTEMS

/* Hardware properties definition ------------------------------------------ */
#define PDHCAPS_BITS            1
#define PDHCAPS_FIRSTCHAN       2
#define PDHCAPS_LASTCHAN        3
#define PDHCAPS_CHANNELS        4



typedef struct DAQ_Information_STRUCT
{
    WORD   iBoardID;          /* board ID*/
    LPSTR  lpBoardName;       /* Name of the specified board*/
    LPSTR  lpBusType;         /* Bus type*/
    LPSTR  lpDSPRAM;          /* Type of DSP and volume of RAM*/
    LPSTR  lpChannels;        /* Number of channels of the all types, main string*/
    LPSTR  lpTrigCaps;        /* AIn triggering capabilities*/
    LPSTR  lpAInRanges;       /* AIn ranges*/
    LPSTR  lpAInGains;        /* AIn gains*/
    LPSTR  lpTransferTypes;   /* Types of suported transfer methods*/
    DWORD  iMaxAInRate;       /* Max AIn rate (pacer clock)*/
    LPSTR  lpAOutRanges;      /* AOut ranges*/
    DWORD  iMaxAOutRate;      /* Max AOut rate (pacer clock)*/
    LPSTR  lpUCTType;         /* Type of used UCT*/
    DWORD  iMaxUCTRate;       /* Max UCT rate*/
    DWORD  iMaxDIORate;       /* Max DIO rate*/
    WORD   wXorMask;          /* Xor mask*/
    WORD   wAndMask;          /* And mask*/
} DAQ_Information, * PDAQ_Information;


/* information structure*/
/*                      */
/*                      */
typedef struct SUBSYS_Info_STRUCT
{
   DWORD  dwChannels;         /* Number of channels of the subsystem type, main string*/
   DWORD  dwChBits;           /**NEW* how wide is the channel*/
   DWORD  dwRate;             /* Maximum output rate*/

   DWORD  dwMaxGains;         /* = MAXGAINS*/
   float  fGains[MAXGAINS];   /* Array of gains*/

   /* Information to convert values*/
   DWORD  dwMaxRanges;          /* = MAXRANGES*/
   float  fRangeLow[MAXRANGES]; /* Low part of range*/
   float  fRangeHigh[MAXRANGES];/* High part of the range*/
   float  fFactor[MAXRANGES];   /* What to mult value to*/
   float  fOffset[MAXRANGES];   /* What to substruct from value*/
   WORD   wXorMask;             /* Xor mask*/
   WORD   wAndMask;             /* And mask*/

   DWORD  dwFifoSize;           /* FIFO Size (in samples) for subsystem*/
   DWORD  dwChListSize;         /* Max number of entries in channel list*/

   /* Variable filled from application side for proper conversion*/
   DWORD  dwMode;               /* Input/Output Mode*/
   DWORD* dwChGainList;         /* Pointer to Active Channel Gain List*/
   DWORD  dwChGainListSize;     /* Size of channel list*/

}  SubSys_Info,  *PSubSys_Info;


#define    atPD2MF   (1 << 0)
#define    atPD2MFS  (1 << 1)
#define    atPDMF    (1 << 2)
#define    atPDMFS   (1 << 3)
#define    atPD2AO   (1 << 4)
#define    atPD2DIO  (1 << 5)
#define    atPXI     (1 << 6)
#define    atPDLMF   (1 << 7)
#define    atMF      (atPD2MF | atPD2MFS | atPDMF | atPDMFS | atPDLMF)

typedef struct ADAPTER_Info_STRUCT
{
    DWORD                 dwBoardID;             /* board ID*/
    DWORD                 atType;                /* Adapter type*/
    char                  lpBoardName[20];       /* Name of the specified board*/
    char                  lpSerialNum[20];       /* Serial Number*/
    SubSys_Info           SSI[MAXSS];            /* Subsystem description array*/
    unsigned char         PXI_Config[5];         /* PXI line config S.S.*/
} Adapter_Info, *PAdapter_Info;



/*------------------------------------------------------------------------*/
/*                                                                        */
/* Function Prototypes:                                                   */
/*                                                                        */
/*                                                                        */
/*------------------------------------------------------------------------*/

/* Returns version information*/
int PdGetVersion(PPWRDAQ_VERSION pVersion);

/* Returns number of adapters installed*/
int PdGetNumberAdapters(void);

/* Returns PCI config space*/
int PdGetPciConfiguration(int handle, PPWRDAQ_PCI_CONFIG pPciConfig);

/* Get/release subsystem to/from use*/
int PdAcquireSubsystem(int board, int dwSubsystem, int Action);

/*--- Event functions ----------------------------------------------------*/
int _PdSetUserEvents(int handle, PD_SUBSYSTEM Subsystem, DWORD dwEvents);
int _PdClearUserEvents(int handle, PD_SUBSYSTEM Subsystem, DWORD dwEvents);
int _PdGetUserEvents(int handle, PD_SUBSYSTEM Subsystem, DWORD *pdwEvents);
int _PdSetAsyncNotify(int handle, struct sigaction *io_act, void (*sig_proc)(int));
int _PdImmediateUpdate(int handle);

int _PdWaitForEvent(int handle, int events, int timeoutms);

int _PdAdapterGetBoardStatus(int handle, tEvents* pEvents);
int _PdAdapterSetBoardEvents1(int handle, DWORD dwEvents);
int _PdAdapterSetBoardEvents2(int handle, DWORD dwEvents);
int _PdAdapterEnableInterrupt(int handle, DWORD dwEnable);

/*--- Buffering functions ------------------------------------------------*/
int _PdRegisterBuffer(int handle,PWORD* pBuffer,
                                 DWORD dwSubsystem,
                                 DWORD dwFramesBfr,
                                 DWORD dwScansFrm,
                                 DWORD dwScanSize,
                                 DWORD bWrapAround);
int _PdUnregisterBuffer(int handle, PWORD pBuf, DWORD dwSubSystem);

int _PdAdapterEepromRead(int handle, DWORD dwMaxSize, WORD *pwReadBuf, DWORD *pdwWORDs);
int _PdAdapterEepromWrite(int handle,WORD *pwWriteBuf, DWORD dwSize);

int _PdAcquireBuffer(int handle,void** pBuffer,
                                 DWORD dwFramesBfr,
                                 DWORD dwScansFrm,
                                 DWORD dwScanSize,
                                 DWORD dwSubsystem,
                                 DWORD dwMode);
int _PdReleaseBuffer(int handle, DWORD dwSubSystem, void* pBuf);



/*--- Analog Input functions --------------------------------------------*/
int _PdAInAsyncInit(int handle,
                    DWORD dwAInCfg,
                    DWORD dwAInPreTrigCount, DWORD dwAInPostTrigCount,
                    DWORD dwAInCvClkDiv, DWORD dwAInClClkDiv,
                    DWORD dwEventsNotify,
                    DWORD dwChListChan, PDWORD pdwChList);
int _PdAInAsyncTerm(int handle);
int _PdAInAsyncStart(int handle);
int _PdAInAsyncStop(int handle);
int _PdAInGetScans(int handle,DWORD NumScans, DWORD ScanRetMode, DWORD *pScanIndex,
                             DWORD *pNumValidScans);
int _PdAInGetBufState(int handle, DWORD NumScans, DWORD ScanRetMode, 
                      DWORD *pScanIndex, DWORD *pNumValidScans); 


int _PdAInSetCfg(int handle, DWORD dwAInCfg, DWORD dwAInPreTrig, DWORD dwAInPostTrig);
int _PdAInSetCvClk(int handle, DWORD dwClkDiv);
int _PdAInSetClClk(int handle, DWORD dwClkDiv);
int _PdAInSetChList(int handle, DWORD dwCh, DWORD *pdwChList);
int _PdAInEnableConv(int handle, DWORD dwEnable);
int _PdAInSetEvents(int handle, DWORD dwEvents);
int _PdAInSwStartTrig(int handle);
int _PdAInSwStopTrig(int handle);
int _PdAInSwCvStart(int handle);
int _PdAInSwClStart(int handle);
int _PdAInResetCl(int handle);
int _PdAInClearData(int handle);
int _PdAInReset(int handle);
int _PdAInGetValue(int handle, WORD *pwSample);
int _PdAInGetDataCount(int handle, DWORD *pdwSamples);
int _PdAInGetSamples(int handle,DWORD dwMaxBufSize, WORD *pwBuf, DWORD *pdwSamples);
int _PdAInGetXFerSamples(int handle, DWORD dwMaxBufSize, WORD *pwBuf, DWORD *pdwSamples);
int _PdAInSetXferSize(int handle, DWORD size);
int _PdAInSetSSHGain(int handle, DWORD dwCfg);

/*--- AOut Subsystem Commands: ---------------------------------------*/
int _PdAOutSetCfg(int handle, DWORD dwAOutCfg, DWORD dwAOutPostTrig);
int _PdAOutSetCvClk(int handle, DWORD dwClkDiv);
int _PdAOutSetEvents(int handle, DWORD dwEvents);
int _PdAOutGetStatus(int handle, DWORD *pdwStatus);
int _PdAOutEnableConv(int handle, DWORD dwEnable);
int _PdAOutSwStartTrig(int handle);
int _PdAOutSwStopTrig(int handle);
int _PdAOutSwCvStart(int handle);
int _PdAOutClearData(int handle);
int _PdAOutReset(int handle);
int _PdAOutPutValue(int handle, DWORD dwValue);
int _PdAOutPutBlock(int handle, DWORD dwValues, DWORD *pdwBuf, DWORD *pdwCount);


int _PdAOutAsyncInit(int handle, DWORD dwAOutCfg, DWORD dwAOutCvClkDiv, 
                     DWORD dwEventNotify); 
int _PdAOutAsyncTerm(int handle); 
int _PdAOutAsyncStart(int handle);
int _PdAOutAsyncStop(int handle); 
int _PdAOutGetBufState(int handle, DWORD NumScans, DWORD ScanRetMode, 
                       DWORD* pScanIndex, DWORD* pNumValidScans); 

/*--- DIn Subsystem Commands: -----------------------------------------*/
int _PdDInSetCfg(int handle, DWORD dwDInCfg);
int _PdDInGetStatus(int handle, DWORD *pdwEvents);
int _PdDInRead(int handle, DWORD *pdwValue);
int _PdDInClearData(int handle);
int _PdDInReset(int handle);

/*--- DOut Subsystem Commands: ----------------------------------------*/
int _PdDOutWrite(int handle, DWORD dwValue);
int _PdDOutReset(int handle);

/*--- DIO256 Subsystem Commands: --------------------------------------*/
int _PdDIOReset(int handle);
int _PdDIOEnableOutput(int handle, DWORD dwRegMask);
int _PdDIOLatchAll(int handle, DWORD dwRegister);
int _PdDIOSimpleRead(int handle, DWORD dwRegister, DWORD *pdwValue);
int _PdDIORead(int handle, DWORD dwRegister, DWORD *pdwValue);
int _PdDIOWrite(int handle, DWORD dwRegister, DWORD dwValue);
int _PdDIOPropEnable(int handle, DWORD dwRegMask);
int _PdDIOExtLatchEnable(int handle, DWORD dwRegister, BOOL bEnable);
int _PdDIOExtLatchRead(int handle, DWORD dwRegister, BOOL *bLatch);
int _PdDIOSetIntrMask(int handle, DWORD* dwIntMask);
int _PdDIOGetIntrData(int handle, DWORD* dwIntData, DWORD* dwEdgeData);
int _PdDIOIntrEnable(int handle, DWORD dwEnable);
int _PdDIOSetIntCh(int handle, DWORD dwChannels);
int _PdDIODMASet(int handle, DWORD dwOffset, DWORD dwCount, DWORD dwSource);
int _PdDIOReadAll(int handle, DWORD *pdwValue);
int _PdDIOWriteAll(int handle, DWORD *pdwValue);



int _PdDIO256CmdWrite(int handle, DWORD dwCmd, DWORD dwValue);
int _PdDIO256CmdRead(int handle, DWORD dwCmd, DWORD *pdwValue);

int _PdDIAsyncInit(int handle,
                   DWORD dwDInCfg,
                   DWORD dwDInCvClkDiv,
                   DWORD dwEventsNotify,
                   DWORD dwChListChan,
                   DWORD dwFirstChannel); 
int _PdDIAsyncTerm(int handle); 
int _PdDIAsyncStart(int handle);
int _PdDIAsyncStop(int handle); 
int _PdDIGetBufState(int handle,DWORD NumScans, DWORD ScanRetMode, 
                     DWORD *pScanIndex, DWORD *pNumValidScans); 

/* PD2-DIO DOut*/
/*             */
int _PdDOAsyncInit(int handle, DWORD dwDOutCfg,
                   DWORD dwDOutCvClkDiv, DWORD dwEventNotify,
                   DWORD dwChListSize, PDWORD pdwChList); 
int _PdDOAsyncTerm(int handle); 
int _PdDOAsyncStart(int handle);
int _PdDOAsyncStop(int handle); 
int _PdDOGetBufState(int handle, DWORD NumScans, DWORD ScanRetMode, 
                     DWORD *pScanIndex, DWORD *pNumValidScans); 

/*--- AO32 Subsystem Commands: --------------------------------------*/
int _PdAO32Reset(int handle);
int _PdAO96Reset(int handle);
int _PdAO32Write(int handle, WORD wChannel, WORD wValue);
int _PdAO96Write(int handle, WORD wChannel, WORD wValue);
int _PdAO32WriteHold(int handle, WORD wChannel, WORD wValue);
int _PdAO96WriteHold(int handle, WORD wChannel, WORD wValue);
int _PdAO32Update(int handle);
int _PdAO96Update(int handle);
int _PdAO32SetUpdateChannel(int handle, WORD wChannel, BOOL bEnable);
int _PdAO96SetUpdateChannel(int handle, WORD wChannel, DWORD Mode);

int _PdAOAsyncInit(int handle, DWORD dwAOutCfg,
                   DWORD dwAOutCvClkDiv, DWORD dwEventNotify,
                   DWORD dwChListSize, PDWORD pdwChList); 
int _PdAOAsyncTerm(int handle); 
int _PdAOAsyncStart(int handle);
int _PdAOAsyncStop(int handle); 
int _PdAOGetBufState(int handle, DWORD NumScans, DWORD ScanRetMode, 
                     DWORD *pScanIndex, DWORD *pNumValidScans); 
int _PdAODMASet(int hAdapter, DWORD dwOffset, 
                           DWORD dwCount, DWORD dwSource); 


/*--- UCT Subsystem Commands: --------------------------------------*/
int _PdUctSetMode(int handle, DWORD dwCounter, 
                  DWORD dwSource, DWORD dwMode); 
int _PdUctWriteValue(int handle, DWORD dwCounter, 
                     WORD wValue); 
int _PdUctReadValue(int handle, DWORD dwCounter, 
                    WORD* wValue); 
int _PdUctFrqCounter(int handle, DWORD dwCounter, 
                     float fTime);
int _PdUctFrqGetValue(int handle, DWORD dwCounter, 
                      int* nFrequency); 


int _PdUctSetCfg(int handle, DWORD dwUctCfg);
int _PdUctGetStatus(int handle, DWORD *pdwStatus);
int _PdUctWrite(int handle, DWORD dwUctWORD);
int _PdUctRead(int handle, DWORD dwUctReadCfg, DWORD *pdwUctWORD);
int _PdUctSwSetGate(int handle, DWORD dwGateLevels);
int _PdUctSwClkStrobe(int handle);
int _PdUctReset(int handle);

/* PD2-DIO CT*/
/**/
int _PdCTAsyncInit(int handle,
                   DWORD dwCTCfg,
                   DWORD dwCTCvClkDiv,
                   DWORD dwEventsNotify,
                   DWORD dwChListChan); /* *note* connect TMR1 and IRQD for 2 channels*/
int _PdCTAsyncTerm(int handle); 
int _PdCTAsyncStart(int handle);
int _PdCTAsyncStop(int handle); 
int _PdCTGetBufState(int handle, DWORD NumScans, DWORD ScanRetMode, 
                     DWORD *pScanIndex, DWORD *pNumValidScans);

int _PdDspCtLoad(int handle, /* Load all registers of the selected counter*/
                           DWORD dwCounter, /* Counter register*/
                           DWORD dwLoad,    /* Load register*/
                           DWORD dwCompare, /* Compare register  */
                           DWORD dwMode,    /* Counting mode*/
                           DWORD bReload,    /* Reload flag (0/1)*/
                           DWORD bInverted,  /* Input/Output polarity*/
                           DWORD bUsePrescaler); /* Use prescaler */

/* Enable(1)/Disable(x) counting for the selected counter */
int _PdDspCtEnableCounter(int handle,
                          DWORD dwCounter, 
                          DWORD bEnable); 

/* Enable(1)/Disable(x) interrupts for the selected events for the selected */
/* counter (HANDLE hAdapter,only one event can be enabled at the time)*/
int _PdDspCtEnableInterrupts(int handle, 
                             DWORD dwCounter, 
                             DWORD bCompare, 
                             DWORD bOverflow);

/* Get count register value from the selected counter*/
int _PdDspCtGetCount(int handle, DWORD dwCounter, DWORD *dwCount);

/* Get compare register value from the selected counter*/
int _PdDspCtGetCompare(int handle, DWORD dwCounter, DWORD *dwCompare);
/* Get control/status register value from the selected counter*/
int _PdDspCtGetStatus(int handle, DWORD dwCounter, DWORD *dwStatus);

/* Set compare register value from the selected counter*/
int _PdDspCtSetCompare(int handle, DWORD dwCounter, DWORD dwCompare);
/* Set load register value from the selected counter*/
int _PdDspCtSetLoad(int handle, DWORD dwCounter, DWORD dwLoad);
/* Set control/status register value from the selected counter*/
int _PdDspCtSetStatus(int handle, DWORD dwCounter, DWORD dwStatus);

/* Load prescaler*/
int _PdDspPSLoad(int handle, DWORD dwLoad, DWORD dwSource);
/* Get prescaler count register value */
int _PdDspPSGetCount(int handle, DWORD *dwCount);

/* Get address of the count register of the selected counter*/
DWORD _PdDspCtGetCountAddr(DWORD dwCounter);

/* Get address of the load register of the selected counter*/
DWORD _PdDspCtGetLoadAddr(DWORD dwCounter);

/* Get address of the control/status register of the selected counter*/
DWORD _PdDspCtGetStatusAddr(DWORD dwCounter);

/* Get address of the compare register of the selected counter*/
DWORD _PdDspCtGetCompareAddr(DWORD dwCounter);



/*--- Calibration Commands: -----------------------------------------*/
int _PdCalSet(int handle, DWORD dwCalCfg);
int _PdCalDACWrite(int handle, DWORD dwCalDACValue);
int _PdCalDACSet(int handle, int nDAC, int nOut, int nValue);

/*--- DSP register access functions --------------------------------*/
int _PdDspRegWrite(int handle, DWORD reg, DWORD data);
int _PdDspRegRead(int handle, DWORD reg, DWORD *data);

/*--- Utility functions --------------------------------------------*/
int PdAInRawToVolts(int board, DWORD dwMode, WORD* dwRawData, double* fVoltage, DWORD dwCount);
int PdAOutVoltsToRaw(int board, DWORD dwMode, double* fVoltage, DWORD* dwRawData, DWORD dwCount);
int _PdGetAdapterInfo(DWORD dwBoardNum, PAdapter_Info pAdInfo);
int __PdGetAdapterInfo(DWORD dwBoardNum, PAdapter_Info pAdInfo);

/*--- Easy functions -----------------------------------------------*/
/* Single-point (one scan) acquisition*/
int PdAInAcqScan(int handle,
                 DWORD dwMode,          /* acquisition mode*/
                 DWORD dwChListSize,    /* size of channel list*/
                 DWORD* dwChList,       /* pointer to the channel list*/
                 double* pfVolts);      /* pointer to store data*/

/*========================================================================*/
/*                                                                        */
/* Function converts raw values to volts with gains                       */
/* Note: minimal conversion amount is one scan = dwChListSize samples     */
/*                                                                        */
int PdAInScanToVolts(int handle,         /* Handle to adapter*/
                     DWORD dwAInCfg,          /* Mode used*/
                     DWORD dwChListSize,      /* Channel list size*/
                     DWORD* dwChList,         /* Pointer to the channel list*/
                     WORD* wRawData,          /* Raw data*/
                     double* fVoltage,        /* Engineering unit*/
                     DWORD dwScans);          /* Number of scans to convert*/

/*========================================================================*/
/*                                                                        */
/* Function converts volts to raw values - into WORD format               */
/*                                                                        */
int PdAOVoltsToRaw16(int handle,         /* Handle to adapter*/
                     double* fVoltage,        /* Engineering unit*/
                     WORD* wRawData,          /* Raw data*/
                     DWORD dwCount);          /* Number of samples to convert*/

/*========================================================================*/
/*                                                                        */
/* Function converts volts to raw values - into DWORD format              */
/*                                                                        */
int PdAOVoltsToRaw32(int handle,         /* Handle to adapter*/
                     double* fVoltage,        /* Engineering unit*/
                     DWORD* dwRawData,        /* Raw data*/
                     DWORD dwScans);          /* Number of samples to convert*/

/*========================================================================*/
/*                                                                        */
/* Function converts volts to raw values - into DWORD format              */
/* and combines them with with channel list entries                       */
/*                                                                        */
int PdAOVoltsToRawCl(int handle,         /* Handle to adapter*/
                     DWORD dwChListSize,      /* Channel list size*/
                     DWORD* dwChList,         /* Pointer to the channel list*/
                     double* fVoltage,        /* Engineering unit*/
                     DWORD* dwRawData,        /* Raw data*/
                     DWORD dwScans);          /* Number of scans to convert*/

/* ----------------------------------------------------------------------*/
/* Function:  Returns actual value of channel list base clock (11/16,6/33 or 50 MHz)*/
/*              (for All PowerDAQ boards)*/
int _PdGetAdapterClBaseClock(int handle, int mode, int *pClBaseClock); 


/* ----------------------------------------------------------------------*/
/* Function:  Returns actual value of conversion base clock (11/16,6/33 or 50 MHz)*/
int _PdGetAdapterCvBaseClock(int hAdapter, int mode, int *pCvBaseClock);


/*--- EEPROM Helper functions -----------------------------------------------*/
int _PdGetADCFifoSize(int board, DWORD *fifoSize);
int _PdGetSerialNumber(int board, char serialNumber[PD_SERIALNUMBER_SIZE]);
int _PdGetManufactureDate(int board, char manfctrDate[PD_DATE_SIZE]);
int _PdGetCalibrationDate(int board, char calDate[PD_DATE_SIZE]);

/*--- benchmarking functions*/
int _PdGetLastIntTsc(int handle, unsigned int tsc[2]);
 
/*--- aux debugging functions*/
int _PdTestEvent(int handle);
void showEvents(DWORD events);
/*--- End of powerdaq32.h -------------------------------------------*/

/* PowerDAQ series ---------------------------*/
/* 12-bit multifunction medium speed (330 kHz)*/
#define PD_MF_16_330_12L        PD_BRD + 0x101
#define PD_MF_16_330_12H        PD_BRD + 0x102
#define PD_MF_64_330_12L        PD_BRD + 0x103
#define PD_MF_64_330_12H        PD_BRD + 0x104

/* 12-bit multifunction high speed (1 MHz)*/
#define PD_MF_16_1M_12L         PD_BRD + 0x105
#define PD_MF_16_1M_12H         PD_BRD + 0x106
#define PD_MF_64_1M_12L         PD_BRD + 0x107
#define PD_MF_64_1M_12H         PD_BRD + 0x108

/* 16-bit multifunction (250 kHz)*/
#define PD_MF_16_250_16L        PD_BRD + 0x109
#define PD_MF_16_250_16H        PD_BRD + 0x10A
#define PD_MF_64_250_16L        PD_BRD + 0x10B
#define PD_MF_64_250_16H        PD_BRD + 0x10C

/* 16-bit multifunction (50 kHz)*/
#define PD_MF_16_50_16L        PD_BRD + 0x10D
#define PD_MF_16_50_16H        PD_BRD + 0x10E

/* 12-bit, 6-ch 1M SSH*/
#define PD_MFS_6_1M_12         PD_BRD + 0x10F
#define PD_Nothing             PD_BRD + 0x110

/* PowerDAQ II series -------------------------*/
/* 14-bit multifunction low speed (400 kHz)*/
#define PD2_MF_16_400_14L        PD_BRD + 0x111
#define PD2_MF_16_400_14H        PD_BRD + 0x112
#define PD2_MF_64_400_14L        PD_BRD + 0x113
#define PD2_MF_64_400_14H        PD_BRD + 0x114

/* 14-bit multifunction medium speed (800 kHz)*/
#define PD2_MF_16_800_14L        PD_BRD + 0x115
#define PD2_MF_16_800_14H        PD_BRD + 0x116
#define PD2_MF_64_800_14L        PD_BRD + 0x117
#define PD2_MF_64_800_14H        PD_BRD + 0x118

/* 12-bit multifunction high speed (1.25 MHz)*/
#define PD2_MF_16_1M_12L         PD_BRD + 0x119
#define PD2_MF_16_1M_12H         PD_BRD + 0x11A
#define PD2_MF_64_1M_12L         PD_BRD + 0x11B
#define PD2_MF_64_1M_12H         PD_BRD + 0x11C

/* 16-bit multifunction (50 kHz)*/
#define PD2_MF_16_50_16L        PD_BRD + 0x11D
#define PD2_MF_16_50_16H        PD_BRD + 0x11E
#define PD2_MF_64_50_16L        PD_BRD + 0x11F
#define PD2_MF_64_50_16H        PD_BRD + 0x120

/* 16-bit multifunction (333 kHz)*/
#define PD2_MF_16_333_16L        PD_BRD + 0x121
#define PD2_MF_16_333_16H        PD_BRD + 0x122
#define PD2_MF_64_333_16L        PD_BRD + 0x123
#define PD2_MF_64_333_16H        PD_BRD + 0x124

/* 14-bit multifunction high speed (2.2 MHz)*/
#define PD2_MF_16_2M_14L        PD_BRD + 0x125
#define PD2_MF_16_2M_14H        PD_BRD + 0x126
#define PD2_MF_64_2M_14L        PD_BRD + 0x127
#define PD2_MF_64_2M_14H        PD_BRD + 0x128

/* 16-bit multifunction high speed (500 kHz)*/
#define PD2_MF_16_500_16L         PD_BRD + 0x129
#define PD2_MF_16_500_16H         PD_BRD + 0x12A
#define PD2_MF_64_500_16L         PD_BRD + 0x12B
#define PD2_MF_64_500_16H         PD_BRD + 0x12C

/* PowerDAQ II SSH section ---------------------*/
/* 14-bit multifunction SSH low speed (500 kHz)*/
#define PD2_MFS_4_500_14         PD_BRD + 0x12D
#define PD2_MFS_8_500_14         PD_BRD + 0x12E
#define PD2_MFS_4_500_14DG       PD_BRD + 0x12F
#define PD2_MFS_4_500_14H        PD_BRD + 0x130
#define PD2_MFS_8_500_14DG       PD_BRD + 0x131
#define PD2_MFS_8_500_14H        PD_BRD + 0x132

/* 14-bit multifunction SSH medium speed (800 kHz)*/
#define PD2_MFS_4_800_14         PD_BRD + 0x133
#define PD2_MFS_8_800_14         PD_BRD + 0x134
#define PD2_MFS_4_800_14DG       PD_BRD + 0x135
#define PD2_MFS_4_800_14H        PD_BRD + 0x136
#define PD2_MFS_8_800_14DG       PD_BRD + 0x137
#define PD2_MFS_8_800_14H        PD_BRD + 0x138

/* 12-bit multifunction SSH high speed (1.25 MHz)*/
#define PD2_MFS_4_1M_12          PD_BRD + 0x139
#define PD2_MFS_8_1M_12          PD_BRD + 0x13A
#define PD2_MFS_4_1M_12DG        PD_BRD + 0x13B
#define PD2_MFS_4_1M_12H         PD_BRD + 0x13C
#define PD2_MFS_8_1M_12DG        PD_BRD + 0x13D
#define PD2_MFS_8_1M_12H         PD_BRD + 0x13E

/* 14-bit multifunction SSH high speed (2.2 MHz)*/
#define PD2_MFS_4_2M_14          PD_BRD + 0x13F
#define PD2_MFS_8_2M_14          PD_BRD + 0x140
#define PD2_MFS_4_2M_14DG        PD_BRD + 0x141
#define PD2_MFS_4_2M_14H         PD_BRD + 0x142
#define PD2_MFS_8_2M_14DG        PD_BRD + 0x143
#define PD2_MFS_8_2M_14H         PD_BRD + 0x144

/* 16-bit multifunction SSH high speed (333 kHz)*/
#define PD2_MFS_4_300_16         PD_BRD + 0x145
#define PD2_MFS_8_300_16         PD_BRD + 0x146
#define PD2_MFS_4_300_16DG       PD_BRD + 0x147
#define PD2_MFS_4_300_16H        PD_BRD + 0x148
#define PD2_MFS_8_300_16DG       PD_BRD + 0x149
#define PD2_MFS_8_300_16H        PD_BRD + 0x14A

/* DIO series*/
#define PD2_DIO_64               PD_BRD + 0x14B
#define PD2_DIO_128              PD_BRD + 0x14C
#define PD2_DIO_256              PD_BRD + 0x14D

/* AO series*/
#define PD2_AO_8_16              PD_BRD + 0x14E
#define PD2_AO_16_16             PD_BRD + 0x14F
#define PD2_AO_32_16             PD_BRD + 0x150
#define PD2_AO_96_16             PD_BRD + 0x151

#define PD2_AO_32_16HC           PD_BRD + 0x152
#define PD2_AO_32_16HV           PD_BRD + 0x153
#define PD2_AO_96_16HS           PD_BRD + 0x154
#define PD2_AO_8_16HSG           PD_BRD + 0x155
#define PD2_AO_R5                PD_BRD + 0x156
#define PD2_AO_R6                PD_BRD + 0x157
#define PD2_AO_R7                PD_BRD + 0x158

/* extended DIO series*/
#define PD2_DIO_64CE             PD_BRD + 0x159
#define PD2_DIO_128CE            PD_BRD + 0x15A

#define PD2_DIO_64ST             PD_BRD + 0x15B
#define PD2_DIO_128ST            PD_BRD + 0x15C

#define PD2_DIO_64ER             PD_BRD + 0x15D
#define PD2_DIO_128ER            PD_BRD + 0x15E

#define PD2_DIO_64CT             PD_BRD + 0x15F
#define PD2_DIO_128CT            PD_BRD + 0x160

#define PD2_DIO_64TS             PD_BRD + 0x161
#define PD2_DIO_128TS            PD_BRD + 0x162

#define PD2_DIO_R2               PD_BRD + 0x163
#define PD2_DIO_R3               PD_BRD + 0x164
#define PD2_DIO_R4               PD_BRD + 0x165
#define PD2_DIO_R5               PD_BRD + 0x166
#define PD2_DIO_R6               PD_BRD + 0x167
#define PD2_DIO_R7               PD_BRD + 0x168

/* 16-bit multifunction SSH high speed (500 kHz)*/
#define PD2_MFS_4_500_16         PD_BRD + 0x169
#define PD2_MFS_8_500_16         PD_BRD + 0x16A
#define PD2_MFS_4_500_16DG       PD_BRD + 0x16B
#define PD2_MFS_4_500_16H        PD_BRD + 0x16C
#define PD2_MFS_8_500_16DG       PD_BRD + 0x16D
#define PD2_MFS_8_500_16H        PD_BRD + 0x16E

/* 16-bit multifunction (100 kHz)*/
#define PD2_MF_16_150_16L        PD_BRD + 0x16F
#define PD2_MF_16_150_16H        PD_BRD + 0x170
#define PD2_MF_64_150_16L        PD_BRD + 0x171
#define PD2_MF_64_150_16H        PD_BRD + 0x172

/* 12-bit multifunction (3 MHz)*/
#define PD2_MF_16_3M_12H         PD_BRD + 0x173
#define PD2_MF_64_3M_12H         PD_BRD + 0x174
#define PD2_MF_16_3M_12L         PD_BRD + 0x175
#define PD2_MF_64_3M_12L         PD_BRD + 0x176

/* 16-bit multifunction (4 MHz)*/
#define PD2_MF_16_4M_16H         PD_BRD + 0x177
#define PD2_MF_64_4M_16H         PD_BRD + 0x178

/* reserved for MF/MFS boards*/
#define PD2_MF_R6                PD_BRD + 0x179
#define PD2_MF_R7                PD_BRD + 0x17A
#define PD2_MF_R8                PD_BRD + 0x17B
#define PD2_MF_R9                PD_BRD + 0x17C
#define PD2_MF_RA                PD_BRD + 0x17D
#define PD2_MF_RB                PD_BRD + 0x17E
#define PD2_MF_RC                PD_BRD + 0x17F
#define PD2_MF_RD                PD_BRD + 0x180
#define PD2_MF_RE                PD_BRD + 0x181
#define PD2_MF_RF                PD_BRD + 0x182

/* reserved for LAB board*/
#define PDL_MF_16_50_16          PD_BRD + 0x183
#define PDL_MF_16_333_16         PD_BRD + 0x184
#define PDL_MF_16_160_16         PD_BRD + 0x185
#define PDL_MF_16_3              PD_BRD + 0x186

#define PDL_DIO_64               PD_BRD + 0x187
#define PDL_DIO_64ST             PD_BRD + 0x188
#define PDL_DIO_64CT             PD_BRD + 0x189
#define PDL_DIO_64TS             PD_BRD + 0x18A

#define PDL_AO_64                PD_BRD + 0x18B
#define PDL_AO_64_1              PD_BRD + 0x18C
#define PDL_AO_64_2              PD_BRD + 0x18D
#define PDL_AO_64_3              PD_BRD + 0x18E


/* PDXI series -------------------------*/
/* 14-bit multifunction low speed (400 kHz)*/
#define PDXI_MF_16_400_14L        PD_BRD + 0x211
#define PDXI_MF_16_400_14H        PD_BRD + 0x212
#define PDXI_MF_64_400_14L        PD_BRD + 0x213
#define PDXI_MF_64_400_14H        PD_BRD + 0x214

/* 14-bit multifunction medium speed (800 kHz)*/
#define PDXI_MF_16_800_14L        PD_BRD + 0x215
#define PDXI_MF_16_800_14H        PD_BRD + 0x216
#define PDXI_MF_64_800_14L        PD_BRD + 0x217
#define PDXI_MF_64_800_14H        PD_BRD + 0x218

/* 12-bit multifunction high speed (1.25 MHz)*/
#define PDXI_MF_16_1M_12L         PD_BRD + 0x219
#define PDXI_MF_16_1M_12H         PD_BRD + 0x21A
#define PDXI_MF_64_1M_12L         PD_BRD + 0x21B
#define PDXI_MF_64_1M_12H         PD_BRD + 0x21C

/* 16-bit multifunction (50 kHz)*/
#define PDXI_MF_16_50_16L        PD_BRD + 0x21D
#define PDXI_MF_16_50_16H        PD_BRD + 0x21E
#define PDXI_MF_64_50_16L        PD_BRD + 0x21F
#define PDXI_MF_64_50_16H        PD_BRD + 0x220

/* 16-bit multifunction (333 kHz)*/
#define PDXI_MF_16_333_16L        PD_BRD + 0x221
#define PDXI_MF_16_333_16H        PD_BRD + 0x222
#define PDXI_MF_64_333_16L        PD_BRD + 0x223
#define PDXI_MF_64_333_16H        PD_BRD + 0x224

/* 14-bit multifunction high speed (2.2 MHz)*/
#define PDXI_MF_16_2M_14L        PD_BRD + 0x225
#define PDXI_MF_16_2M_14H        PD_BRD + 0x226
#define PDXI_MF_64_2M_14L        PD_BRD + 0x227
#define PDXI_MF_64_2M_14H        PD_BRD + 0x228

/* 16-bit multifunction high speed (500 kHz)*/
#define PDXI_MF_16_500_16L         PD_BRD + 0x229
#define PDXI_MF_16_500_16H         PD_BRD + 0x22A
#define PDXI_MF_64_500_16L         PD_BRD + 0x22B
#define PDXI_MF_64_500_16H         PD_BRD + 0x22C

/* PowerDAQ II SSH section ---------------------*/
/* 14-bit multifunction SSH low speed (500 kHz)*/
#define PDXI_MFS_4_500_14         PD_BRD + 0x22D
#define PDXI_MFS_8_500_14         PD_BRD + 0x22E
#define PDXI_MFS_4_500_14DG       PD_BRD + 0x22F
#define PDXI_MFS_4_500_14H        PD_BRD + 0x230
#define PDXI_MFS_8_500_14DG       PD_BRD + 0x231
#define PDXI_MFS_8_500_14H        PD_BRD + 0x232

/* 14-bit multifunction SSH medium speed (800 kHz)*/
#define PDXI_MFS_4_800_14         PD_BRD + 0x233
#define PDXI_MFS_8_800_14         PD_BRD + 0x234
#define PDXI_MFS_4_800_14DG       PD_BRD + 0x235
#define PDXI_MFS_4_800_14H        PD_BRD + 0x236
#define PDXI_MFS_8_800_14DG       PD_BRD + 0x237
#define PDXI_MFS_8_800_14H        PD_BRD + 0x238

/* 12-bit multifunction SSH high speed (1.25 MHz)*/
#define PDXI_MFS_4_1M_12          PD_BRD + 0x239
#define PDXI_MFS_8_1M_12          PD_BRD + 0x23A
#define PDXI_MFS_4_1M_12DG        PD_BRD + 0x23B
#define PDXI_MFS_4_1M_12H         PD_BRD + 0x23C
#define PDXI_MFS_8_1M_12DG        PD_BRD + 0x23D
#define PDXI_MFS_8_1M_12H         PD_BRD + 0x23E

/* 14-bit multifunction SSH high speed (2.2 MHz)*/
#define PDXI_MFS_4_2M_14          PD_BRD + 0x23F
#define PDXI_MFS_8_2M_14          PD_BRD + 0x240
#define PDXI_MFS_4_2M_14DG        PD_BRD + 0x241
#define PDXI_MFS_4_2M_14H         PD_BRD + 0x242
#define PDXI_MFS_8_2M_14DG        PD_BRD + 0x243
#define PDXI_MFS_8_2M_14H         PD_BRD + 0x244

/* 16-bit multifunction SSH high speed (333 kHz)*/
#define PDXI_MFS_4_300_16         PD_BRD + 0x245
#define PDXI_MFS_8_300_16         PD_BRD + 0x246
#define PDXI_MFS_4_300_16DG       PD_BRD + 0x247
#define PDXI_MFS_4_300_16H        PD_BRD + 0x248
#define PDXI_MFS_8_300_16DG       PD_BRD + 0x249
#define PDXI_MFS_8_300_16H        PD_BRD + 0x24A

/* DIO series*/
#define PDXI_DIO_64               PD_BRD + 0x24B
#define PDXI_DIO_128              PD_BRD + 0x24C
#define PDXI_DIO_256              PD_BRD + 0x24D

/* AO series*/
#define PDXI_AO_8_16              PD_BRD + 0x24E
#define PDXI_AO_16_16             PD_BRD + 0x24F
#define PDXI_AO_32_16             PD_BRD + 0x250

#define PDXI_AO_R0                PD_BRD + 0x251
#define PDXI_AO_R1                PD_BRD + 0x252
#define PDXI_AO_R2                PD_BRD + 0x253
#define PDXI_AO_R3                PD_BRD + 0x254
#define PDXI_AO_R4                PD_BRD + 0x255
#define PDXI_AO_R5                PD_BRD + 0x256
#define PDXI_AO_R6                PD_BRD + 0x257
#define PDXI_AO_R7                PD_BRD + 0x258

/* extended DIO series*/
#define PDXI_DIO_64CE             PD_BRD + 0x259
#define PDXI_DIO_128CE            PD_BRD + 0x25A

#define PDXI_DIO_64ST             PD_BRD + 0x25B
#define PDXI_DIO_128ST            PD_BRD + 0x25C

#define PDXI_DIO_64HS             PD_BRD + 0x25D
#define PDXI_DIO_128HS            PD_BRD + 0x25E

#define PDXI_DIO_64CT             PD_BRD + 0x25F
#define PDXI_DIO_128CT            PD_BRD + 0x260

#define PDXI_DIO_64TS             PD_BRD + 0x261
#define PDXI_DIO_128TS            PD_BRD + 0x262

#define PDXI_DIO_R2               PD_BRD + 0x263
#define PDXI_DIO_R3               PD_BRD + 0x264
#define PDXI_DIO_R4               PD_BRD + 0x265
#define PDXI_DIO_R5               PD_BRD + 0x266
#define PDXI_DIO_R6               PD_BRD + 0x267
#define PDXI_DIO_R7               PD_BRD + 0x268

/* 16-bit multifunction SSH high speed (500 kHz)*/
#define PDXI_MFS_4_500_16         PD_BRD + 0x269
#define PDXI_MFS_8_500_16         PD_BRD + 0x26A
#define PDXI_MFS_4_500_16DG       PD_BRD + 0x26B
#define PDXI_MFS_4_500_16H        PD_BRD + 0x26C
#define PDXI_MFS_8_500_16DG       PD_BRD + 0x26D
#define PDXI_MFS_8_500_16H        PD_BRD + 0x26E

/* 16-bit multifunction (200 kHz)*/
#define PDXI_MF_16_200_16L        PD_BRD + 0x26F
#define PDXI_MF_16_200_16H        PD_BRD + 0x270
#define PDXI_MF_64_200_16L        PD_BRD + 0x271
#define PDXI_MF_64_200_16H        PD_BRD + 0x272

/* 12-bit multifunction (3 MHz)*/
#define PDXI_MF_16_3M_12H         PD_BRD + 0x273
#define PDXI_MF_64_3M_12H         PD_BRD + 0x274
#define PDXI_MF_16_3M_12L         PD_BRD + 0x275
#define PDXI_MF_64_3M_12L         PD_BRD + 0x276

/* 16-bit multifunction (4 MHz)*/
#define PDXI_MF_16_4M_16H         PD_BRD + 0x277
#define PDXI_MF_64_4M_16H         PD_BRD + 0x278

/* reserved for MF/MFS boards*/
#define PDXI_MF_R6                PD_BRD + 0x279
#define PDXI_MF_R7                PD_BRD + 0x27A
#define PDXI_MF_R8                PD_BRD + 0x27B
#define PDXI_MF_R9                PD_BRD + 0x27C
#define PDXI_MF_RA                PD_BRD + 0x27D
#define PDXI_MF_RB                PD_BRD + 0x27E
#define PDXI_MF_RC                PD_BRD + 0x27F
#define PDXI_MF_RD                PD_BRD + 0x280
#define PDXI_MF_RE                PD_BRD + 0x281
#define PDXI_MF_RF                PD_BRD + 0x282

/* reserved for LAB board*/
#define PDXL_MF_16_50_16          PD_BRD + 0x283
#define PDXL_MF_16_100_16         PD_BRD + 0x284
#define PDXL_MF_16_160_16         PD_BRD + 0x285
#define PDXL_MF_16_3              PD_BRD + 0x286

#define PDXL_DIO_64               PD_BRD + 0x287
#define PDXL_DIO_64ST             PD_BRD + 0x288
#define PDXL_DIO_64CT             PD_BRD + 0x289
#define PDXL_DIO_64TS             PD_BRD + 0x28A

#define PDXL_AO_64                PD_BRD + 0x28B
#define PDXL_AO_64_1              PD_BRD + 0x28C
#define PDXL_AO_64_2              PD_BRD + 0x28D
#define PDXL_AO_64_3              PD_BRD + 0x28E


#define PD_BRD_LST               PDXL_AO_64_3 - 0x101 - PD_BRD

#ifdef __cplusplus
}
#endif


#endif /* __POWERDAQ32_H__ */

