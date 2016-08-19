/* 
 *  pci4eHelper.c
 *
 */

#ifndef PCI4E_HELPER_H
#define PCI4E_HELPER_H

#include "pci4e_helper.h"

/****************************************************
|	Exported Functions... 
****************************************************/

int PCI4E_CardCount();

/******** PCI-4E Functions **********************/
int PCI4E_GetSlotNumber( short iDeviceNo, short *iSlotNo);

int PCI4E_GotoU2CommandMode( short iDeviceNo, short iEncoder );

int PCI4E_Initialize(short *piDeviceCount );

int PCI4E_ReadRegister( short iDeviceNo, short iRegister, long *plVal );

void PCI4E_Shutdown();

int PCI4E_GetCount( short iDeviceNo, short iEncoder, long *lVal );

int PCI4E_GetPresetValue( short iDeviceNo, short iEncoder, long *lVal );

int PCI4E_SetPresetValue( short iDeviceNo, short iEncoder, long lVal );

int PCI4E_GetMultiplier( short iDeviceNo, short iEncoder, short *iMode );

int PCI4E_SetMultiplier( short iDeviceNo, short iEncoder, short iMode );

int PCI4E_SetCount( short iDeviceNo, short iEncoder, long lVal );

int PCI4E_GetMatch( short iDeviceNo, short iEncoder, long *lVal );

int PCI4E_SetMatch( short iDeviceNo, short iEncoder, long lVal );

int PCI4E_GetControlMode( short iDeviceNo, short iEncoder, long *plVal );

int PCI4E_SetControlMode( short iDeviceNo, short iEncoder, long lVal );

int PCI4E_GetStatus( short iDeviceNo, short iEncoder, long *plVal );

int PCI4E_GetEnableAccumulator( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetEnableAccumulator( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetForward( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetForward( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetEnableIndex( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetEnableIndex( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetInvertIndex( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetInvertIndex( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetPresetOnIndex( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetPresetOnIndex( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetCaptureEnabled( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetCaptureEnabled( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetCounterMode( short iDeviceNo, short iEncoder, short *iMode );

int PCI4E_SetCounterMode( short iDeviceNo, short iEncoder, short iMode );

int PCI4E_GetInterruptOnZero( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetInterruptOnZero( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetInterruptOnMatch( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetInterruptOnMatch( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetInterruptOnRollover( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetInterruptOnRollover( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetInterruptOnRollunder( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetInterruptOnRollunder( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetInterruptOnIndex( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetInterruptOnIndex( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetInterruptOnIncrease( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetInterruptOnIncrease( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetInterruptOnDecrease( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetInterruptOnDecrease( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetTriggerOnZero( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetTriggerOnZero( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetTriggerOnMatch( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetTriggerOnMatch( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetTriggerOnRollover( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetTriggerOnRollover( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetTriggerOnRollunder( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetTriggerOnRollunder( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetTriggerOnIndex( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetTriggerOnIndex( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetTriggerOnIncrease( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetTriggerOnIncrease( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_GetTimeStamp( short iDeviceNo, long *plVal );

int PCI4E_GetTriggerOnDecrease( short iDeviceNo, short iEncoder, int *bVal );

int PCI4E_SetTriggerOnDecrease( short iDeviceNo, short iEncoder, int bVal );

int PCI4E_ClearCapturedStatus( short iDeviceNo, short iEncoder );

int PCI4E_GetABSPosition( short iDeviceNo, short iEncoder, long *plVal );

int PCI4E_PresetCount( short iDeviceNo, short iEncoder );

int PCI4E_ResetCount( short iDeviceNo, short iEncoder );

long PCI4E_GetDeviceID();

void PCI4E_SetDeviceID( long deviceID );
	
long PCI4E_GetVendorID();
	
void PCI4E_SetVendorID(long vendorID );

int PCI4E_GetVersion( short iDeviceNo, short *iVersion );

int PCI4E_GetROM_ID( short iDeviceNo, short *iROM_ID );

int PCI4E_ReadOutputLatch( short iDeviceNo, short iEncoder, long *plVal );

int PCI4E_ReadTimeStamp( short iDeviceNo, long *plVal );

int PCI4E_ResetTimeStamp( short iDeviceNo );

int PCI4E_ReadTimeAndCount( short iDeviceNo, short iEncoder, long *plValue,  long *plTimeStamp );

int PCI4E_ReadTimeAndCounts( short iDeviceNo, long *plValues,  long *plTimeStamp );

int PCI4E_CaptureTimeAndCount( short iDeviceNo,  short iEncoder, long *plValue,  long *plTimeStamp );

int PCI4E_CaptureTimeAndCounts( short iDeviceNo, long *plValues,  long *plTimeStamp );

// ***** Added to support PC I/O Bus Board 32-bit Control...			*****
int PCI4E_PCIOReadCycle( short iDeviceNo, short iBoardSelect, short iAddress, long *plData );

int PCI4E_PCIOResetCycle( short iDeviceNo );

int PCI4E_PCIOWriteCycle( short iDeviceNo, short iBoardSelect, short iAddress, long lDataOut );

int PCI4E_PCIOSignal( short iDeviceNo, short iBoardSelect, short iState);

void PCI4E_WaitTimer(short iTimeOut );

// ***** Added to support UART Communication Control...	*****
int PCI4E_ClearInBuffer( short iDeviceNo );

int PCI4E_GetInBufferCount( short iDeviceNo, short *piCount );

int PCI4E_GetPortNumber( short iDeviceNo, short *piPortNumber );

int PCI4E_ReceiveData( short iDeviceNo, short *piSize, unsigned char *pucData );

int PCI4E_SetPortNumber( short iDeviceNo, short iPortNumber );

int PCI4E_TransmitData( short iDeviceNo, short iSize, unsigned char *piData );

int IsTxBufferEmpty( short iDeviceNo );

void ResetUart( short iDeviceNo );

// Added to support Channel Buffering...
int PCI4E_ClearChannelBuffer( short iDeviceNo );

int PCI4E_DisableChannelBuffer( short iDeviceNo );

int PCI4E_EnableChannelBuffer( short iDeviceNo );

int PCI4E_GetChannelBufferCount( short iDeviceNo, short *piCount );

int PCI4E_ReadChannelBuffer( short iDeviceNo, short *piSize, ChannelBufferRecord *pCBR );

int PCI4E_WriteRegister( short iDeviceNo, short iRegister, long lVal );


#endif // PCI4E_HELPER_H
