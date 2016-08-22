/* 
 *  pci4eHelper.h
 */

#ifndef PCI4E_HELPER_H
#define PCI4E_HELPER_H


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <sys/timeb.h>
#include <linux/types.h>

//#include "pci4e.h"

// ****** Error Codes ***************/
#define NO_CARDS_DETECTED		-1	
#define INVALID_DEVICE_NUMBER		-2
#define DEVICE_NOT_OPEN			-3
#define INVALID_ENCODER_NUMBER		-4
#define INVALID_REGISTER_NUMBER		-5	
#define INVALID_SLOT_NUMBER		-6
#define INVALID_QUADRATURE_MODE		-7
#define FEATURE_NOT_SUPPORTED		-8
#define INVALID_COUNTER_MODE		-9
#define FAILED_TO_OPEN_DRIVER		-10
#define CONTROL_MODE_IS_ZERO		-11
#define INCORRECT_WINDRIVER_VERSION	-12
#define OVERRUN_ERROR_DETECTED		-13
#define FRAMING_ERROR_DETECTED		-14
#define RX_BUFFER_FULL			-15
#define INVALID_PARAMETER		-16
#define FATAL_ERROR			-17

//****** REGISTERS **************************/
#define PRESET_REGISTER			0
#define OUTPUT_LATCH_REGISTER		1
#define MATCH_REGISTER			2
#define CONTROL_REGISTER		3
#define STATUS_REGISTER			4
#define RESET_CHANNEL_REGISTER		5
#define TRANSFER_PRESET_REGISTER	6
#define CMD_REGISTER			7
#define TIMESTAMP_OUTPUT_LATCH_REGISTER	15
#define TIMESTAMP_REGISTER		23
#define INPUT_REGISTER			30
#define OUTPUT_REGISTER			31
#define RX_CONTROL_REGISTER		32
#define RX_STATUS_REGISTER		33
#define TX_CONTROL_REGISTER		34
#define TX_STATUS_REGISTER		35
#define TX_DATA_REGISTER		36
#define PORT_SELECT_REGISTER		37
#define FIFO2_STATUS_CONTROL_REGISTER	38
#define FIFO2_READ_DATA_REGISTER	39

//****** Misc Constants *********************/
#define MAX_DEVICES	2 		// maximum number of PCI-4E devices.
#define MAX_DEV_NM_SZ	11 		// maximum size of a PCI-4E device name.
#define MAX_ENCODERS 4			// max number of encoders per device.
#define CMD_CHANGE_SLEEP_TIME 65	// milliseconds.
#define IO_WRITE_CYCLE_SLEEP_TIME 1	// milliseconds.

//****** ROM Version Features ***************/
#define PB16_SUPPORTED			0x70
#define UART_SUPPORTED			0x80
#define CHANNEL_BUFFER_SUPPORTED	0x84

//****** IO Signals *************************/
#define SETUP_OUTPUT			0xC100
#define WRITE_OUTPUT			0x4100
#define HOLD_OUTPUT			0xC100
#define SETUP_INPUT			0xE100
#define READ_INPUT			0xA100
#define SET_INACTIVE			0xFF00
#define RESET				0xFE00

//****** UART Signals ***********************/
#define RX_READ				0x1
#define RX_INIT				0x2
#define RX_EMPTY			0x200
#define RX_FULL				0x100
#define OVERRUN_ERROR			0x4
#define FRAMING_ERROR			0x2
#define TX_EMPTY			0x1
#define TX_WRITE			0x1
#define NOT_RX_READ			0xFFFFFFFE
#define NOT_RX_INIT			0xFFFFFFFD
#define NOT_TX_WRITE			0xFFFFFFFE
#define CLEAR_ERROR_0			0xFFFFFFF9 	// Bit Overrun Err = 2, Framing Err = 1
#define CLEAR_ERROR_1			0x6		// Bit Overrun Err = 2, Framing Err = 1
#define CLEAR_DATA			0xFFFFFF00
#define PORT_MASK			0xF
#define NOT_PORT_MASK			0xFFFFFFF0
#define RX_DATA_MASK			0xFF

//****** Channel Buffer Signals ***************/
#define COUNT_MASK			0xFFFFFF
#define STATUS_MASK			0xFF
#define DISABLE_BUFFERING		0xFFFFFEFF
#define ENABLE_BUFFERING		0x100

typedef struct 
{
  int fd;
  int iSlotNo;	
  char devname[ MAX_DEV_NM_SZ ];
  short version; 
} 
DEVICE_DATA;

typedef struct ChannelBufferRecord
{
  long Time;
  long ChannelData[ 4 ];
  char ChannelStatus[ 4 ];
}
ChannelBufferRecord;

#ifdef __cplusplus 
extern "C"{ 
#endif 

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


#ifdef __cplusplus 
} 
#endif 

#endif // PCI4E_HELPER_H
