////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016 Leonardo José Consoni                                  //
//                                                                            //
//  This file is part of RobRehabSystem.                                      //
//                                                                            //
//  RobRehabSystem is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobRehabSystem is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "signal_io/interface.h"
#include "signal_io/pci4e/pci4e_helper.h"

#include "klib/khash.h"

#include "debug/async_debug.h"

#define REG4E( i, REG ) ( (i) * 8 + REG )

#define ENCODERS_NUMBER 4

static const size_t AQUISITION_BUFFER_LENGTH = 1;
static const long ENCODER_MAX = ( 1 << 24 ) / 2;

typedef struct _IOBoardData
{
  unsigned int inputChannelUsesList[ ENCODERS_NUMBER ];
  double measuresList[ ENCODERS_NUMBER ];
  bool isReading;
}
IOBoardData;

typedef IOBoardData* IOBoard;

IOBoard boardsList = NULL;
short boardsNumber = 0;
int boardUsesCount = 0;

DECLARE_MODULE_INTERFACE( SIGNAL_IO_INTERFACE ) 

static inline bool IsTaskStillUsed( IOBoard );

int InitTask( const char* boardName )
{
  if( boardsNumber == 0 ) 
  {
    if( PCI4E_Initialize( &boardsNumber ) < 0 )
      return SIGNAL_IO_TASK_INVALID_ID;
    boardsList = (IOBoard) calloc( (size_t) boardsNumber, sizeof(IOBoardData) );
  }
  
  unsigned long boardID = strtoul( boardName, NULL, 0 );
  
  if( boardID >= (unsigned long) boardsNumber )
    return SIGNAL_IO_TASK_INVALID_ID;
  
  for( size_t encoderIndex = 0; encoderIndex < ENCODERS_NUMBER; encoderIndex++ )
  {
    PCI4E_SetCount( boardID, encoderIndex, 0 );
    if( PCI4E_WriteRegister( boardID, REG4E( encoderIndex, CONTROL_REGISTER ), 0x7C000 ) < 0 )
      return SIGNAL_IO_TASK_INVALID_ID;
    if( PCI4E_WriteRegister( boardID, REG4E( encoderIndex, PRESET_REGISTER ), 2 * ENCODER_MAX - 1 ) < 0 )
      return SIGNAL_IO_TASK_INVALID_ID;
  }
  
  boardUsesCount++;
  
  DEBUG_PRINT( "board ID %lu (%d) available", boardID, (int) boardID );
  
  return (int) boardID;
}

void EndTask( int boardID )
{
  if( boardID >= boardsNumber ) return;
  
  IOBoard board = (IOBoard) &(boardsList[ boardID ]);
  
  board->isReading = false;
  
  if( IsTaskStillUsed( board ) ) return;
  
  if( --boardUsesCount <= 0 )
  {
    free( boardsList );
    boardsList = NULL;
    boardsNumber = 0;
  }
}

size_t GetMaxInputSamplesNumber( int boardID )
{
  if( boardID >= boardsNumber ) return 0;
  
  return 1;
}

size_t Read( int boardID, unsigned int channel, double* ref_value )
{
  if( boardID >= boardsNumber ) return 0;
  
  if( channel >= ENCODERS_NUMBER ) return 0;
  
  long encoderCount;
  if( PCI4E_GetCount( boardID, channel, &encoderCount ) < 0 ) return 0;
  if( encoderCount > ENCODER_MAX ) encoderCount -= 2 * ENCODER_MAX;
  
  *ref_value = ( (double) encoderCount ) / ENCODER_MAX;

  return 1;
}

bool HasError( int boardID )
{
  if( boardID >= boardsNumber ) return false;
  


  return false;
}

void Reset( int boardID )
{
  if( boardID >= boardsNumber ) return;
  
  for( size_t encoderIndex = 0; encoderIndex < ENCODERS_NUMBER; encoderIndex++ )
  {
    PCI4E_WriteRegister( boardID, REG4E( encoderIndex, RESET_CHANNEL_REGISTER ), 0 );
    PCI4E_WriteRegister( boardID, REG4E( encoderIndex, CONTROL_REGISTER ), 0x7C000 );
    PCI4E_WriteRegister( boardID, REG4E( encoderIndex, PRESET_REGISTER ), 2 * ENCODER_MAX - 1 );
  }
}

bool AquireInputChannel( int boardID, unsigned int channel )
{
  if( boardID >= boardsNumber ) return false;
  
  if( channel >= ENCODERS_NUMBER ) return false;
  
  IOBoard board = (IOBoard) &(boardsList[ boardID ]);
  
  if( board->inputChannelUsesList[ channel ] >= SIGNAL_INPUT_CHANNEL_MAX_USES ) return false;
  
  board->inputChannelUsesList[ channel ]++;
  
  return true;
}

void ReleaseInputChannel( int boardID, unsigned int channel )
{
  if( boardID >= boardsNumber ) return;
  
  if( channel >= ENCODERS_NUMBER ) return;
  
  IOBoard board = (IOBoard) &(boardsList[ boardID ]);
  
  if( board->inputChannelUsesList[ channel ] > 0 ) board->inputChannelUsesList[ channel ]--;
  
  if( !IsTaskStillUsed( board ) && !board->isReading ) EndTask( boardID );
}

void EnableOutput( int boardID, bool enable )
{
  return;
}

bool IsOutputEnabled( int boardID )
{
  return false;
}

bool Write( int boardID, unsigned int channel, double value )
{
  return false;
}

bool AquireOutputChannel( int boardID, unsigned int channel )
{  
  return false;
}

void ReleaseOutputChannel( int boardID, unsigned int channel )
{
  return;
}

bool IsTaskStillUsed( IOBoard board )
{
  bool isStillUsed = false;
  if( board->inputChannelUsesList != NULL )
  {
    for( size_t channel = 0; channel < ENCODERS_NUMBER; channel++ )
    {
      if( board->inputChannelUsesList[ channel ] > 0 )
      {
        isStillUsed = true;
        break;
      }
    }
  }
  
  if( board->isOutputChannelUsed ) isStillUsed = true;
  
  return isStillUsed;
}
 
