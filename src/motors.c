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


#include "configuration.h"

#include "signal_io/interface.h"

#include "debug/async_debug.h"

#include "motors.h"
      
struct _MotorData
{
  DECLARE_MODULE_INTERFACE_REF( SIGNAL_IO_INTERFACE );
  int interfaceID;
  unsigned int outputChannel;
  double outputGain, outputBaseGain;
  double outputOffset;
};

DEFINE_NAMESPACE_INTERFACE( Motors, MOTOR_INTERFACE )


Motor Motors_Init( const char* configFileName )
{
  static char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Initializing Axis Motor %s", configFileName );
  
  Motor newMotor = NULL;
  
  sprintf( filePath, "motors/%s", configFileName );
  int configFileID = Configuration.LoadConfigFile( filePath );
  if( configFileID != DATA_INVALID_ID )
  {
    newMotor = (Motor) malloc( sizeof(MotorData) );
    memset( newMotor, 0, sizeof(MotorData) );

    bool loadSuccess = true;
    sprintf( filePath, "signal_io/%s", Configuration.GetIOHandler()->GetStringValue( configFileID, "", "output_interface.type" ) );
    LOAD_MODULE_IMPLEMENTATION( SIGNAL_IO_INTERFACE, filePath, newMotor, &loadSuccess );
    if( loadSuccess )
    {
      newMotor->interfaceID = newMotor->InitTask( Configuration.GetIOHandler()->GetStringValue( configFileID, "", "output_interface.id" ) );
      if( newMotor->interfaceID != SIGNAL_IO_TASK_INVALID_ID ) 
      {
        newMotor->outputChannel = (unsigned int) Configuration.GetIOHandler()->GetIntegerValue( configFileID, -1, "output_interface.channel" );
        DEBUG_PRINT( "trying to aquire channel %u from interface %d", newMotor->outputChannel, newMotor->interfaceID );
        loadSuccess = newMotor->AquireOutputChannel( newMotor->interfaceID, newMotor->outputChannel );
      }
      else loadSuccess = false;
    }

    newMotor->outputBaseGain = Configuration.GetIOHandler()->GetRealValue( configFileID, 1.0, "output_gain.multiplier" );
    newMotor->outputBaseGain /= Configuration.GetIOHandler()->GetRealValue( configFileID, 1.0, "output_gain.divisor" );
    newMotor->outputGain = newMotor->outputBaseGain;

    Configuration.GetIOHandler()->UnloadData( configFileID );

    if( !loadSuccess )
    {
      Motors_End( newMotor );
      return NULL;
    }

    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Axis Motor %s initialized", configFileName );
  }
  else
    DEBUG_PRINT( "configuration parser for axis sensor %s not available", configFileName );
  
  return newMotor;
}

void Motors_End( Motor motor )
{
  if( motor == NULL ) return;
  
  motor->EndTask( motor->interfaceID );
  
  free( motor );
}

void Motors_Enable( Motor motor )
{
  if( motor == NULL ) return;
  
  motor->Reset( motor->interfaceID );
  motor->EnableOutput( motor->interfaceID, true );
}

void Motors_Disable( Motor motor )
{
  if( motor == NULL ) return;
  
  motor->EnableOutput( motor->interfaceID, false );
}

void Motors_Reset( Motor motor )
{
  if( motor == NULL ) return;
  
  motor->Reset( motor->interfaceID );
}

void Motors_SetGain( Motor motor, double gainFactor )
{
  if( motor == NULL ) return;
  
  motor->outputGain = gainFactor * motor->outputBaseGain;
}

void Motors_SetOffset( Motor motor, double offset )
{
  if( motor == NULL ) return;
  
  motor->outputOffset = offset;
}

bool Motors_IsEnabled( Motor motor )
{
  if( motor == NULL ) return false;
  
  return motor->IsOutputEnabled( motor->interfaceID );
}

bool Motors_HasError( Motor motor )
{
  if( motor == NULL ) return false;
  
  return motor->HasError( motor->interfaceID );
}

void Motors_WriteControl( Motor motor, double setpoint )
{
  if( motor == NULL ) return;
  
  //if( setpoint > 1.0 ) setpoint = 1.0;
  //else if( setpoint < -1.0 ) setpoint = -1.0;
  
  //DEBUG_PRINT( "motor output: %g * %.5f = %g", motor->outputGain, setpoint, setpoint * motor->outputGain );
  
  setpoint = ( setpoint + motor->outputOffset ) * motor->outputGain;
  
  motor->Write( motor->interfaceID, motor->outputChannel, setpoint );
}
