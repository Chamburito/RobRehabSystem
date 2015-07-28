/* 
 * File:   AXIS.h
 * Author: GUILHERME FERNANDES
 *
 * ESTA FUNÇÃO REALIZA O CONTROLE DE UM EIXO DA EPOS
 * 
 * Created on 26 de Janeiro de 2012, 19:34
 */

#ifndef AXIS_H
#define	AXIS_H

#include "axis_interface.h"
#include "ni_can_epos_axis/axis_interface.h"
#include "shm_axis/axis_interface.h"

#ifdef WIN32
  #include "../timing_windows.h"
#else
  #include "../timing_unix.h"
#endif

#ifdef _CVI_
  #include <ansi_c.h>
#else
  #include <stdio.h>
  #include <string.h>
  #include <stdlib.h>
  #include <malloc.h>
#endif

#include <stdbool.h>

#include "async_debug.h"

typedef struct _MotorDrive
{
  AxisInterface* interface;
  int interfaceID;
  double measuresList[ AXIS_MEASURES_NUMBER ];
  double measureRatiosList[ AXIS_MEASURES_NUMBER ];
}
MotorDrive;

typedef struct _Motor
{
  MotorDrive* drive;
  double setpoint;
  int operationMode;
}
Motor;

Motor* Motor_Connect( int );
MotorDrive* MotorDrive_Connect( int );
extern inline void Motor_Enable( Motor* );
extern inline void Motor_Disable( Motor* );
void Motor_Disconnect( Motor* );
void MotorDrive_Disconnect( MotorDrive* );
extern inline bool Motor_IsActive( Motor* );
extern inline double MotorDrive_GetMeasure( MotorDrive*, enum AxisMeasures );
extern inline void MotorDrive_SetMeasureRatio( MotorDrive*, enum AxisMeasures, double );
extern inline void Motor_SetSetpoint( Motor*, double );
extern inline void MotorDrive_Reset( MotorDrive* );
void MotorDrive_Read( MotorDrive* );
void Motor_Write( Motor* );
void Motor_SetOperationMode( Motor*, enum OperationModes );

// Create CAN controlled DC motor handle
Motor* Motor_Connect( int connectionData ) 
{
  DEBUG_EVENT( 0, "created motor with network index %u", connectionData ); 
  
  Motor* motor = (Motor*) malloc( sizeof(Motor) );
  
  motor->setpoint = 0.0;
  motor->operationMode = AXIS_OP_MODES_POSITION;
  
  if( (motor->drive = MotorDrive_Connect( connectionData )) == NULL )
  {
    Motor_Disconnect( motor );
    return NULL;
  }
  
  Motor_Disable( motor );

  DEBUG_EVENT( 0, "created motor with network index %u", connectionData );
  
  return motor;
}

MotorDrive* MotorDrive_Connect( int connectionData )
{
  DEBUG_EVENT( 0, "creating drive with network index %u", connectionData );
  
  MotorDrive* drive = (MotorDrive*) malloc( sizeof(MotorDrive) );
  
  for( size_t dimensionIndex = 0; dimensionIndex < AXIS_MEASURES_NUMBER; dimensionIndex++ )
  {
    drive->measuresList[ dimensionIndex ] = 0.0;
    drive->measureRatiosList[ dimensionIndex ] = 1.0;
  }
  
  drive->interface = &AxisCANEPOSInterface;
  drive->interfaceID = drive->interface->Connect( connectionData );
  
  DEBUG_EVENT( 0, "created drive with network index %u", connectionData );
  
  return drive;
}

void Motor_Disconnect( Motor* motor )
{
  if( motor != NULL )
  {
    Motor_Disable( motor );
    
    MotorDrive_Disconnect( motor->drive );
    motor->drive = NULL;

    free( motor );
    motor = NULL;
  }
}

void MotorDrive_Disconnect( MotorDrive* drive )
{
  if( drive != NULL )
  {
    drive->interface->Disconnect( drive->interfaceID );

    free( drive );
    drive = NULL;
  }
}

extern inline bool Motor_IsActive( Motor* motor )
{
  return motor->drive->interface->IsEnabled( motor->drive->interfaceID );
}

extern inline bool MotorDrive_HasError( MotorDrive* drive )
{
  return drive->interface->HasError( drive->interfaceID ); 
}

extern inline void Motor_Enable( Motor* motor )
{
  motor->drive->interface->Enable( motor->drive->interfaceID );
}

extern inline void Motor_Disable( Motor* motor )
{
  motor->drive->interface->Disable( motor->drive->interfaceID );
}

double MotorDrive_GetMeasure( MotorDrive* drive, enum AxisMeasures index )
{
  if( index >= AXIS_MEASURES_NUMBER )
  {
    ERROR_EVENT( "invalid dimension index: %d\n", index );
    return 0;
  }
      
  return drive->measuresList[ index ];
}

void MotorDrive_SetMeasureRatio( MotorDrive* drive, enum AxisMeasures index, double conversionRatio )
{
  if( index >= AXIS_MEASURES_NUMBER )
  {
    ERROR_EVENT( "invalid dimension index: %d\n", index );
    return 0;
  }
  
  if( conversionRatio != 0.0 ) drive->measureRatiosList[ index ] = conversionRatio;
}

extern inline void Motor_SetSetpoint( Motor* motor, double value )
{
  motor->setpoint = value;
}

extern inline void MotorDrive_Reset( MotorDrive* drive )
{
  drive->interface->Reset( drive->interfaceID );
}

void MotorDrive_Read( MotorDrive* drive )
{
  if( drive->interface->ReadMeasures( drive->interfaceID, drive->measuresList ) )
  {
    for( size_t dimensionIndex = 0; dimensionIndex < AXIS_MEASURES_NUMBER; dimensionIndex++ )
      drive->measuresList[ dimensionIndex ] /= drive->measureRatiosList[ dimensionIndex ];
  }
}

void Motor_Write( Motor* motor )
{
  static double setpointsList[ AXIS_OP_MODES_NUMBER ];

  setpointsList[ motor->operationMode ] = ( motor->setpoint * motor->drive->measureRatiosList[ motor->operationMode ] );
  
  motor->drive->interface->WriteControl( motor->drive->interfaceID, setpointsList );
}

extern inline void Motor_SetOperationMode( Motor* motor, enum OperationMode mode )
{
  motor->operationMode = mode;
  motor->drive->interface->SetOperationMode( motor->drive->interfaceID, motor->operationMode );
}


#endif	/* AXIS_H */

