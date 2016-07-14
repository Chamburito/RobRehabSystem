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


#include "shm_control.h"
#include "shm_joint_control.h"

//#include "optimization.h"
#include "emg_processing.h"

#include "config_parser.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

#include <stdlib.h>
#include <math.h> 

#include "robot_control/interface.h"

#define MAX_SAMPLES_NUMBER 5000

const unsigned long UPDATE_INTERVAL_MS = 5;

typedef struct _EMGJointSamplingData
{
  int jointID;
  double** muscleSignalsList;
  double* jointAnglesList;
  double* jointIDTorquesList;
  double* sampleTimesList;
  size_t musclesCount, samplesCount;
}
EMGJointSamplingData;

typedef EMGJointSamplingData* EMGJointSampler;

typedef struct _ControlData 
{
  enum ControlState currentControlState;
  EMGJointSampler* samplersList;
  char** jointNamesList;
  size_t jointsNumber;
}
ControlData;


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE ) 


Controller InitController( const char* config, const char* logDirectory )
{
  DEBUG_PRINT( "Trying to load robot control config %s", config );
  
  ConfigParsing.Init( "JSON" );

  DataLogging.SetBaseDirectory( logDirectory );
  
  int configDataID = ConfigParsing.LoadConfigString( config );
  if( configDataID != DATA_INVALID_ID )
  {
    ControlData* newController = (ControlData*) malloc( sizeof(ControlData) );
    memset( newController, 0, sizeof(ControlData) );
    
    newController->currentControlState = CONTROL_OPERATION;
  
    newController->jointsNumber = ConfigParsing.GetParser()->GetListSize( configDataID, "joints" );
    newController->samplersList = (EMGJointSampler*) calloc( newController->jointsNumber, sizeof(EMGJointSampler) );
    newController->jointNamesList = (char**) calloc( newController->jointsNumber, sizeof(char*) );
    
    for( size_t jointIndex = 0; jointIndex < newController->jointsNumber; jointIndex++ )
    {
      newController->samplersList[ jointIndex ] = (EMGJointSampler) malloc( sizeof(EMGJointSamplingData) );
      EMGJointSampler newSampler = newController->samplersList[ jointIndex ];
      
      newController->jointNamesList[ jointIndex ] = ConfigParsing.GetParser()->GetStringValue( configDataID, "", "joints.%lu", jointIndex );
      if( (newSampler->jointID = EMGProcessing.InitJoint( newController->jointNamesList[ jointIndex ] )) != EMG_JOINT_INVALID_ID )
      {
        newSampler->musclesCount = EMGProcessing.GetJointMusclesCount( newSampler->jointID );
        newSampler->muscleSignalsList = (double**) calloc( MAX_SAMPLES_NUMBER, sizeof(double*) );
        for( size_t sampleIndex = 0; sampleIndex < MAX_SAMPLES_NUMBER; sampleIndex++ )
          newSampler->muscleSignalsList[ sampleIndex ] = (double*) calloc( newSampler->musclesCount, sizeof(double) );
        newSampler->jointAnglesList = (double*) calloc( MAX_SAMPLES_NUMBER, sizeof(double) );
        newSampler->jointIDTorquesList = (double*) calloc( MAX_SAMPLES_NUMBER, sizeof(double) );
        newSampler->sampleTimesList = (double*) calloc( MAX_SAMPLES_NUMBER, sizeof(double) );

        DEBUG_PRINT( "Got new shared joint with ID %d and %u muscles", newSampler->jointID, newSampler->musclesCount );
      }
    }
    
    ConfigParsing.GetParser()->UnloadData( configDataID );

    DEBUG_PRINT( "robot control config %s loaded", config );
    
    return newController;
  }
  
  return NULL;
}

void EndController( Controller genericController )
{
  if( genericController == NULL ) return;
  
  ControlData* controller = (ControlData*) genericController;
  
  DEBUG_PRINT( "ending robot controller %p", controller );
  
  for( size_t jointIndex = 0; jointIndex < controller->jointsNumber; jointIndex++ )
  {
    DEBUG_PRINT( "ending joint control %lu", jointIndex ); 
      
    EMGJointSampler sampler = controller->samplersList[ jointIndex ];
      
    EMGProcessing.EndJoint( sampler->jointID );

    free( sampler->sampleTimesList );
    free( sampler->jointIDTorquesList );
    free( sampler->jointAnglesList );
    for( size_t sampleIndex = 0; sampleIndex < MAX_SAMPLES_NUMBER; sampleIndex++ )
      free( sampler->muscleSignalsList[ sampleIndex ] );
    free( sampler->muscleSignalsList );
    
    free( controller->jointNamesList[ jointIndex ] );
  }
  
  free( controller->jointNamesList );
  free( controller->samplersList );
}

size_t GetJointsNumber( Controller genericController )
{
  if( genericController == NULL ) return 0;
  
  ControlData* controller = (ControlData*) genericController;
  
  return controller->jointsNumber;
}

char** GetJointNamesList( Controller genericController )
{
  if( genericController == NULL ) return 0;
  
  ControlData* controller = (ControlData*) genericController;
  
  return (char**) controller->jointNamesList;
}

size_t GetAxesNumber( Controller genericController )
{
  if( genericController == NULL ) return 0;
  
  ControlData* controller = (ControlData*) genericController;
  
  return controller->jointsNumber;
}

char** GetAxisNamesList( Controller genericController )
{
  if( genericController == NULL ) return 0;
  
  ControlData* controller = (ControlData*) genericController;
  
  return (char**) controller->jointNamesList;
}

void SetControlState( Controller genericController, enum ControlState newControlState )
{
  if( genericController == NULL ) return;
  
  ControlData* controller = (ControlData*) genericController;
  
  for( size_t jointIndex = 0; jointIndex < controller->jointsNumber; jointIndex++ )
  {
    EMGJointSampler sampler = controller->samplersList[ jointIndex ];

    if( newControlState == CONTROL_OFFSET )
    {
      DEBUG_PRINT( "starting offset phase for joint %d", sampler->jointID );
      EMGProcessing.SetProcessingPhase( sampler->jointID, SIGNAL_PROCESSING_PHASE_OFFSET );
    }
    else if( newControlState == CONTROL_CALIBRATION )
    {
      DEBUG_PRINT( "starting calibration for joint %d", sampler->jointID );
      EMGProcessing.SetProcessingPhase( sampler->jointID, SIGNAL_PROCESSING_PHASE_CALIBRATION );
    }
    else if( newControlState == CONTROL_OPTIMIZATION )
    {
      DEBUG_PRINT( "reseting sampling count for joint %d", sampler->jointID );
      EMGProcessing.SetProcessingPhase( sampler->jointID, SIGNAL_PROCESSING_PHASE_MEASUREMENT );
      sampler->samplesCount = 0;
    }
    else if( newControlState == CONTROL_OPERATION )
    {
      size_t parametersNumber = sampler->musclesCount * MUSCLE_GAINS_NUMBER + 1;
      if( controller->currentControlState == CONTROL_OPTIMIZATION && parametersNumber > 0 )
      {
        DEBUG_PRINT( "starting optimization for shared joint %d", sampler->jointID );

        double* parametersList = (double*) calloc( parametersNumber, sizeof(double) );
        for( size_t muscleIndex = 0; muscleIndex < sampler->musclesCount; muscleIndex++ )
        {
          for( size_t parameterIndex = 0; parameterIndex < MUSCLE_GAINS_NUMBER; parameterIndex++ )
          {
            //double initialValue = ( MUSCLE_MIN_GAINS[ parameterIndex ] + MUSCLE_MAX_GAINS[ parameterIndex ] ) / 2.0;
            parametersList[ muscleIndex * MUSCLE_GAINS_NUMBER + parameterIndex ] = 0.0;
          }
        }

        //Optimization.Run( parametersList, parametersNumber, CalculateMuscleParametersError, &(sharedJoint->samplingData), OPTIMIZATION_MINIMIZE, 100 );

        free( parametersList );
      }
      
      EMGProcessing.SetProcessingPhase( sampler->jointID, SIGNAL_PROCESSING_PHASE_MEASUREMENT );
    }
  }
  
  controller->currentControlState = newControlState;
}

void RunControlStep( Controller genericController, double** jointMeasuresTable, double** axisMeasuresTable, double** jointSetpointsTable, double** axisSetpointsTable )
{
  if( genericController == NULL ) return;
  
  ControlData* controller = (ControlData*) genericController;
  
  for( size_t jointIndex = 0; jointIndex < controller->jointsNumber; jointIndex++ )
  {
    double jointAngle = jointMeasuresTable[ jointIndex ][ CONTROL_POSITION ] * 360.0;
    double jointIDTorque = jointMeasuresTable[ jointIndex ][ CONTROL_FORCE ];

    axisMeasuresTable[ jointIndex ][ CONTROL_POSITION ] = jointMeasuresTable[ jointIndex ][ CONTROL_POSITION ];
    axisMeasuresTable[ jointIndex ][ CONTROL_VELOCITY ] = jointMeasuresTable[ jointIndex ][ CONTROL_VELOCITY ];
    axisMeasuresTable[ jointIndex ][ CONTROL_ACCELERATION ] = jointMeasuresTable[ jointIndex ][ CONTROL_ACCELERATION ];
    axisMeasuresTable[ jointIndex ][ CONTROL_FORCE ] = jointMeasuresTable[ jointIndex ][ CONTROL_FORCE ];
    axisMeasuresTable[ jointIndex ][ CONTROL_STIFFNESS ] = jointMeasuresTable[ jointIndex ][ CONTROL_STIFFNESS ];
    axisMeasuresTable[ jointIndex ][ CONTROL_DAMPING ] = jointMeasuresTable[ jointIndex ][ CONTROL_DAMPING ];
    
    EMGJointSampler sampler = controller->samplersList[ jointIndex ];
    
    //for( size_t muscleIndex = 0; muscleIndex < sampler->musclesCount; muscleIndex++ )
    //  jointMeasuresTable[ jointIndex ][ muscleIndex ] = EMGProcessing.GetJointMuscleSignal( sampler->jointID, muscleIndex );
    
    if( controller->currentControlState == CONTROL_OPTIMIZATION )
    {
      if( sampler->samplesCount < MAX_SAMPLES_NUMBER )
      {
        double* currentSampleList = sampler->muscleSignalsList[ sampler->samplesCount ];

        sampler->sampleTimesList[ sampler->samplesCount ] = Timing.GetExecTimeSeconds();

        for( size_t muscleIndex = 0; muscleIndex < sampler->musclesCount; muscleIndex++ )
          currentSampleList[ muscleIndex ] = jointMeasuresTable[ jointIndex ][ muscleIndex ];

        sampler->jointAnglesList[ sampler->samplesCount ] = jointAngle;
        sampler->jointIDTorquesList[ sampler->samplesCount ] = jointIDTorque;

        //DEBUG_PRINT( "Saving sample %u: %.3f, %.3f", sampler->samplesCount, jointAngle, jointIDTorque );

        sampler->samplesCount++;
      }
    }
    //else
    //{
      /*jointMeasuresTable[ jointIndex ][ CONTROL_FORCE ] =*/ EMGProcessing.GetJointTorque( sampler->jointID, jointAngle, jointIDTorque );
      //jointMeasuresTable[ jointIndex ][ CONTROL_STIFFNESS ] = EMGProcessing.GetJointStiffness( sampler->jointID, jointAngle );

      //DEBUG_PRINT( "Joint pos: %.3f - force: %.3f - stiff: %.3f", jointAngle, jointMeasuresTable[ jointIndex ][ CONTROL_FORCE ], jointMeasuresTable[ jointIndex ][ CONTROL_STIFFNESS ] );
    //}

    jointSetpointsTable[ jointIndex ][ CONTROL_POSITION ] = axisSetpointsTable[ jointIndex ][ CONTROL_POSITION ];
    jointSetpointsTable[ jointIndex ][ CONTROL_VELOCITY ] = axisSetpointsTable[ jointIndex ][ CONTROL_VELOCITY ];
    jointSetpointsTable[ jointIndex ][ CONTROL_ACCELERATION ] = axisSetpointsTable[ jointIndex ][ CONTROL_ACCELERATION ];
    jointSetpointsTable[ jointIndex ][ CONTROL_FORCE ] = axisSetpointsTable[ jointIndex ][ CONTROL_FORCE ];
    jointSetpointsTable[ jointIndex ][ CONTROL_STIFFNESS ] = axisSetpointsTable[ jointIndex ][ CONTROL_STIFFNESS ];
    jointSetpointsTable[ jointIndex ][ CONTROL_DAMPING ] = axisSetpointsTable[ jointIndex ][ CONTROL_DAMPING ];

    double stiffness = jointSetpointsTable[ jointIndex ][ CONTROL_STIFFNESS ];
    double positionError = jointSetpointsTable[ jointIndex ][ CONTROL_POSITION ] - jointMeasuresTable[ jointIndex ][ CONTROL_POSITION ];
    jointSetpointsTable[ jointIndex ][ CONTROL_FORCE ] = stiffness * positionError;
  }
}



static double CalculateMuscleParametersError( double* parametersList, size_t parametersNumber, void* ref_samplingData )
{
  EMGJointSampler sampler = (EMGJointSampler) ref_samplingData;
  
  double squaredErrorSum = 0;
  size_t musclesNumber = parametersNumber / MUSCLE_GAINS_NUMBER;
  
  parametersList[ parametersNumber - 1 ] = EMGProcessing.SetJointGain( sampler->jointID, parametersList[ parametersNumber - 1 ] );
  
  for( size_t muscleIndex = 0; muscleIndex < musclesNumber; muscleIndex++ )
  {
    for( size_t muscleParameterIndex = 0; muscleParameterIndex < MUSCLE_GAINS_NUMBER; muscleParameterIndex++ )
    {
      double currentValue = parametersList[ muscleIndex * MUSCLE_GAINS_NUMBER + muscleParameterIndex ];
      currentValue = EMGProcessing.SetJointMuscleGain( sampler->jointID, muscleIndex, muscleParameterIndex, currentValue );
      parametersList[ muscleIndex * MUSCLE_GAINS_NUMBER + muscleParameterIndex ] = currentValue;
    }
    
    double* muscleSignalList = sampler->muscleSignalsList[ muscleIndex ];
    
    for( size_t sampleIndex = 0; sampleIndex < sampler->samplesCount; sampleIndex++ )
    {
      double normalizedSample = muscleSignalList[ sampleIndex ];
      double jointAngle = sampler->jointAnglesList[ sampleIndex ];
      double jointIDTorque = sampler->jointIDTorquesList[ sampleIndex ];
    
      double jointEMGTorque = EMGProcessing.GetJointMuscleTorque( sampler->jointID, muscleIndex, normalizedSample, jointAngle );
      
      double sampleError = jointEMGTorque - jointIDTorque;
      
      squaredErrorSum += ( sampleError * sampleError );
    }
  }
  
  return squaredErrorSum / sampler->samplesCount;
}
