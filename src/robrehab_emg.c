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
#include "shm_emg_control.h"

//#include "optimization.h"
#include "emg_processing.h"

#include "config_parser.h"

#include "klib/kvec.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

#include <stdlib.h>

#include "robrehab_subsystem.h"

#define MAX_SAMPLES_NUMBER 5000

const unsigned long UPDATE_INTERVAL_MS = 5;


typedef struct _SamplingData
{
  int jointID;
  double** muscleSignalsList;
  double* jointAnglesList;//double jointAnglesList[ MAX_SAMPLES_NUMBER ];
  double* jointIDTorquesList;//double jointIDTorquesList[ MAX_SAMPLES_NUMBER ];
  double* sampleTimesList;//double sampleTimesList[ MAX_SAMPLES_NUMBER ];
  size_t musclesCount, samplesCount;
}
SamplingData;

typedef struct _SHMJointData
{
  int lastJointPhase;
  SamplingData samplingData;
  int samplingLogID;
}
SHMJointData;

typedef SHMJointData* SHMJoint;

kvec_t( SHMJointData ) sharedJointsList;

SHMController sharedRobotJointsInfo;
SHMController sharedRobotJointsData;

char robotJointsInfo[ SHM_CONTROL_MAX_DATA_SIZE ] = "";
      

DEFINE_NAMESPACE_INTERFACE( SubSystem, ROBREHAB_SUBSYSTEM_INTERFACE )

void LoadSharedJointsInfo( void );
double CalculateMuscleParametersError( double*, size_t, void* );


int SubSystem_Init( const char* configType )
{
  kv_init( sharedJointsList );
  
  sharedRobotJointsInfo = SHMControl.InitData( "robot_joints_info", SHM_CONTROL_OUT );
  sharedRobotJointsData = SHMControl.InitData( "robot_joints_data", SHM_CONTROL_OUT );
  
  if( ConfigParsing.Init( configType ) )
  {
    SHMControl.GetData( sharedRobotJointsInfo, (void*) robotJointsInfo, 0, SHM_CONTROL_MAX_DATA_SIZE );
    
    strtok( robotJointsInfo, "|" );
    char* jointsInfo = strtok( NULL, "|" );
    if( jointsInfo != NULL )
    {
      for( char* jointIDString = strtok( jointsInfo, ":" ); jointIDString != NULL; jointIDString = strtok( NULL, ":" ) )
      {
        uint8_t jointIndex = (uint8_t) strtoul( jointIDString, NULL, 0 );
        char* jointName = strtok( NULL, ":" );
        
        SHMJointData newSharedJoint = { .lastJointPhase = SHM_EMG_CALIBRATION };
        if( (newSharedJoint.samplingData.jointID = EMGProcessing.InitJoint( jointName )) != EMG_JOINT_INVALID_ID )
        {
          newSharedJoint.samplingData.musclesCount = EMGProcessing.GetJointMusclesCount( newSharedJoint.samplingData.jointID );
          newSharedJoint.samplingData.muscleSignalsList = (double**) calloc( MAX_SAMPLES_NUMBER, sizeof(double*) );
          for( size_t sampleIndex = 0; sampleIndex < MAX_SAMPLES_NUMBER; sampleIndex++ )
            newSharedJoint.samplingData.muscleSignalsList[ sampleIndex ] = (double*) calloc( newSharedJoint.samplingData.musclesCount, sizeof(double) );
          newSharedJoint.samplingData.jointAnglesList = (double*) calloc( MAX_SAMPLES_NUMBER, sizeof(double) );
          newSharedJoint.samplingData.jointIDTorquesList = (double*) calloc( MAX_SAMPLES_NUMBER, sizeof(double) );
          newSharedJoint.samplingData.sampleTimesList = (double*) calloc( MAX_SAMPLES_NUMBER, sizeof(double) );

          size_t sampleValuesNumber = newSharedJoint.samplingData.musclesCount + 3;
          newSharedJoint.samplingLogID = DataLogging.InitLog( jointName, sampleValuesNumber, sampleValuesNumber * MAX_SAMPLES_NUMBER / 5.0 );
          DataLogging.SetDataPrecision( newSharedJoint.samplingLogID, DATA_LOG_MAX_PRECISION );

          kv_push( SHMJointData, sharedJointsList, newSharedJoint );

          DEBUG_PRINT( "Got new shared joint with ID %d and %u muscles (total: %u)", newSharedJoint.samplingData.jointID, newSharedJoint.samplingData.musclesCount, kv_size( sharedJointsList ) );
        }
      }
    }
  }
  
  return 0;
}

void SubSystem_End( void )
{
  DEBUG_PRINT( "cleaning shared joints vector %p", sharedJointsList.a );
  
  for( size_t jointIndex = 0; jointIndex < kv_size( sharedJointsList ); jointIndex++ )
  {
    DEBUG_PRINT( "ending joint control %u", jointIndex ); 
      
    SHMJoint sharedJoint = &(kv_A( sharedJointsList, jointIndex ));
      
    EMGProcessing.EndJoint( sharedJoint->samplingData.jointID );
    
    DataLogging.EndLog( sharedJoint->samplingLogID );

    for( size_t sampleIndex = 0; sampleIndex < MAX_SAMPLES_NUMBER; sampleIndex++ )
      free( sharedJoint->samplingData.muscleSignalsList[ sampleIndex ] );
    free( sharedJoint->samplingData.muscleSignalsList );
  }
  
  SHMControl.EndData( sharedRobotJointsInfo );
  SHMControl.EndData( sharedRobotJointsData );
  
  kv_destroy( sharedJointsList );
}

void SubSystem_Update( void )
{
  static uint8_t controlData[ SHM_CONTROL_MAX_DATA_SIZE ];
  
  SHMControl.GetData( sharedRobotJointsInfo, controlData, 0, SHM_CONTROL_MAX_DATA_SIZE );
  
  for( size_t jointIndex = 0; jointIndex < kv_size( sharedJointsList ); jointIndex++ )
  {
    SHMJoint sharedJoint = &(kv_A( sharedJointsList, jointIndex ));
    
    uint8_t jointPhase = SHMControl.GetMaskByte( sharedRobotJointsInfo, 3, SHM_CONTROL_REMOVE );
    
    jointPhase = SHM_EMG_SAMPLING;
    
    if( jointPhase == SHM_EMG_OFFSET && sharedJoint->lastJointPhase != SHM_EMG_OFFSET )
    {
      DEBUG_PRINT( "starting offset phase for joint %d", sharedJoint->samplingData.jointID );
      EMGProcessing.SetProcessingPhase( sharedJoint->samplingData.jointID, SIGNAL_PROCESSING_PHASE_OFFSET );
      sharedJoint->lastJointPhase = SHM_EMG_OFFSET;
    }
    else if( jointPhase == SHM_EMG_CALIBRATION && sharedJoint->lastJointPhase != SHM_EMG_CALIBRATION )
    {
      DEBUG_PRINT( "starting calibration for joint %d", sharedJoint->samplingData.jointID );
      EMGProcessing.SetProcessingPhase( sharedJoint->samplingData.jointID, SIGNAL_PROCESSING_PHASE_CALIBRATION );
      sharedJoint->lastJointPhase = SHM_EMG_CALIBRATION;
    }
    else if( jointPhase == SHM_EMG_SAMPLING && sharedJoint->lastJointPhase != SHM_EMG_SAMPLING )
    {
      DEBUG_PRINT( "reseting sampling count for joint %d", sharedJoint->samplingData.jointID );
      EMGProcessing.SetProcessingPhase( sharedJoint->samplingData.jointID, SIGNAL_PROCESSING_PHASE_MEASUREMENT );
      sharedJoint->samplingData.samplesCount = 0;
      sharedJoint->lastJointPhase = SHM_EMG_SAMPLING;
    }
    else if( jointPhase == SHM_EMG_MEASUREMENT && sharedJoint->lastJointPhase != SHM_EMG_MEASUREMENT )
    {
      size_t parametersNumber = sharedJoint->samplingData.musclesCount * MUSCLE_GAINS_NUMBER + 1;
      if( sharedJoint->lastJointPhase == SHM_EMG_SAMPLING && parametersNumber > 0 )
      {
        DEBUG_PRINT( "starting optimization for shared joint %d", sharedJoint->samplingData.jointID );
        
        double* parametersList = (double*) calloc( parametersNumber, sizeof(double) );
        for( size_t muscleIndex = 0; muscleIndex < sharedJoint->samplingData.musclesCount; muscleIndex++ )
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
      sharedJoint->lastJointPhase = SHM_EMG_MEASUREMENT;
    }
    
    //DEBUG_PRINT( "updating joint %lu", jointIndex );
    
    float* jointControlMeasures = (float*) controlData;
    double jointAngle = (double) jointControlMeasures[ SHM_JOINT_POSITION ] * 360.0;
    double jointIDTorque = (double) jointControlMeasures[ SHM_JOINT_FORCE ];

    if( sharedJoint->lastJointPhase == SHM_EMG_SAMPLING )
    {
      if( sharedJoint->samplingData.samplesCount < MAX_SAMPLES_NUMBER )
      {
        double* currentSampleList = sharedJoint->samplingData.muscleSignalsList[ sharedJoint->samplingData.samplesCount ];

        sharedJoint->samplingData.sampleTimesList[ sharedJoint->samplingData.samplesCount ] = Timing.GetExecTimeSeconds();

        for( size_t muscleIndex = 0; muscleIndex < sharedJoint->samplingData.musclesCount; muscleIndex++ )
          currentSampleList[ muscleIndex ] = EMGProcessing.GetJointMuscleSignal( sharedJoint->samplingData.jointID, muscleIndex );

        sharedJoint->samplingData.jointAnglesList[ sharedJoint->samplingData.samplesCount ] = jointAngle;
        sharedJoint->samplingData.jointIDTorquesList[ sharedJoint->samplingData.samplesCount ] = jointIDTorque;

        double relativeTime = sharedJoint->samplingData.sampleTimesList[ sharedJoint->samplingData.samplesCount ] - sharedJoint->samplingData.sampleTimesList[ 0 ];

        DEBUG_PRINT( "Saving sample %u (%.3f): %.3f, %.3f", sharedJoint->samplingData.samplesCount, relativeTime, jointAngle, jointIDTorque );

        DataLogging.RegisterValues( sharedJoint->samplingLogID, 3, relativeTime, jointAngle, jointIDTorque );
        DataLogging.RegisterList( sharedJoint->samplingLogID, sharedJoint->samplingData.musclesCount, currentSampleList );

        sharedJoint->samplingData.samplesCount++;
      }
    }
    else
    {
      //double jointEMGTorque = EMGProcessing.GetJointTorque( sharedJoint->samplingData.jointID, jointAngle );
      //double jointEMGStiffness = EMGProcessing.GetJointStiffness( sharedJoint->samplingData.jointID, jointAngle );

      //DEBUG_PRINT( "Joint position: %.3f", jointAngle );

      //SHMControl.SetNumericValue( sharedJoint->controller, SHM_JOINT_EMG_TORQUE, jointEMGTorque );
      //SHMControl.SetNumericValue( sharedJoint->controller, SHM_JOINT_EMG_STIFFNESS, jointEMGStiffness );
    }
  }
}


double CalculateMuscleParametersError( double* parametersList, size_t parametersNumber, void* ref_samplingData )
{
  SamplingData* samplingData = (SamplingData*) ref_samplingData;
  
  double squaredErrorSum = 0;
  size_t musclesNumber = parametersNumber / MUSCLE_GAINS_NUMBER;
  
  parametersList[ parametersNumber - 1 ] = EMGProcessing.SetJointGain( samplingData->jointID, parametersList[ parametersNumber - 1 ] );
  
  for( size_t muscleIndex = 0; muscleIndex < musclesNumber; muscleIndex++ )
  {
    for( size_t muscleParameterIndex = 0; muscleParameterIndex < MUSCLE_GAINS_NUMBER; muscleParameterIndex++ )
    {
      double currentValue = parametersList[ muscleIndex * MUSCLE_GAINS_NUMBER + muscleParameterIndex ];
      currentValue = EMGProcessing.SetJointMuscleGain( samplingData->jointID, muscleIndex, muscleParameterIndex, currentValue );
      parametersList[ muscleIndex * MUSCLE_GAINS_NUMBER + muscleParameterIndex ] = currentValue;
    }
    
    double* muscleSignalList = samplingData->muscleSignalsList[ muscleIndex ];
    
    for( size_t sampleIndex = 0; sampleIndex < samplingData->samplesCount; sampleIndex++ )
    {
      double normalizedSample = muscleSignalList[ sampleIndex ];
      double jointAngle = samplingData->jointAnglesList[ sampleIndex ];
      double jointIDTorque = samplingData->jointIDTorquesList[ sampleIndex ];
    
      double jointEMGTorque = EMGProcessing.GetJointMuscleTorque( samplingData->jointID, muscleIndex, normalizedSample, jointAngle );
      
      double sampleError = jointEMGTorque - jointIDTorque;
      
      squaredErrorSum += ( sampleError * sampleError );
    }
  }
  
  return squaredErrorSum / samplingData->samplesCount;
}
