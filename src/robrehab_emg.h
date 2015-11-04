#ifndef ROBREHAB_EMG_H
#define ROBREHAB_EMG_H   

#include "shm_control.h"
#include "shm_emg_control.h"

#include "optimization.h"
#include "emg_processing.h"

#include "config_parser.h"

#include "klib/kvec.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

#include <stdlib.h>

const unsigned long UPDATE_INTERVAL_MS = 50; // 5;

#define MAX_SAMPLES_NUMBER 100
typedef double SamplingBuffer[ MAX_SAMPLES_NUMBER ];

typedef struct _SamplingData
{
  SamplingBuffer* muscleSignalsList;
  SamplingBuffer jointAnglesList;
  SamplingBuffer jointIDTorquesList;
  size_t musclesCount, samplesCount;
}
SamplingData;

typedef struct _SHMJointData
{
  SHMController controller;
  int jointID;
  int lastJointPhase;
  SamplingData samplingData;
}
SHMJointData;

typedef SHMJointData* SHMJoint;

kvec_t( SHMJointData ) sharedJointsList;
      
static double CalculateMuscleParametersError( double*, size_t, void* );

#define SUBSYSTEM RobRehabEMG

#define ROBREHAB_EMG_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, Init, void ) \
        function_init( void, namespace, End, void ) \
        function_init( void, namespace, Update, void )

INIT_NAMESPACE_INTERFACE( SUBSYSTEM, ROBREHAB_EMG_FUNCTIONS )


int RobRehabEMG_Init( void )
{
  kv_init( sharedJointsList );
  
  if( ConfigParsing.Init( "JSON" ) )
  {
    ParserInterface parser = ConfigParsing.GetParser();
    int configFileID = parser.LoadFileData( "shared_robots" );
    if( configFileID != -1 )
    {
      size_t sharedRobotsNumber = parser.GetListSize( configFileID, "robots" );
      
      DEBUG_PRINT( "List size: %u", sharedRobotsNumber );
      
      char robotVarName[ PARSER_MAX_KEY_PATH_LENGTH ];
      for( size_t sharedRobotIndex = 0; sharedRobotIndex < sharedRobotsNumber; sharedRobotIndex++ )
      {
        char* robotName = parser.GetStringValue( configFileID, "", "robots.%lu.name", sharedRobotIndex );
        DEBUG_PRINT( "found robot %s", robotName );
        
        size_t sharedJointsNumber = parser.GetListSize( configFileID, "robots.%lu.joints", sharedRobotIndex );
        for( size_t sharedJointIndex = 0; sharedJointIndex < sharedJointsNumber; sharedJointIndex++ )
        {
          SHMJointData newSharedJoint = { .lastJointPhase = SHM_EMG_CALIBRATION };
          char* jointName = parser.GetStringValue( configFileID, "", "robots.%lu.joints.%lu", sharedRobotIndex, sharedJointIndex );
          sprintf( robotVarName, "%s-%s", robotName, jointName );
          if( (newSharedJoint.jointID = EMGProcessing.InitJoint( jointName )) != EMG_JOINT_INVALID_ID )
          {
            if( (newSharedJoint.controller = SHMControl.InitData( robotVarName, SHM_CONTROL_OUT )) != NULL )
            {           
              (void) EMGProcessing.GetJointMusclesList( newSharedJoint.jointID, &(newSharedJoint.samplingData.musclesCount) );
              newSharedJoint.samplingData.muscleSignalsList = (SamplingBuffer*) calloc( newSharedJoint.samplingData.musclesCount, sizeof(SamplingBuffer) );
              
              kv_push( SHMJointData, sharedJointsList, newSharedJoint );
              
              DEBUG_PRINT( "Got new shared joint with ID %d, %u muscles and axis %p (total: %u)", newSharedJoint.jointID, newSharedJoint.samplingData.musclesCount, newSharedJoint.controller, kv_size( sharedJointsList ) );
            }
            else EMGProcessing.EndJoint( newSharedJoint.jointID );
          }
        }
      }
    
      parser.UnloadData( configFileID );
    }
  }
  
  return 0;
}

void RobRehabEMG_End( void )
{
  DEBUG_PRINT( "cleaning shared joints vector %p", sharedJointsList.a );
  
  for( size_t sharedJointID = 0; sharedJointID < kv_size( sharedJointsList ); sharedJointID++ )
  {
    DEBUG_PRINT( "ending joint control %u", sharedJointID ); 
      
    SHMJoint sharedJoint = &(kv_A( sharedJointsList, sharedJointID ));
      
    SHMControl.EndData( sharedJoint->controller );
    EMGProcessing.EndJoint( sharedJoint->jointID );
    
    free( sharedJoint->samplingData.muscleSignalsList );
  }
  
  kv_destroy( sharedJointsList );
}

void RobRehabEMG_Update( void )
{
  for( size_t sharedJointID = 0; sharedJointID < kv_size( sharedJointsList ); sharedJointID++ )
  {
    SHMJoint sharedJoint = &(kv_A( sharedJointsList, sharedJointID ));
    
    uint8_t jointPhase = SHMControl.GetByteValue( sharedJoint->controller, SHM_CONTROL_REMOVE );
    if( jointPhase == SHM_EMG_CALIBRATION )
    {
      DEBUG_PRINT( "starting calibration for joint %d", sharedJoint->jointID );
      EMGProcessing.SetJointCalibration( sharedJoint->jointID, true );
      sharedJoint->lastJointPhase = SHM_EMG_CALIBRATION;
    }
    else if( jointPhase == SHM_EMG_SAMPLING )
    {
      DEBUG_PRINT( "reseting sampling count for joint %d", sharedJoint->jointID );
      EMGProcessing.SetJointCalibration( sharedJoint->jointID, false );
      sharedJoint->samplingData.samplesCount = 0;
      sharedJoint->lastJointPhase = SHM_EMG_SAMPLING;
    }
    else if( jointPhase == SHM_EMG_MEASUREMENT )
    {
      size_t parametersNumber = sharedJoint->samplingData.musclesCount * ( sizeof(EMGMuscleData) / sizeof(double) );
      if( sharedJoint->lastJointPhase == SHM_EMG_SAMPLING && parametersNumber > 0 )
      {
        DEBUG_PRINT( "starting optimization for shared joint %d", sharedJoint->jointID );
        double* parametersList = (double*) EMGProcessing.GetJointMusclesList( sharedJoint->jointID, NULL );
        
        // hack
        for( size_t parameterIndex = 0; parameterIndex < parametersNumber; parameterIndex++ )
          parametersList[ parameterIndex ] = ( (double) ( rand() % 10 ) ) / 100.0;
        
        Optimization.Run( parametersList, parametersNumber, CalculateMuscleParametersError, &(sharedJoint->samplingData), OPTIMIZATION_MINIMIZE, 100 );
      }
      sharedJoint->lastJointPhase = SHM_EMG_MEASUREMENT;
    }

    float seed = ( (float) ( rand() % 2001 - 1000 ) ) / 1000.0;
    
    float jointAngle, jointIDTorque;
    if( SHMControl.GetNumericValue( sharedJoint->controller, SHM_JOINT_ANGLE, &jointAngle, SHM_CONTROL_PEEK ) )
    {
      if( SHMControl.GetNumericValue( sharedJoint->controller, SHM_JOINT_ID_TORQUE, &jointIDTorque, SHM_CONTROL_PEEK ) )
      {
        if( sharedJoint->lastJointPhase == SHM_EMG_SAMPLING )
        {      
          if( sharedJoint->samplingData.samplesCount < MAX_SAMPLES_NUMBER )
          {
            for( size_t muscleIndex = 0; muscleIndex < sharedJoint->samplingData.musclesCount; muscleIndex++ )
            {
              double normalizedSignal = EMGProcessing.GetJointMuscleSignal( sharedJoint->jointID, muscleIndex );
              sharedJoint->samplingData.muscleSignalsList[ muscleIndex ][ sharedJoint->samplingData.samplesCount ] = normalizedSignal;
            }
            
            DEBUG_PRINT( "sample %u: %g - %g %g %g %g %g", sharedJoint->samplingData.samplesCount, jointIDTorque, 
                         sharedJoint->samplingData.muscleSignalsList[ 0 ][ sharedJoint->samplingData.samplesCount ],
                         sharedJoint->samplingData.muscleSignalsList[ 1 ][ sharedJoint->samplingData.samplesCount ],
                         sharedJoint->samplingData.muscleSignalsList[ 2 ][ sharedJoint->samplingData.samplesCount ],
                         sharedJoint->samplingData.muscleSignalsList[ 3 ][ sharedJoint->samplingData.samplesCount ],
                         sharedJoint->samplingData.muscleSignalsList[ 4 ][ sharedJoint->samplingData.samplesCount ] );
            
            sharedJoint->samplingData.jointAnglesList[ sharedJoint->samplingData.samplesCount ] = jointAngle;
            sharedJoint->samplingData.jointIDTorquesList[ sharedJoint->samplingData.samplesCount ] = jointIDTorque;
            
            sharedJoint->samplingData.samplesCount++;
          }
        }  
        else
        {      
          double jointEMGTorque = EMGProcessing_GetJointTorque( sharedJoint->jointID, jointAngle );
          double jointEMGStiffness = EMGProcessing_GetJointStiffness( sharedJoint->jointID, jointAngle );
          
          //DEBUG_PRINT( "Joint position: %.3f - torque: %.3f - stiffness: %.3f", jointAngle, jointEMGTorque, jointEMGStiffness );
          
          SHMControl.SetNumericValue( sharedJoint->controller, SHM_JOINT_EMG_TORQUE, jointEMGTorque );
          SHMControl.SetNumericValue( sharedJoint->controller, SHM_JOINT_EMG_STIFFNESS, jointEMGStiffness );
        }
      }
    }
  }
}


double CalculateMuscleParametersError( double* parametersList, size_t parametersNumber, void* ref_samplingData )
{
  SamplingData* samplingData = (SamplingData*) ref_samplingData;
  
  double squaredErrorSum = 0;
  size_t musclesNumber = parametersNumber / ( sizeof(EMGMuscleData) / sizeof(double) );
  for( size_t muscleIndex = 0; muscleIndex < musclesNumber; muscleIndex++ )
  {
    EMGMuscle muscleParameters = ( (EMGMuscle) parametersList ) + muscleIndex;
    double* muscleSignalList = samplingData->muscleSignalsList[ muscleIndex ];
    for( size_t sampleIndex = 0; sampleIndex < samplingData->samplesCount; sampleIndex++ )
    {
      double normalizedSample = muscleSignalList[ sampleIndex ];
      double jointAngle = samplingData->jointAnglesList[ sampleIndex ];
      double jointIDTorque = samplingData->jointIDTorquesList[ sampleIndex ];
    
      double jointEMGTorque = EMGProcessing.GetMuscleTorque( muscleParameters, normalizedSample, jointAngle );
      
      double sampleError = jointEMGTorque - jointIDTorque;
      
      squaredErrorSum += ( sampleError * sampleError );
    }
  }
  
  return squaredErrorSum / samplingData->samplesCount;
}

#endif // ROBREHAB_EMG_H   
