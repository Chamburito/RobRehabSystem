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


const char* CONFIG_KEY = "keys";
int RobRehabEMG_Init( void )
{
  kv_init( sharedJointsList );
  
  char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  if( ConfigParser.Init( "JSON" ) )
  {
    sprintf( searchPath, "shared_%s", CONFIG_KEY );
    int configFileID = ConfigParser.LoadFile( searchPath );
    if( configFileID != -1 )
    {
      if( ConfigParser.HasKey( configFileID, CONFIG_KEY ) )
      {
        size_t sharedJointsNumber = ConfigParser.GetListSize( configFileID, CONFIG_KEY );
      
        DEBUG_PRINT( "List size: %u", sharedJointsNumber );
      
        for( size_t sharedJointIndex = 0; sharedJointIndex < sharedJointsNumber; sharedJointIndex++ )
        {
          sprintf( searchPath, "%s.%u", CONFIG_KEY, sharedJointIndex );
          char* jointName = ConfigParser.GetStringValue( configFileID, searchPath );

          DEBUG_PRINT( "found joint %s", jointName );

          bool loadError = false;

          int newSharedJointID = EMG_JOINT_INVALID_ID;
          SHMController newSharedAxis = NULL;
          if( (newSharedJointID = EMGProcessing.InitJoint( jointName )) != EMG_JOINT_INVALID_ID )
          {
            if( (newSharedAxis = SHMControl.InitData( jointName, SHM_CONTROL_OUT )) != NULL )
            {
              SHMJointData newSharedJoint = { .controller = newSharedAxis, .jointID = newSharedJointID, .lastJointPhase = SHM_EMG_CALIBRATION };
              
              (void) EMGProcessing.GetJointMusclesList( newSharedJointID, &(newSharedJoint.samplingData.musclesCount) );
              newSharedJoint.samplingData.muscleSignalsList = (SamplingBuffer*) calloc( newSharedJoint.samplingData.musclesCount, sizeof(SamplingBuffer) );
            
              kv_push( SHMJointData, sharedJointsList, newSharedJoint );

              DEBUG_PRINT( "Got new shared joint with ID %d, %u muscles and axis %p (total: %u)", newSharedJointID, newSharedJoint.samplingData.musclesCount, newSharedAxis, kv_size( sharedJointsList ) );
            }
            else loadError = true;
          }
          else loadError = true;

          if( loadError )
          {
            SHMControl.EndData( newSharedAxis );
            EMGProcessing_EndJoint( newSharedJointID );
            return -1;
          }
        }
      }
    
      ConfigParser.UnloadFile( configFileID );
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
    EMGProcessing_EndJoint( sharedJoint->jointID );
    
    free( sharedJoint->samplingData.muscleSignalsList );
  }
  
  kv_destroy( sharedJointsList );
}

void RobRehabEMG_Update( void )
{
  for( size_t sharedJointID = 0; sharedJointID < kv_size( sharedJointsList ); sharedJointID++ )
  {
    SHMJoint sharedJoint = &(kv_A( sharedJointsList, sharedJointID ));
    
    // hack
    uint8_t jointPhase = SHMControl.GetByteValue( sharedJoint->controller, SHM_CONTROL_PEEK );
    if( jointPhase == SHM_CONTROL_BYTE_NULL ) jointPhase = (uint8_t) sharedJoint->lastJointPhase;
    
    //DEBUG_PRINT( "updating joint %d", sharedJoint->jointID );
    
    // hack
    if( sharedJoint->lastJointPhase == SHM_EMG_CALIBRATION ) 
    {
      DEBUG_PRINT( "starting sampling for joint %d", sharedJoint->jointID );
      jointPhase = SHM_EMG_SAMPLING;
    }
    else if( sharedJoint->samplingData.samplesCount >= MAX_SAMPLES_NUMBER ) 
    {
      DEBUG_PRINT( "starting optimization for joint %d (%u samples)", sharedJoint->jointID, sharedJoint->samplingData.samplesCount );
      jointPhase = SHM_EMG_MEASUREMENT;
    }

    if( jointPhase == SHM_EMG_CALIBRATION && sharedJoint->lastJointPhase != SHM_EMG_CALIBRATION )
    {
      EMGProcessing.SetJointCalibration( sharedJoint->jointID, true );
      sharedJoint->lastJointPhase = SHM_EMG_CALIBRATION;
    }
    else 
    {
      float seed = ( (float) ( rand() % 2001 - 1000 ) ) / 1000.0;
      
      float jointAngle;
      SHMControl.GetNumericValue( sharedJoint->controller, SHM_AXIS_POSITION, &jointAngle, SHM_CONTROL_PEEK );
      //if( SHMControl.GetNumericValue( sharedJoint->controller, SHM_AXIS_POSITION, &jointAngle, SHM_CONTROL_PEEK ) )
      //{
        if( jointPhase == SHM_EMG_SAMPLING )
        {
          if( sharedJoint->lastJointPhase != SHM_EMG_SAMPLING )
          {
            DEBUG_PRINT( "reseting sampling count for joint %d", sharedJoint->jointID );
            EMGProcessing.SetJointCalibration( sharedJoint->jointID, false );
            sharedJoint->samplingData.samplesCount = 0;
            sharedJoint->lastJointPhase = SHM_EMG_SAMPLING;
          }
      
          float jointIDTorque;
          jointIDTorque = seed * 50.0;
          //if( SHMControl.GetNumericValue( sharedJoint->controller, SHM_AXIS_FORCE, &jointIDTorque, SHM_CONTROL_PEEK ) )
          //{
            if( sharedJoint->samplingData.samplesCount < MAX_SAMPLES_NUMBER )
            {
              for( size_t muscleIndex = 0; muscleIndex < sharedJoint->samplingData.musclesCount; muscleIndex++ )
              {
                double normalizedSignal = ( seed * ( ( (int) muscleIndex ) - 2 ) > 0.0 ) ? fabs( seed ) : 0.0;//EMGProcessing.GetJointMuscleSignal( sharedJoint->jointID, muscleIndex );
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
          //}
        }  
        else
        {
          if( jointPhase == SHM_EMG_MEASUREMENT && sharedJoint->lastJointPhase == SHM_EMG_SAMPLING )
          {
            DEBUG_PRINT( "starting optimization for shared joint %p", sharedJoint );
            double* parametersList = (double*) EMGProcessing.GetJointMusclesList( sharedJoint->jointID, NULL );
            double parametersNumber = sharedJoint->samplingData.musclesCount * ( sizeof(EMGMuscleData) / sizeof(double) );
            
            // hack
            for( size_t parameterIndex = 0; parameterIndex < parametersNumber; parameterIndex++ )
              parametersList[ parameterIndex ] = ( (double) ( rand() % 10 ) ) / 100.0;
            
            Optimization.Run( parametersList, parametersNumber, CalculateMuscleParametersError, &(sharedJoint->samplingData), OPTIMIZATION_MINIMIZE, 100 );
            sharedJoint->lastJointPhase = SHM_EMG_MEASUREMENT;
          }
          
          double jointEMGTorque = EMGProcessing_GetJointTorque( sharedJoint->jointID, jointAngle );
          double jointEMGStiffness = EMGProcessing_GetJointStiffness( sharedJoint->jointID, jointAngle );

          //DEBUG_PRINT( "Joint position: %.3f - torque: %.3f - stiffness: %.3f", jointAngle, jointEMGTorque, jointEMGStiffness );

          SHMControl.SetNumericValue( sharedJoint->controller, SHM_EMG_TORQUE, jointEMGTorque );
          SHMControl.SetNumericValue( sharedJoint->controller, SHM_EMG_STIFFNESS, jointEMGStiffness );
        }
      //}
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
