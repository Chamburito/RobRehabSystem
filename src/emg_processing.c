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


#include <math.h>
#include <stdbool.h>

#include "sensors.h"
#include "curve_interpolation.h"

#include "config_parser.h"

#include "klib/khash.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

#include "emg_processing.h"

enum { MUSCLE_ACTIVE_FORCE, MUSCLE_PASSIVE_FORCE, MUSCLE_MOMENT_ARM, MUSCLE_NORM_LENGTH, MUSCLE_PENATION_ANGLE, MUSCLE_CURVES_NUMBER };
//enum EMGMuscleGains { MUSCLE_GAIN_ACTIVATION, MUSCLE_GAIN_LENGTH, MUSCLE_GAIN_ARM, MUSCLE_GAIN_PENATION, MUSCLE_GAIN_FORCE, MUSCLE_GAINS_NUMBER };

const double JOINT_MIN_GAIN = 0.1;
const double JOINT_MAX_GAIN = 2.0;

const double MUSCLE_MAX_GAINS[ MUSCLE_GAINS_NUMBER ] = { -0.1, 1.2, 1.2, 1.2, 1.5 };
const double MUSCLE_MIN_GAINS[ MUSCLE_GAINS_NUMBER ] = { -3.0, 0.8, 0.8, 0.8, 0.5 };

typedef struct _EMGMuscleData
{
  Sensor emgSensor;
  double* emgRawBuffer;
  size_t emgRawBufferLength;
  Curve curvesList[ MUSCLE_CURVES_NUMBER ];
  double gainsList[ MUSCLE_GAINS_NUMBER ];
}
EMGMuscleData;

typedef EMGMuscleData* EMGMuscle;

typedef struct _EMGJointData
{
  EMGMuscle* musclesList;
  size_t musclesListLength;
  double scaleFactor;
  int offsetLogID, calibrationLogID, measurementLogID;
  int currentLogID;
  int emgRawLogID;
}
EMGJointData;

typedef EMGJointData* EMGJoint;

KHASH_MAP_INIT_INT( JointInt, EMGJoint )
static khash_t( JointInt )* jointsList = NULL;


DEFINE_NAMESPACE_INTERFACE( EMGProcessing, EMG_PROCESSING_FUNCTIONS )

static EMGJoint LoadEMGJointData( const char* );
static void UnloadEMGJointData( EMGJoint );


static int EMGProcessing_InitJoint( const char* configFileName )
{
  if( jointsList == NULL ) jointsList = kh_init( JointInt );
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newJointIndex = kh_put( JointInt, jointsList, configKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( jointsList, newJointIndex ) = LoadEMGJointData( configFileName );
    if( kh_value( jointsList, newJointIndex ) == NULL )
    {
      DEBUG_PRINT( "EMG joint controller %s configuration failed", configFileName );
      EMGProcessing_EndJoint( (int) newJointIndex );
      return EMG_JOINT_INVALID_ID;
    }
    
    DEBUG_PRINT( "new key %d inserted (iterator: %u - total: %u)", kh_key( jointsList, newJointIndex ), newJointIndex, kh_size( jointsList ) );
  }
  else if( insertionStatus == 0 ) DEBUG_PRINT( "joint key %d already exists", configKey );
  
  return (int) kh_key( jointsList, newJointIndex );
}

void EMGProcessing_EndJoint( int jointID )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return;
  
  UnloadEMGJointData( kh_value( jointsList, jointIndex ) );
  
  kh_del( JointInt, jointsList, jointIndex );
  
  if( kh_size( jointsList ) == 0 )
  {
    kh_destroy( JointInt, jointsList );
    jointsList = NULL;
  }
}

double EMGProcessing_GetJointMuscleSignal( int jointID, size_t muscleIndex )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  if( muscleIndex >= joint->musclesListLength ) return 0.0;
  
  //DEBUG_PRINT( "updating sensor %d-%lu (%p)", jointID, muscleIndex, joint->musclesList[ muscleIndex ]->emgSensor );
  
  double normalizedSignal = Sensors.Update( joint->musclesList[ muscleIndex ]->emgSensor, NULL );
  
  return normalizedSignal;
}

double GetMuscleTorque( EMGMuscle muscle, double normalizedSignal, double jointAngle )
{
  if( muscle == NULL ) return 0.0;
  
  double activationFactor = muscle->gainsList[ MUSCLE_GAIN_ACTIVATION ];
  double activation = ( exp( activationFactor * normalizedSignal ) - 1 ) / ( exp( activationFactor ) - 1 );
  
  double activeForce = CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_ACTIVE_FORCE ], jointAngle, 0.0 );
  double passiveForce = CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_PASSIVE_FORCE ], jointAngle, 0.0 );
  
  double normalizedLength = muscle->gainsList[ MUSCLE_GAIN_LENGTH ] * CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_NORM_LENGTH ], jointAngle, 0.0 );
  double momentArm = muscle->gainsList[ MUSCLE_GAIN_ARM ] * CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_MOMENT_ARM ], jointAngle, 0.0 );
  
  double initialPenationAngle = CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_PENATION_ANGLE ], jointAngle, 0.0 );
  double penationAngle = muscle->gainsList[ MUSCLE_GAIN_PENATION ] * asin( sin( initialPenationAngle ) / normalizedLength );
  
  double normalizedForce = activeForce * activation + passiveForce;
  double resultingForce = muscle->gainsList[ MUSCLE_GAIN_FORCE ] * cos( penationAngle ) * normalizedForce;
  
  return resultingForce * momentArm;
}

double EMGProcessing_GetJointTorque( int jointID, double jointAngle, double jointExternalTorque )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  double jointTorque = 0.0;
  double normalizedSignalsList[ joint->musclesListLength ];
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
  {
    normalizedSignalsList[ muscleIndex ] = Sensors.Update( joint->musclesList[ muscleIndex ]->emgSensor, joint->musclesList[ muscleIndex ]->emgRawBuffer );
    //jointTorque += GetMuscleTorque( joint->musclesList[ muscleIndex ], normalizedSignalsList[ muscleIndex ], jointAngle );
  }
  
  double samplingTime = Timing.GetExecTimeSeconds();
  
  if( joint->currentLogID != DATA_LOG_INVALID_ID )
  {
    if( joint->emgRawLogID != DATA_LOG_INVALID_ID )
    {
      DataLogging.RegisterValues( joint->emgRawLogID, 1, samplingTime );
      for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
        DataLogging.RegisterList( joint->emgRawLogID, joint->musclesList[ muscleIndex ]->emgRawBufferLength, joint->musclesList[ muscleIndex ]->emgRawBuffer );
    }
    
    DataLogging.RegisterValues( joint->currentLogID, 3, samplingTime, jointAngle, jointExternalTorque );
    DataLogging.RegisterList( joint->currentLogID, joint->musclesListLength, normalizedSignalsList );
  }
  
  return jointTorque;
}

double EMGProcessing_GetJointStiffness( int jointID, double jointAngle )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  double jointStiffness = 0.0;
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
  {
    double normalizedSignal = Sensors.Update( joint->musclesList[ muscleIndex ]->emgSensor, NULL );
    jointStiffness += fabs( GetMuscleTorque( joint->musclesList[ muscleIndex ], normalizedSignal, jointAngle ) );
  }
  
  //DEBUG_PRINT( "joint stiffness: %.3f + %.3f = %.3f", EMGProcessing_GetMuscleTorque( 0, jointAngle ), EMGProcessing_GetMuscleTorque( 1, jointAngle ), jointStiffness );
  
  return jointStiffness;
}

double EMGProcessing_SetJointGain( int jointID, double value )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  if( value > JOINT_MAX_GAIN ) value = JOINT_MAX_GAIN;
  else if( value < JOINT_MIN_GAIN ) value = JOINT_MIN_GAIN;
  
  joint->scaleFactor = value;
  
  return joint->scaleFactor;
}

void EMGProcessing_SetProcessingPhase( int jointID, enum SignalProcessingPhase processingPhase )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  DataLogging.SaveData( joint->currentLogID, NULL, 0 );
  
  if( processingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
    joint->currentLogID = joint->offsetLogID;
  else if( processingPhase == SIGNAL_PROCESSING_PHASE_CALIBRATION )
    joint->currentLogID = joint->calibrationLogID;
  else if( processingPhase == SIGNAL_PROCESSING_PHASE_MEASUREMENT )
    joint->currentLogID = joint->measurementLogID;
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
    Sensors.SetState( joint->musclesList[ muscleIndex ]->emgSensor, processingPhase );
}

size_t EMGProcessing_GetJointMusclesCount( int jointID )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  return joint->musclesListLength;
}

double EMGProcessing_SetJointMuscleGain( int jointID, size_t muscleIndex, enum EMGMuscleGain gainIndex, double value )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  if( muscleIndex >= joint->musclesListLength ) return 0.0;
  
  if( gainIndex < 0 && gainIndex >= MUSCLE_GAINS_NUMBER ) return 0.0;
  
  EMGMuscle muscle = joint->musclesList[ muscleIndex ];
  if( value < MUSCLE_MIN_GAINS[ gainIndex ] ) value = MUSCLE_MIN_GAINS[ gainIndex ];
  else if( value > MUSCLE_MAX_GAINS[ gainIndex ] ) value = MUSCLE_MAX_GAINS[ gainIndex ];
  
  muscle->gainsList[ gainIndex ] = value;
  
  return muscle->gainsList[ gainIndex ];
}

double EMGProcessing_GetJointMuscleTorque( int jointID, size_t muscleIndex, double normalizedSignal, double jointAngle )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  if( muscleIndex >= joint->musclesListLength ) return 0.0;
  
  return GetMuscleTorque( joint->musclesList[ muscleIndex ], normalizedSignal, jointAngle );
}


const char* MUSCLE_CURVE_NAMES[ MUSCLE_CURVES_NUMBER ] = { "active_force", "passive_force", "moment_arm", "normalized_length", "penation_angle" };
static EMGMuscle LoadEMGMuscleData( const char* configFileName )
{
  static char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  
  DEBUG_PRINT( "Trying to load muscle %s EMG data", configFileName );
  
  EMGMuscle newMuscle = NULL;
  
  sprintf( filePath, "muscles/%s", configFileName );
  int configFileID = ConfigParsing.LoadConfigFile( filePath );
  if( configFileID != DATA_INVALID_ID )
  {
    newMuscle = (EMGMuscle) malloc( sizeof(EMGMuscleData) );
    
    for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
    {
      char* curveString = ConfigParsing.GetParser()->GetStringValue( configFileID, NULL, "curves.%s", MUSCLE_CURVE_NAMES[ curveIndex ] );
      newMuscle->curvesList[ curveIndex ] = CurveInterpolation.LoadCurveString( curveString );
    }

    newMuscle->gainsList[ MUSCLE_GAIN_ACTIVATION ] = -2.0;
    newMuscle->gainsList[ MUSCLE_GAIN_LENGTH ] = 1.0;
    newMuscle->gainsList[ MUSCLE_GAIN_ARM ] = 1.0;
    newMuscle->gainsList[ MUSCLE_GAIN_PENATION ] = 1.0;
    newMuscle->gainsList[ MUSCLE_GAIN_FORCE ] = 1.0;

    ConfigParsing.GetParser()->UnloadData( configFileID );
  }
  else
  {
    DEBUG_PRINT( "configuration for muscle %s not found", configFileName );
    return NULL;
  }
  
  return newMuscle;
}

static EMGJoint LoadEMGJointData( const char* configFileName )
{
  static char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  
  DEBUG_PRINT( "Trying to load joint %s EMG data", configFileName );
  
  EMGJoint newJoint = NULL;
  
  sprintf( filePath, "joints/%s", configFileName );
  int configFileID = ConfigParsing.LoadConfigFile( filePath );
  if( configFileID != DATA_INVALID_ID )
  {
    newJoint = (EMGJoint) malloc( sizeof(EMGJointData) );
    memset( newJoint, 0, sizeof(EMGJointData) );
    
    bool loadError = false;
    size_t emgRawSamplesNumber = 0;
    if( (newJoint->musclesListLength = (size_t) ConfigParsing.GetParser()->GetListSize( configFileID, "muscles" )) > 0 )
    {
      DEBUG_PRINT( "%u muscles found for joint %s", newJoint->musclesListLength, configFileName );
      
      newJoint->musclesList = (EMGMuscle*) calloc( newJoint->musclesListLength , sizeof(EMGMuscle) );
      for( size_t muscleIndex = 0; muscleIndex < newJoint->musclesListLength; muscleIndex++ )
      {
        char* muscleName = ConfigParsing.GetParser()->GetStringValue( configFileID, "", "muscles.%u.properties", muscleIndex );
        newJoint->musclesList[ muscleIndex ] = LoadEMGMuscleData( muscleName );
        if( newJoint->musclesList[ muscleIndex ] != NULL )
        {
          char* sensorName = ConfigParsing.GetParser()->GetStringValue( configFileID, "", "muscles.%u.sensor", muscleIndex );
          newJoint->musclesList[ muscleIndex ]->emgSensor = Sensors.Init( sensorName, SIGNAL_PROCESSING_RECTIFY | SIGNAL_PROCESSING_NORMALIZE );
          if( newJoint->musclesList[ muscleIndex ]->emgSensor != NULL )
          {
            newJoint->musclesList[ muscleIndex ]->emgRawBufferLength = Sensors.GetInputBufferLength( newJoint->musclesList[ muscleIndex ]->emgSensor );
            newJoint->musclesList[ muscleIndex ]->emgRawBuffer = (double*) calloc( newJoint->musclesList[ muscleIndex ]->emgRawBufferLength, sizeof(double) );
            emgRawSamplesNumber += newJoint->musclesList[ muscleIndex ]->emgRawBufferLength;
          }
          else
            loadError = true;
        }
        else loadError = true;
      }
      
      newJoint->currentLogID = newJoint->offsetLogID = newJoint->calibrationLogID = newJoint->measurementLogID = newJoint->emgRawLogID = DATA_LOG_INVALID_ID;
      if( ConfigParsing.GetParser()->GetBooleanValue( configFileID, false, "log_data" ) )
      {
        size_t jointSampleValuesNumber = newJoint->musclesListLength + 3;
        
        snprintf( filePath, LOG_FILE_PATH_MAX_LEN, "joints/%s_offset", configFileName );                                    
        newJoint->offsetLogID = DataLogging.InitLog( filePath, jointSampleValuesNumber, jointSampleValuesNumber * 1000 );
        DataLogging.SetDataPrecision( newJoint->offsetLogID, DATA_LOG_MAX_PRECISION );
        snprintf( filePath, LOG_FILE_PATH_MAX_LEN, "joints/%s_calibration", configFileName );
        newJoint->calibrationLogID = DataLogging.InitLog( filePath, jointSampleValuesNumber, jointSampleValuesNumber * 1000 );
        DataLogging.SetDataPrecision( newJoint->calibrationLogID, DATA_LOG_MAX_PRECISION );
        snprintf( filePath, LOG_FILE_PATH_MAX_LEN, "joints/%s_sampling", configFileName );
        newJoint->measurementLogID = DataLogging.InitLog( filePath, jointSampleValuesNumber, jointSampleValuesNumber * 1000 );
        DataLogging.SetDataPrecision( newJoint->measurementLogID, DATA_LOG_MAX_PRECISION );
        snprintf( filePath, LOG_FILE_PATH_MAX_LEN, "joints/%s_raw", configFileName );
        newJoint->emgRawLogID = DataLogging.InitLog( filePath, emgRawSamplesNumber + 1, jointSampleValuesNumber * 1000 );
        DataLogging.SetDataPrecision( newJoint->emgRawLogID, 6 );
      }
    }
    else loadError = true;

    ConfigParsing.GetParser()->UnloadData( configFileID );
    
    if( loadError )
    {
      UnloadEMGJointData( newJoint );
      return NULL;
    }
  }
  else
    DEBUG_PRINT( "configuration for joint %s not found", configFileName );
  
  return newJoint;
}

static void UnloadEMGJointData( EMGJoint joint )
{
  if( joint == NULL ) return;
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
  {
    Sensors.End( joint->musclesList[ muscleIndex ]->emgSensor );
    for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
      CurveInterpolation.UnloadCurve( joint->musclesList[ muscleIndex ]->curvesList[ curveIndex ] );
    free( joint->musclesList[ muscleIndex ]->emgRawBuffer );
  }
  
  DataLogging.EndLog( joint->offsetLogID );
  DataLogging.EndLog( joint->calibrationLogID );
  DataLogging.EndLog( joint->measurementLogID );
  DataLogging.EndLog( joint->emgRawLogID );
  
  free( joint->musclesList );
  
  free( joint );
}
