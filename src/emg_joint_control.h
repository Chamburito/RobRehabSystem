#ifndef EMG_JOINT_CONTROL_H
#define EMG_JOINT_CONTROL_H

#include "emg_processing.h"

#include "klib/khash.h"
#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

enum MuscleGroups { MUSCLE_AGONIST, MUSCLE_ANTAGONIST, MUSCLE_GROUPS_NUMBER };

enum { JOINT_TORQUE, JOINT_STIFFNESS, JOINT_MEASURES_NUMBER };

typedef struct _EMGJoint
{
  int* muscleIDsTable[ MUSCLE_GROUPS_NUMBER ];
  size_t muscleGroupsSizesList[ MUSCLE_GROUPS_NUMBER ];
}
EMGJoint;

KHASH_MAP_INIT_STR( EMGJoint, EMGJoint )
static khash_t( EMGJoint )* emgJointsList = NULL;

static int InitJoint( const char* );
static void EndJoint( int );
static double GetTorque( int, double );
static double GetStiffness( int, double );
static void ChangeState( int, enum MuscleGroups, enum EMGProcessPhase );

const struct
{
  int (*InitJoint)( const char* );
  void (*EndJoint)( int );
  double (*GetTorque)( int, double );
  double (*GetStiffness)( int, double );
  void (*ChangeState)( int, enum MuscleGroups, enum EMGProcessPhase );
}
EMGJointControl = { .InitJoint = InitJoint, .EndJoint = EndJoint, .GetTorque = GetTorque, .GetStiffness = GetStiffness, .ChangeState = ChangeState };

static EMGJoint* LoadEMGJointData( const char* );
static void UnloadEMGJointData( EMGJoint* );

static int InitJoint( const char* configFileName )
{
  if( emgJointsList != NULL )
  {
    khint_t jointID = kh_get( EMGJoint, emgJointsList, configFileName );
    if( jointID != kh_end( emgJointsList ) )
    {
      DEBUG_PRINT( "joint key %s already exists", configFileName );
      return (int) jointID;
    }
  }
  
  EMGJoint* ref_newJointData = LoadEMGJointData( configFileName );
  if( ref_newJointData == NULL ) return -1;
  
  if( emgJointsList == NULL ) emgJointsList = kh_init( EMGJoint );
  
  int insertionStatus;
  khint_t newJointID = kh_put( EMGJoint, emgJointsList, configFileName, &insertionStatus );

  EMGJoint* newJoint = &(kh_value( emgJointsList, newJointID ));
  memcpy( newJoint, ref_newJointData, sizeof(EMGJoint) );
  
  return (int) newJointID;
}

void EndJoint( int jointID )
{
  if( !kh_exist( emgJointsList, (khint_t) jointID ) ) return;
  
  UnloadEMGJointData( &(kh_value( emgJointsList, (khint_t) jointID )) );
  
  kh_del( EMGJoint, emgJointsList, (khint_t) jointID );
  
  if( kh_size( emgJointsList ) == 0 )
  {
    kh_destroy( EMGJoint, emgJointsList );
    emgJointsList = NULL;
  }
}

double GetTorque( int jointID, double jointAngle )
{
  if( !kh_exist( emgJointsList, (khint_t) jointID ) ) return 0.0;
  
  double jointTorque = 0.0;
  
  EMGJoint* joint = &(kh_value( emgJointsList, (khint_t) jointID ));
  
  for( size_t muscleIndex = 0; muscleIndex < joint->muscleGroupsSizesList[ MUSCLE_AGONIST ]; muscleIndex++ )
    jointTorque += fabs( EMGProcessing.GetMuscleTorque( joint->muscleIDsTable[ MUSCLE_AGONIST ][ muscleIndex ], jointAngle ) );
  
  for( size_t muscleIndex = 0; muscleIndex < joint->muscleGroupsSizesList[ MUSCLE_ANTAGONIST ]; muscleIndex++ )
    jointTorque -= fabs( EMGProcessing.GetMuscleTorque( joint->muscleIDsTable[ MUSCLE_ANTAGONIST ][ muscleIndex ], jointAngle ) );
  
  return jointTorque;
}

double GetStiffness( int jointID, double jointAngle )
{
  if( !kh_exist( emgJointsList, (khint_t) jointID ) ) return 0.0;
  
  const double scaleFactor = 1.0;
  
  double jointStiffness = 0.0;
  
  EMGJoint* joint = &(kh_value( emgJointsList, (khint_t) jointID ));
  
  for( size_t muscleGroupID = 0; muscleGroupID < MUSCLE_GROUPS_NUMBER; muscleGroupID++ )
  {
    for( size_t muscleIndex = 0; muscleIndex < joint->muscleGroupsSizesList[ muscleGroupID ]; muscleIndex++ )
      jointStiffness += fabs( EMGProcessing.GetMuscleTorque( joint->muscleIDsTable[ muscleGroupID ][ muscleIndex ], jointAngle ) );
  }
  
  jointStiffness *= scaleFactor;
  
  //DEBUG_PRINT( "joint stiffness: %.3f + %.3f = %.3f", EMGProcessing_GetMuscleTorque( 0, jointAngle ), EMGProcessing_GetMuscleTorque( 1, jointAngle ), jointStiffness );
  
  if( jointStiffness < 0.0 ) jointStiffness = 0.0;
  
  return jointStiffness;
}

void ChangeState( int jointID, enum MuscleGroups muscleGroupID, enum EMGProcessPhase emgProcessingPhase )
{
  if( !kh_exist( emgJointsList, (khint_t) jointID ) ) return;
  
  EMGJoint* joint = &(kh_value( emgJointsList, (khint_t) jointID ));
  
  if( muscleGroupID < 0 || muscleGroupID >= MUSCLE_GROUPS_NUMBER ) return;

  for( size_t muscleIndex = 0; muscleIndex < joint->muscleGroupsSizesList[ muscleGroupID ]; muscleIndex++ )
    EMGProcessing.ChangePhase( joint->muscleIDsTable[ muscleGroupID ][ muscleIndex ], emgProcessingPhase );
}


const char* MUSCLE_GROUP_NAMES[ MUSCLE_GROUPS_NUMBER ] = { "agonist", "antagonist" };
static EMGJoint* LoadEMGJointData( const char* configFileName )
{
  DEBUG_PRINT( "Trying to load joint %s EMG data", configFileName );
  
  static EMGJoint emgJointData;
  static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  bool loadError = false;
  
  memset( &emgJointData, 0, sizeof(EMGJoint) );
  
  FileParser parser = JSONParser;
  int configFileID = parser.OpenFile( configFileName );
  if( configFileID != -1 )
  {
    parser.SetBaseKey( configFileID, "muscle_groups" );
    for( size_t muscleGroupIndex = 0; muscleGroupIndex < MUSCLE_GROUPS_NUMBER; muscleGroupIndex++ )
    {
      if( (emgJointData.muscleGroupsSizesList[ muscleGroupIndex ] = (size_t) parser.GetListSize( configFileID, MUSCLE_GROUP_NAMES[ muscleGroupIndex ] )) > 0 )
      {
        emgJointData.muscleIDsTable[ muscleGroupIndex ] = (int*) calloc( emgJointData.muscleGroupsSizesList[ muscleGroupIndex ], sizeof(int) );
      
        for( size_t muscleIndex = 0; muscleIndex < emgJointData.muscleGroupsSizesList[ muscleGroupIndex ]; muscleIndex++ )
        {
          sprintf( searchPath, "%s.%u", MUSCLE_GROUP_NAMES[ muscleGroupIndex ], muscleIndex );
          emgJointData.muscleIDsTable[ muscleGroupIndex ][ muscleIndex ] = EMGProcessing.InitSensor( parser.GetStringValue( configFileID, searchPath ) );
        }
      }
    }
    
    parser.CloseFile( configFileID );
  }
  else
  {
    DEBUG_PRINT( "configuration for joint %s not found", configFileName );
    loadError = true;
  }
  
  if( loadError )
  {
    UnloadEMGJointData( &emgJointData );
    return NULL;
  }
    
  return &emgJointData;
}

static void UnloadEMGJointData( EMGJoint* joint )
{
  if( joint == NULL ) return;
  
  for( size_t muscleGroupIndex = 0; muscleGroupIndex < MUSCLE_GROUPS_NUMBER; muscleGroupIndex++ )
  {
    for( size_t muscleIndex = 0; muscleIndex < joint->muscleGroupsSizesList[ muscleGroupIndex ]; muscleIndex++ )
      EMGProcessing.EndSensor( joint->muscleIDsTable[ muscleGroupIndex ][ muscleIndex ] );
    free( joint->muscleIDsTable[ muscleGroupIndex ] );
  }
}

#endif /* EMG_JOINT_CONTROL_H */
