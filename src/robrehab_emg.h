#ifndef ROBREHAB_EMG_H
#define ROBREHAB_EMG_H   

#include "shm_axis_control.h"
#include "emg_joint_control.h"

#include <analysis.h>
#include <utility.h>
#include <cvinetv.h>
#include <cvirte.h>
#include <ansi_c.h>
#include <rtutil.h>

#include "file_parsing/json_parser.h"

#include "klib/kvec.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

typedef struct _SHMJointData
{
  SHMAxisController controller;
  int jointID;
}
SHMJointData;

typedef SHMJointData* SHMJoint;

kvec_t( SHMJointData ) sharedJointsList;

#define NAMESPACE RobRehabEMG

#define NAMESPACE_FUNCTIONS( namespace ) \
        NAMESPACE_FUNCTION( int, namespace, Init, void ) \
        NAMESPACE_FUNCTION( void, namespace, End, void ) \
        NAMESPACE_FUNCTION( void, namespace, Update, void ) \

#define NAMESPACE_FUNCTION( rvalue, namespace, name, ... ) static rvalue namespace##_##name( __VA_ARGS__ );
NAMESPACE_FUNCTIONS( NAMESPACE )
#undef NAMESPACE_FUNCTION

#define NAMESPACE_FUNCTION( rvalue, namespace, name, ... ) rvalue (*name)( __VA_ARGS__ );
const struct { NAMESPACE_FUNCTIONS( NAMESPACE ) }
#undef NAMESPACE_FUNCTION
#define NAMESPACE_FUNCTION( rvalue, namespace, name, ... ) .name = namespace##_##name,
NAMESPACE = { NAMESPACE_FUNCTIONS( NAMESPACE ) };
#undef NAMESPACE_FUNCTION

#undef NAMESPACE_FUNCTIONS
#undef NAMESPACE

const char* jointsDirectory = "emg_joints";
int RobRehabEMG_Init()
{
  kv_init( sharedJointsList );
  
  SET_PATH( "./config/" );
  
  FileParser parser = JSONParser;
  int configFileID = parser.LoadFile( "shared_axes" );
  if( configFileID != -1 )
  {
    if( parser.HasKey( configFileID, "axes" ) )
    {
      size_t sharedAxesNumber = parser.GetListSize( configFileID, "axes" );
      
      DEBUG_PRINT( "List size: %u", sharedAxesNumber );
      
      char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
      for( size_t shmAxisDataIndex = 0; shmAxisDataIndex < sharedAxesNumber; shmAxisDataIndex++ )
      {
        sprintf( searchPath, "axes.%u", shmAxisDataIndex );
        char* axisName = parser.GetStringValue( configFileID, searchPath );

        DEBUG_PRINT( "found axis %s", axisName );

        bool loadError = false;

        sprintf( searchPath, "%s/%s", jointsDirectory, axisName );

        int newSharedJointID = -1;
        SHMAxisController newSharedController = NULL;
        if( (newSharedJointID = EMGJointControl.InitJoint( searchPath )) != -1 )
        {
          if( (newSharedController = SHMAxisControl.InitController( axisName )) != NULL )
          {
            SHMJointData newSharedJoint = { .controller = newSharedController, .jointID = newSharedJointID };
            
            kv_push( SHMJointData, sharedJointsList, newSharedJoint );

            DEBUG_PRINT( "Got new shared joint with ID %d and axis controller %p (total: %u)", newSharedJointID, newSharedController, kv_size( sharedJointsList ) );
          }
          else loadError = true;
        }
        else loadError = true;

        if( loadError )
        {
          SHMAxisControl.EndController( newSharedController );
          EMGJointControl.EndJoint( newSharedJointID );
          return -1;
        }
      }
    }
    
    parser.UnloadFile( configFileID );
  }
  
  SET_PATH( ".." );
  
  return 0;
}

void RobRehabEMG_End()
{
  DEBUG_PRINT( "cleaning shared joints vector %p", sharedJointsList.a );
  
  for( size_t sharedJointID = 0; sharedJointID < kv_size( sharedJointsList ); sharedJointID++ )
  {
    DEBUG_PRINT( "ending joint control %u", sharedJointID ); 
      
    SHMJoint sharedJoint = &(kv_A( sharedJointsList, sharedJointID ));
      
    SHMAxisControl.EndController( sharedJoint->controller );
    EMGJointControl.EndJoint( sharedJoint->jointID );
  }
  
  kv_destroy( sharedJointsList );
}

void RobRehabEMG_Update( void )
{
  for( size_t sharedJointID = 0; sharedJointID < kv_size( sharedJointsList ); sharedJointID++ )
  {
    SHMJoint sharedJoint = &(kv_A( sharedJointsList, sharedJointID ));

    float jointAngle;
    SHMAxisControl.GetNumericValue( sharedJoint->controller, SHM_CONTROL_OUT, SHM_CONTROL_POSITION, &jointAngle, SHM_PEEK );
    //if( SHMAxisControl.GetNumericValue( sharedJoint->controller, SHM_CONTROL_OUT, SHM_CONTROL_POSITION, &jointAngle, SHM_PEEK ) )
    //{
      //double jointTorque = EMGJointControl.GetTorque( sharedJoint->jointID, jointAngle );
      double jointStiffness = EMGJointControl.GetStiffness( sharedJoint->jointID, jointAngle );

      //DEBUG_PRINT( "Joint position: %.3f - torque: %.3f - stiffness: %.3f", jointAngle, jointTorque, jointStiffness );

      SHMAxisControl.SetNumericValue( sharedJoint->controller, SHM_CONTROL_IN, SHM_CONTROL_STIFFNESS, jointStiffness );
    //}
  }
}


#endif // ROBREHAB_EMG_H   
