#ifndef ROBREHAB_EMG_H
#define ROBREHAB_EMG_H   

#include "shm_control.h"
#include "shm_emg_control.h" 
#include "emg_joint_control.h"

#include <analysis.h>
#include <utility.h>
#include <cvinetv.h>
#include <cvirte.h>
#include <ansi_c.h>
#include <rtutil.h>

#include "config_parser.h"

#include "klib/kvec.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

const unsigned long UPDATE_INTERVAL_MS = 5;

typedef struct _SHMJointData
{
  SHMController controller;
  int jointID;
}
SHMJointData;

typedef SHMJointData* SHMJoint;

kvec_t( SHMJointData ) sharedJointsList;


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
          //sprintf( searchPath, "%s/%s", SHM_EMG_BASE_KEY, jointName );
          if( (newSharedJointID = EMGJointControl.InitJoint( jointName )) != EMG_JOINT_INVALID_ID )
          {
            //sprintf( searchPath, "%s-%s", SHM_AXIS_BASE_KEY, jointName );
            if( (newSharedAxis = SHMControl.InitData( jointName, SHM_CONTROL_OUT )) != NULL )
            {
              SHMJointData newSharedJoint = { .controller = newSharedAxis, .jointID = newSharedJointID };
            
              kv_push( SHMJointData, sharedJointsList, newSharedJoint );

              DEBUG_PRINT( "Got new shared joint with ID %d and axis %p (total: %u)", newSharedJointID, newSharedAxis, kv_size( sharedJointsList ) );
            }
            else loadError = true;
          }
          else loadError = true;

          if( loadError )
          {
            SHMControl.EndData( newSharedAxis );
            EMGJointControl.EndJoint( newSharedJointID );
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
    if( SHMControl.GetNumericValue( sharedJoint->controller, SHM_AXIS_POSITION, &jointAngle, SHM_CONTROL_PEEK ) )
    {
      double jointTorque = EMGJointControl.GetTorque( sharedJoint->jointID, jointAngle );
      double jointStiffness = EMGJointControl.GetStiffness( sharedJoint->jointID, jointAngle );

      DEBUG_PRINT( "Joint position: %.3f - torque: %.3f - stiffness: %.3f", jointAngle, jointTorque, jointStiffness );

      SHMControl.SetNumericValue( sharedJoint->controller, SHM_AXIS_STIFFNESS, jointStiffness );
    }
  }
}


#endif // ROBREHAB_EMG_H   
