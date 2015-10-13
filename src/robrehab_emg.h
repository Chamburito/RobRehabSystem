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

#include "config_parser.h"

#include "klib/kvec.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

const unsigned long UPDATE_INTERVAL_MS = 5;

typedef struct _SHMJointData
{
  SHMAxis axis;
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


int RobRehabEMG_Init( void )
{
  kv_init( sharedJointsList );
  
  if( ConfigParser.Init( "JSON" ) )
  {
    int configFileID = ConfigParser.LoadFile( "shared_axes" );
    if( configFileID != -1 )
    {
      if( ConfigParser.HasKey( configFileID, "axes" ) )
      {
        size_t sharedAxesNumber = ConfigParser.GetListSize( configFileID, "axes" );
      
        DEBUG_PRINT( "List size: %u", sharedAxesNumber );
      
        char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
        for( size_t shmAxisDataIndex = 0; shmAxisDataIndex < sharedAxesNumber; shmAxisDataIndex++ )
        {
          sprintf( searchPath, "axes.%u", shmAxisDataIndex );
          char* axisName = ConfigParser.GetStringValue( configFileID, searchPath );

          DEBUG_PRINT( "found axis %s", axisName );

          bool loadError = false;

          int newSharedJointID = -1;
          SHMAxis newSharedAxis = NULL;
          if( (newSharedJointID = EMGJointControl.InitJoint( axisName )) != -1 )
          {
            if( (newSharedAxis = SHMAxisControl.InitData( axisName, SHM_CONTROL_OUT )) != NULL )
            {
              SHMJointData newSharedJoint = { .axis = newSharedAxis, .jointID = newSharedJointID };
            
              kv_push( SHMJointData, sharedJointsList, newSharedJoint );

              DEBUG_PRINT( "Got new shared joint with ID %d and axis %p (total: %u)", newSharedJointID, newSharedAxis, kv_size( sharedJointsList ) );
            }
            else loadError = true;
          }
          else loadError = true;

          if( loadError )
          {
            SHMAxisControl.EndData( newSharedAxis );
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
      
    SHMAxisControl.EndData( sharedJoint->axis );
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
    if( SHMAxisControl.GetNumericValue( sharedJoint->axis, SHM_CONTROL_POSITION, &jointAngle, SHM_PEEK ) )
    {
      double jointTorque = EMGJointControl.GetTorque( sharedJoint->jointID, jointAngle );
      double jointStiffness = EMGJointControl.GetStiffness( sharedJoint->jointID, jointAngle );

      DEBUG_PRINT( "Joint position: %.3f - torque: %.3f - stiffness: %.3f", jointAngle, jointTorque, jointStiffness );

      SHMAxisControl.SetNumericValue( sharedJoint->axis, SHM_CONTROL_STIFFNESS, jointStiffness );
    }
  }
}


#endif // ROBREHAB_EMG_H   
