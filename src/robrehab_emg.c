#include "shm_axis_control.h"
#include "emg_processing.h"
#include "emg_joint_control.h"

#include <analysis.h>
#include <utility.h>
#include <cvinetv.h>
#include <cvirte.h>
#include <ansi_c.h>
#include <rtutil.h>

#include "file_parsing/json_parser.h"

#include "async_debug.h"

typedef struct _SHMJointControl
{
  int shmAxisControlDataID;
  int emgJointControlID;
}
SHMJointControl;

KHASH_MAP_INIT_INT( SHMJointControl, SHMJointControl )
static khash_t( SHMJointControl )* shmJointControlsList = NULL;

int Init( void );
void End( void );

/* Program entry-point */
void CVIFUNC_C RTmain( void )
{
  Init();
  
  while( !RTIsShuttingDown() )
  {
    for( khint_t shmJointControlID = 0; shmJointControlID != kh_end( shmJointControlsList ); shmJointControlID++ )
    {
      if( kh_exist( shmJointControlsList, shmJointControlID ) )
      {
        SHMJointControl* jointControl = &(kh_value( shmJointControlsList, shmJointControlID ));

        double jointTorque = EMGJointControl.GetTorque( jointControl->emgJointControlID, 0.0 );
        double jointStiffness = EMGJointControl.GetStiffness( jointControl->emgJointControlID, 0.0 );

        DEBUG_PRINT( "Joint torque: %g - stiffness: %g", jointTorque, jointStiffness );
      }
    }

    SleepUntilNextMultipleUS( 5000 );
  }

  End();
}


int Init()
{
  int shmAxisControlDataID, emgJointControlID;
  bool loadError = false;
  
  shmJointControlsList = kh_init( SHMJointControl );
  
  FileParser parser = JSONParser;
  int configFileID = parser.OpenFile( "../config/shared_axes" );
  if( configFileID != -1 )
  {
    if( parser.HasKey( configFileID, "axes" ) )
    {
      size_t shmAxesNumber = parser.GetListSize( configFileID, "axes" );
      
      DEBUG_PRINT( "List size: %u", shmAxesNumber );
      
      char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
      for( size_t shmAxisDataIndex = 0; shmAxisDataIndex < shmAxesNumber; shmAxisDataIndex++ )
      {
        sprintf( searchPath, "axes.%u.name", shmAxisDataIndex );
        char* deviceName = parser.GetStringValue( configFileID, searchPath );
        
        sprintf( searchPath, "axes.%u.shm_key", shmAxisDataIndex );
        int shmKey = (int) parser.GetIntegerValue( configFileID, searchPath );
        
        DEBUG_PRINT( "found axis with name %s and shm key %x", deviceName, shmKey );
        
        if( kh_get( SHMJointControl, shmJointControlsList, shmKey ) == kh_end( shmJointControlsList ) )
        {
          if( (emgJointControlID = EMGJointControl.InitJoint( deviceName )) != -1 )
          {
            //if( (shmAxisControlDataID = ShmAxisControl_Init( shmKey )) != -1 )
            //{
              int insertionStatus;
              khint_t shmJointControlID = kh_put( SHMJointControl, shmJointControlsList, shmKey, &insertionStatus );

              //kh_value( shmJointControlsList, shmJointControlID ).shmAxisControlDataID = shmAxisControlDataID;
              kh_value( shmJointControlsList, shmJointControlID ).emgJointControlID = emgJointControlID;
            //}
            //else loadError = true;
          }
          else loadError = true;
        
          if( loadError )
          {
            //SHMAxisControl_End( shmAxisControlDataID );
            EMGJointControl.EndJoint( emgJointControlID );
            return -1;
          }
        }
      }
    }
    
    parser.CloseFile( configFileID );
  }
  
  return 0;
}

void End()
{
  for( khint_t shmJointControlID = 0; shmJointControlID != kh_end( shmJointControlsList ); shmJointControlsList++ )
  {
    if( kh_exist( shmJointControlsList, shmJointControlID ) )
    {
      SHMJointControl* jointControl = &(kh_value( shmJointControlsList, shmJointControlID ));
      
      //SHMAxisControl_End( jointControl->shmAxisControlDataID );
      EMGJointControl.EndJoint( jointControl->emgJointControlID );
      
      kh_del( SHMJointControl, shmJointControlsList, shmJointControlID );
    }
  }
  
  kh_destroy( SHMJointControl, shmJointControlsList );
}
