#include "emg_processing.h"
#include "emg_joint_control.h"

#include <analysis.h>
#include <utility.h>
#include <cvinetv.h>
#include <cvirte.h>
#include <ansi_c.h>
#include <rtutil.h>

/*#include "utils/file_parsing/json_parser.h"

#include "async_debug.h"

typedef struct _SHMJointControl
{
  int shmAxisControlDataID;
  int emgJointControlID;
}
SHMJointControl;

KHASH_MAP_INIT_INT( SHMJointControl, SHMJointControl )
static khash_t( SHMJointControl )* shmAxisControlsList = NULL;

int RobRehabControl_Init( void );*/

/* Program entry-point */
void CVIFUNC_C RTmain( void )
{
  int jointID = EMGJointControl.InitJoint( "../config/emg_joint/knee" );
  if( jointID != -1 )
  {
    double jointTorque = EMGJointControl.GetTorque( jointID, 0.0 );
    double jointStiffness = EMGJointControl.GetStiffness( jointID, 0.0 );

    DEBUG_PRINT( "Joint torque: %g", jointTorque );
    DEBUG_PRINT( "Joint stiffness: %g", jointStiffness );
    
    Timing_Delay( 10000 );

    EMGJointControl.EndJoint( jointID );
  }
}


/*int RobRehabControl_Init()
{
  int shmAxisControlDataID, emgJointControlID;
  bool loadError = false;
  
  shmAxisControlsList = kh_init( SHMJointControl );
  
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
        
        if( (emgJointControlID = AxisControl.Init( deviceName )) != -1 )
        {
          sprintf( searchPath, "axes.%u.key", shmAxisDataIndex );
          int shmKey = (int) parser.GetIntegerValue( configFileID, searchPath );
          
          if( (shmAxisControlDataID = ShmAxisControl_Init( shmKey )) != -1 )
          {
            int insertionStatus;
            khint_t shmAxisDataID = kh_put( SHMJointControlControl, shmAxisControlsList, shmAxisControlDataID, &insertionStatus );
            if( insertionStatus != -1 )
            {
              kh_value( shmAxisControlsList, shmAxisDataID ).shmAxisControlDataID = shmAxisControlDataID;
              kh_value( shmAxisControlsList, shmAxisDataID ).emgJointControlID = emgJointControlID;
            }
            else loadError = true;
          }
          else loadError = true;
        }
        else loadError = true;
        
      }
    }
    else loadError = true;
    
    parser.CloseFile( configFileID );
  }
  else loadError = true;
  
  if( loadError )
  {
    ShmAxisControl_End( shmAxisControlDataID );
    AxisControl.End( emgJointControlID );
    return -1;
  }
  
  //DEBUG_PRINT( "Created axes info string: %s", axesInfoString );
  
  /*DEBUG_EVENT( 0,DEBUG_PRINT( "RobRehab Network initialized on thread %x", THREAD_ID );
  
  return 0;
}*/
