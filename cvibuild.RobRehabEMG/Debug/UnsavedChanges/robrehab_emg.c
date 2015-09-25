#include "shm_axis_control.h"
#include "emg_joint_control.h"

#include <analysis.h>
#include <utility.h>
#include <cvinetv.h>
#include <cvirte.h>
#include <ansi_c.h>
#include <rtutil.h>

#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

typedef struct _SHMJointControl
{
  int shmAxisControlDataID;
  int emgJointControlID;
}
SHMJointControl;

KHASH_MAP_INIT_STR( SHMJointControl, SHMJointControl )
static khash_t( SHMJointControl )* shmJointControlsList = NULL;

int Init( void );
void End( void );

/* Program entry-point */
void CVIFUNC_C RTmain( void )
{
  int logID = DataLogging.InitLog( "../logs/joints/test", 3 );
  
  Init();
  
  while( !RTIsShuttingDown() )
  {
    for( khint_t shmJointControlID = 0; shmJointControlID != kh_end( shmJointControlsList ); shmJointControlID++ )
    {
      if( kh_exist( shmJointControlsList, shmJointControlID ) )
      {
        SHMJointControl* jointControl = &(kh_value( shmJointControlsList, shmJointControlID ));

        double* measuresList = SHMAxisControl.GetNumericValuesList( jointControl->shmAxisControlDataID, CONTROL_MEASURES, REMOVE, NULL );
        if( measuresList != NULL )
        {
          double jointTorque = EMGJointControl.GetTorque( jointControl->emgJointControlID, measuresList[ CONTROL_POSITION ] );
          double jointStiffness = EMGJointControl.GetStiffness( jointControl->emgJointControlID, measuresList[ CONTROL_POSITION ] );

          DEBUG_PRINT( "Joint position: %.3f - torque: %.3f - stiffness: %.3f", measuresList[ CONTROL_POSITION ], jointTorque, jointStiffness );
          
          DataLogging.RegisterValues( logID, measuresList[ CONTROL_POSITION ], jointTorque, jointStiffness );
        }
        
        SHMAxisControl.SetNumericValues( jointControl->shmAxisControlDataID, CONTROL_MEASURES, NULL );
      }
    }

    SleepUntilNextMultipleUS( 5000 );
  }
  
  DataLogging.EndLog( logID );

  End();
}


int Init()
{
  int shmAxisControlDataID = -1, emgJointControlID = -1;
  bool loadError = false;
  
  shmJointControlsList = kh_init( SHMJointControl );
  
  SET_PATH( "../config/" );
  
  FileParser parser = JSONParser;
  int configFileID = parser.LoadFile( "shared_axes" );
  if( configFileID != -1 )
  {
    if( parser.HasKey( configFileID, "axes" ) )
    {
      size_t shmAxesNumber = parser.GetListSize( configFileID, "axes" );
      
      DEBUG_PRINT( "List size: %u", shmAxesNumber );
      
      char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
      for( size_t shmAxisDataIndex = 0; shmAxisDataIndex < shmAxesNumber; shmAxisDataIndex++ )
      {
        sprintf( searchPath, "axes.%u", shmAxisDataIndex );
        char* deviceName = parser.GetStringValue( configFileID, searchPath );
        
        DEBUG_PRINT( "found axis %s", deviceName );
        
        if( kh_get( SHMJointControl, shmJointControlsList, deviceName ) == kh_end( shmJointControlsList ) )
        {
          if( (emgJointControlID = EMGJointControl.InitJoint( deviceName )) != -1 )
          {
            if( (shmAxisControlDataID = SHMAxisControl.InitControllerData( deviceName )) != -1 )
            {
              int insertionStatus;
              khint_t shmJointControlID = kh_put( SHMJointControl, shmJointControlsList, deviceName, &insertionStatus );
              
              DEBUG_PRINT( "Got hash table iterator %u (insertion status: %d)", shmJointControlID, insertionStatus );

              kh_value( shmJointControlsList, shmJointControlID ).shmAxisControlDataID = shmAxisControlDataID;
              kh_value( shmJointControlsList, shmJointControlID ).emgJointControlID = emgJointControlID;
              
              DEBUG_PRINT( "Got joint %d and axis %d", kh_value( shmJointControlsList, shmJointControlID ).emgJointControlID,
                                                       kh_value( shmJointControlsList, shmJointControlID ).shmAxisControlDataID );
            }
            else loadError = true;
          }
          else loadError = true;
        
          if( loadError )
          {
            SHMAxisControl.EndControllerData( shmAxisControlDataID );
            EMGJointControl.EndJoint( emgJointControlID );
            return -1;
          }
        }
      }
    }
    
    parser.UnloadFile( configFileID );
  }
  
  return 0;
}

void End()
{
  DEBUG_PRINT( "cleaning hash table %p from %u to %u", shmJointControlsList, kh_begin( shmJointControlsList ), kh_end( shmJointControlsList ) );
  
  for( khint_t shmJointControlID = 0; shmJointControlID != kh_end( shmJointControlsList ); shmJointControlsList++ )
  {
    DEBUG_PRINT( "searching joint control %u", shmJointControlID );
    if( kh_exist( shmJointControlsList, shmJointControlID ) )
    {
      DEBUG_PRINT( "ending joint control %u", shmJointControlID ); 
      
      SHMJointControl* jointControl = &(kh_value( shmJointControlsList, shmJointControlID ));
      
      SHMAxisControl.EndControllerData( jointControl->shmAxisControlDataID );
      EMGJointControl.EndJoint( jointControl->emgJointControlID );
      
      kh_del( SHMJointControl, shmJointControlsList, shmJointControlID );
    }
  }
  
  kh_destroy( SHMJointControl, shmJointControlsList );
}
