#ifndef EMG_JOINT_CONTROL_H
#define EMG_JOINT_CONTROL_H

#include "async_debug.h"

#include "axis_control.h"
#include "emg_processing.h"

enum { MUSCLE_AGONIST, MUSCLE_ANTAGONIST, MUSCLE_GROUPS_NUMBER };

typedef struct _EMGAxisControl
{
  unsigned int axisID;
  unsigned int* muscleIDsTable[ MUSCLE_GROUPS_NUMBER ];
  size_t muscleGroupsSizesList[ MUSCLE_GROUPS_NUMBER ];
}
EMGAxisControl;

static EMGAxisControl* emgAxisControlsList = NULL;
static size_t emgAxisControlsNumber = 0;

static void LoadEMGControlsProperties();

void EMGAxisControl_Init()
{
  EMGProcessing_Init();
  AxisControl_Init();
  
  LoadEMGControlsProperties();
}

void EMGAxisControl_End()
{
  AxisControl_End();
  EMGProcessing_End();
  
  if( emgAxisControlsList != NULL )
  {
    for( size_t controlID = 0; controlID < emgAxisControlsNumber; controlID++ )
    {
      EMGAxisControl* control = &(emgAxisControlsList[ controlID ]);
      
      for( size_t muscleGroupID = 0; muscleGroupID < MUSCLE_GROUPS_NUMBER; muscleGroupID++ )
      {
        if( control->muscleIDsTable[ muscleGroupID ] != NULL )
          free( control->muscleIDsTable[ muscleGroupID ] );
      }
    }
    free( emgAxisControlsList );
    emgAxisControlsList = NULL;
    emgAxisControlsNumber = 0;
  }
}

static void LoadEMGControlsProperties()
{
  char readBuffer[ MUSCLE_NAME_MAX_LEN ];
  
  EMGAxisControl* newControl = NULL;
  
  FILE* configFile = fopen( "../config/emg_control_config.txt", "r" );
  if( configFile != NULL ) 
  {
    while( fscanf( configFile, "%s", readBuffer ) != EOF )
    {
      if( strcmp( readBuffer, "BEGIN_EMG_AXIS_CONTROL" ) == 0 )
      {
        emgAxisControlsList = (EMGAxisControl*) realloc( emgAxisControlsList, sizeof(EMGAxisControl) * ( emgAxisControlsNumber + 1 ) );
  
        newControl = &(emgAxisControlsList[ emgAxisControlsNumber ]);
  
        fscanf( configFile, "%s", readBuffer );
        
        newControl->axisID = (unsigned int) AxisControl_GetAxisID( readBuffer );
        
        DEBUG_PRINT( "found EMG axis control %s (axis ID: %d)", readBuffer, newControl->axisID );
        //DEBUG_EVENT( 0, "found EMG axis control %s (axis ID: %d)", readBuffer, newControl->axisID );
        
        for( size_t muscleGroupID = 0; muscleGroupID < MUSCLE_GROUPS_NUMBER; muscleGroupID++ )
        {
          newControl->muscleIDsTable[ muscleGroupID ] = NULL;
          newControl->muscleGroupsSizesList[ muscleGroupID ] = 0;
        }
      }
      
      if( newControl == NULL ) continue;
      
      if( strcmp( readBuffer, "agonist_muscle:" ) == 0 )
      {
        fscanf( configFile, "%s", readBuffer );
        
        int muscleID = EMGProcessing_GetMuscleID( readBuffer );
        
        if( muscleID != -1 )
        {
          newControl->muscleIDsTable[ MUSCLE_AGONIST ] = (unsigned int*) realloc( newControl->muscleIDsTable[ MUSCLE_AGONIST ], 
                                                                                  sizeof(unsigned int) * ( newControl->muscleGroupsSizesList[ MUSCLE_AGONIST ] + 1 ) );
          
          newControl->muscleIDsTable[ MUSCLE_AGONIST ][ newControl->muscleGroupsSizesList[ MUSCLE_AGONIST ] ] = (unsigned int) muscleID;
          
          DEBUG_PRINT( "found joint agonist muscle %u: %s (muscle ID: %u)", newControl->muscleGroupsSizesList[ MUSCLE_AGONIST ], readBuffer, 
                                                                            newControl->muscleIDsTable[ MUSCLE_AGONIST ][ newControl->muscleGroupsSizesList[ MUSCLE_AGONIST ] ] );
          //DEBUG_EVENT( 2, "found joint agonist muscle %s (ID: %u)", readBuffer, newControl->muscleIDsTable[ MUSCLE_AGONIST ][ newControl->muscleGroupsSizesList[ MUSCLE_AGONIST ] ] );
          
          newControl->muscleGroupsSizesList[ MUSCLE_AGONIST ]++;
        }
      }
      else if( strcmp( readBuffer, "antagonist_muscle:" ) == 0 )
      {
        fscanf( configFile, "%s", readBuffer );
        
        int muscleID = EMGProcessing_GetMuscleID( readBuffer );
        
        if( muscleID != -1 )
        {
          newControl->muscleIDsTable[ MUSCLE_ANTAGONIST ] = (unsigned int*) realloc( newControl->muscleIDsTable[ MUSCLE_ANTAGONIST ], 
                                                                                     sizeof(unsigned int) * ( newControl->muscleGroupsSizesList[ MUSCLE_ANTAGONIST ] + 1 ) );
          
          newControl->muscleIDsTable[ MUSCLE_ANTAGONIST ][ newControl->muscleGroupsSizesList[ MUSCLE_ANTAGONIST ] ] = (unsigned int) muscleID;
          
          DEBUG_PRINT( "found joint antagonist muscle %u: %s (muscle ID: %u)", newControl->muscleGroupsSizesList[ MUSCLE_ANTAGONIST ], readBuffer, 
                                                                               newControl->muscleIDsTable[ MUSCLE_ANTAGONIST ][ newControl->muscleGroupsSizesList[ MUSCLE_ANTAGONIST ] ] );
          //DEBUG_EVENT( 3, "found joint antagonist muscle %s (ID: %u)", readBuffer, newControl->muscleIDsTable[ MUSCLE_ANTAGONIST ][ newControl->muscleGroupsSizesList[ MUSCLE_ANTAGONIST ] ] );
          
          newControl->muscleGroupsSizesList[ MUSCLE_ANTAGONIST ]++;
        }
      }
      else if( strcmp( readBuffer, "END_EMG_AXIS_CONTROL" ) == 0 )
      {
        if( (int) newControl->axisID != -1 )
        {
          emgAxisControlsNumber++;
        
          DEBUG_EVENT( 4, "%u joint controls added", emgAxisControlsNumber );
        }
      }
      else if( strcmp( readBuffer, "#" ) == 0 )
      {
        char dummyChar;
        
        do { 
          if( fscanf( configFile, "%c", &dummyChar ) == EOF ) 
            break; 
        } while( dummyChar != '\n' ); 
      }
    }
    
    fclose( configFile );
  }
}

static inline EMGAxisControl* FindAxisControl( unsigned int axisID )
{
  for( size_t controlID = 0; controlID < emgAxisControlsNumber; controlID++ )
  {
    EMGAxisControl* control = &(emgAxisControlsList[ controlID ]);
    
    if( control->axisID == axisID )
      return control;
  }
  
  return NULL;
}

double EMGAxisControl_GetTorque( unsigned int axisID )
{
  double jointTorque = 0.0;
  
  EMGAxisControl* control = FindAxisControl( axisID );
  
  if( control == NULL ) return 0.0;
  
  double* motorMeasuresList = AxisControl_GetSensorMeasures( axisID );
  
  if( motorMeasuresList == NULL ) return 0.0;
  
  double jointAngle = -motorMeasuresList[ POSITION ] * 180.0 / PI;
  
  for( size_t muscleIndex = 0; muscleIndex < control->muscleGroupsSizesList[ MUSCLE_AGONIST ]; muscleIndex++ )
    jointTorque += EMGProcessing_GetMuscleTorque( control->muscleIDsTable[ MUSCLE_AGONIST ][ muscleIndex ], jointAngle );
  
  for( size_t muscleIndex = 0; muscleIndex < control->muscleGroupsSizesList[ MUSCLE_ANTAGONIST ]; muscleIndex++ )
    jointTorque -= EMGProcessing_GetMuscleTorque( control->muscleIDsTable[ MUSCLE_ANTAGONIST ][ muscleIndex ], jointAngle );
  
  return jointTorque;
}

double EMGAxisControl_GetStiffness( unsigned int axisID )
{
  const double scaleFactor = 1.0;
  
  double jointStiffness = 0.0;
  
  EMGAxisControl* control = FindAxisControl( axisID );
  
  if( control == NULL ) return 0.0;
  
  double* motorMeasuresList = AxisControl_GetSensorMeasures( axisID );
  
  if( motorMeasuresList == NULL ) return 0.0;
  
  double jointAngle = -motorMeasuresList[ POSITION ] * 180.0 / PI;
  
  for( size_t muscleGroupID = 0; muscleGroupID < MUSCLE_GROUPS_NUMBER; muscleGroupID++ )
  {
    for( size_t muscleIndex = 0; muscleIndex < control->muscleGroupsSizesList[ muscleGroupID ]; muscleIndex++ )
      jointStiffness += EMGProcessing_GetMuscleTorque( control->muscleIDsTable[ muscleGroupID ][ muscleIndex ], jointAngle );
  }
  
  jointStiffness *= scaleFactor;
  
  //DEBUG_PRINT( "joint stiffness: %.3f + %.3f = %.3f", EMGProcessing_GetMuscleTorque( 0, jointAngle ), EMGProcessing_GetMuscleTorque( 1, jointAngle ), jointStiffness );
  
  if( jointStiffness < 0.0 ) jointStiffness = 0.0;
  
  return jointStiffness;
}

double* EMGAxisControl_ApplyGains( unsigned int axisID, double maxGain )
{
  const double forgettingFactor = 0.9;
  
  static double controlParametersList[ CONTROL_PARAMS_NUMBER ];
  
  EMGAxisControl* control = FindAxisControl( axisID );
  
  if( control == NULL ) return NULL;
  
  double* motorParametersList = AxisControl_GetParameters( axisID );
  
  double jointStiffness = EMGAxisControl_GetStiffness( axisID );
  if( jointStiffness > maxGain ) jointStiffness = maxGain;
  
  controlParametersList[ PROPORTIONAL_GAIN ] = forgettingFactor * motorParametersList[ PROPORTIONAL_GAIN ] + ( 1 - forgettingFactor ) * ( maxGain - jointStiffness );
      
  AxisControl_SetParameters( axisID, controlParametersList );
  
  return motorParametersList;
}

void EMGAxisControl_ChangeState( unsigned int axisID, int muscleGroupID, int emgProcessingPhase )
{
  EMGAxisControl* control = FindAxisControl( axisID );
  
  if( control == NULL ) return;
  
  if( muscleGroupID < 0 || muscleGroupID >= MUSCLE_GROUPS_NUMBER ) return;

  for( size_t muscleIndex = 0; muscleIndex < control->muscleGroupsSizesList[ muscleGroupID ]; muscleIndex++ )
    EMGProcessing_ChangePhase( control->muscleIDsTable[ muscleGroupID ][ muscleIndex ], emgProcessingPhase );
}

#endif /* EMG_JOINT_CONTROL_H */
