#include "emg_processing.h"
#include "emg_joint_control.h"

int main( int argc, char* argv[] )
{
  if( argc >= 3 )
  {
    int jointID = EMGJointControl.InitJoint( argv[ 1 ] );
    if( jointID != -1 )
    {
      double jointAngle = strtod( argv[ 2 ], NULL );
      
      DEBUG_PRINT( "Joint angle: %g", jointAngle );
      
      double jointTorque = EMGJointControl.GetTorque( jointID, jointAngle );
      double jointStiffness = EMGJointControl.GetStiffness( jointID, jointAngle );
      
      DEBUG_PRINT( "Joint torque: %g", jointTorque );
      DEBUG_PRINT( "Joint stiffness: %g", jointStiffness );
      
      EMGJointControl.EndJoint( jointID );
    }
  }
  
  return 0;
}
