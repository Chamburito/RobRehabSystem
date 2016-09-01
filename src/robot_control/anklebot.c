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


#include "robot_control/interface.h"

#define DOFS_NUMBER 2

const char* AXIS_NAMES[ DOFS_NUMBER ] = { "DP", "IE" };
const char* JOINT_NAMES[ DOFS_NUMBER ] = { "RIGHT", "LEFT" };

DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE ) 

Controller InitController( const char* data, const char* logDirectory )
{
  return NULL;
}

void EndController( Controller controller )
{
  
}

size_t GetJointsNumber( Controller controller )
{
  return DOFS_NUMBER;
}

char** GetJointNamesList( Controller controller )
{
  return (char**) JOINT_NAMES;
}

size_t GetAxesNumber( Controller controller )
{
  return DOFS_NUMBER;
}

char** GetAxisNamesList( Controller controller )
{
  return (char**) AXIS_NAMES;
}

void SetControlState( Controller ref_controller, enum ControlState controlState )
{
  fprintf( stderr, "Setting robot control phase: %x\n", controlState );
}

void RunControlStep( Controller controller, double** jointMeasuresTable, double** axisMeasuresTable, double** jointSetpointsTable, double** axisSetpointsTable )
{
  const double BALL_LENGTH = 0.14;
  const double BALL_BALL_WIDTH = 0.19;
  const double SHIN_LENGTH = 0.42;
  const double ACTUATOR_LENGTH = 0.443;
  
  double positionMean = ( jointMeasuresTable[ 0 ][ CONTROL_POSITION ] + jointMeasuresTable[ 1 ][ CONTROL_POSITION ] ) / 2.0;
  double dpSin = ( pow( BALL_LENGTH, 2 ) + pow( SHIN_LENGTH, 2 ) - pow( ACTUATOR_LENGTH - positionMean, 2 ) ) / ( 2 * BALL_LENGTH * SHIN_LENGTH );
  if( dpSin > 1.0 ) dpSin = 1.0;
  else if( dpSin < -1.0 ) dpSin = -1.0;
  axisMeasuresTable[ 0 ][ CONTROL_POSITION ] = asin( dpSin ); // + dpOffset; ?
  
  double positionDiff = ( jointMeasuresTable[ 0 ][ CONTROL_POSITION ] - jointMeasuresTable[ 1 ][ CONTROL_POSITION ] );
  axisMeasuresTable[ 1 ][ CONTROL_POSITION ] = atan( positionDiff / BALL_BALL_WIDTH );  // + ieOffset; ? 
  
  double dpRefStiffness = axisSetpointsTable[ 0 ][ CONTROL_STIFFNESS ]; 
  double dpPositionError = axisSetpointsTable[ 0 ][ CONTROL_POSITION ] - axisMeasuresTable[ 0 ][ CONTROL_POSITION ];
  double dpRefDamping = axisSetpointsTable[ 0 ][ CONTROL_DAMPING ];
  double dpVelocity = axisSetpointsTable[ 0 ][ CONTROL_VELOCITY ];
  double dpRefTorque = dpRefStiffness * dpPositionError + dpRefDamping * dpVelocity;
  axisSetpointsTable[ 0 ][ CONTROL_FORCE ] = dpRefTorque;
  
  double ieRefStiffness = axisSetpointsTable[ 1 ][ CONTROL_STIFFNESS ]; 
  double iePositionError = axisSetpointsTable[ 1 ][ CONTROL_POSITION ] - axisMeasuresTable[ 1 ][ CONTROL_POSITION ];
  double ieRefDamping = axisSetpointsTable[ 1 ][ CONTROL_DAMPING ];
  double ieVelocity = axisSetpointsTable[ 1 ][ CONTROL_VELOCITY ];
  double ieRefTorque = ieRefStiffness * iePositionError + ieRefDamping * ieVelocity;
  axisSetpointsTable[ 1 ][ CONTROL_FORCE ] = ieRefTorque;
  
  double dpRefForce = dpRefTorque / BALL_LENGTH;
  double ieRefForce = ieRefTorque / ( BALL_BALL_WIDTH / 2.0 );
  jointSetpointsTable[ 0 ][ CONTROL_FORCE ] = ( -dpRefForce - ieRefForce ) / 2.0;
  jointSetpointsTable[ 1 ][ CONTROL_FORCE ] = ( -dpRefForce + ieRefForce ) / 2.0;
  
  double rightForce = jointMeasuresTable[ 0 ][ CONTROL_FORCE ];
  if( jointSetpointsTable[ 0 ][ CONTROL_FORCE ] < 0.0 ) rightForce = -rightForce;
  double leftForce = jointMeasuresTable[ 1 ][ CONTROL_FORCE ];
  if( jointSetpointsTable[ 1 ][ CONTROL_FORCE ] < 0.0 ) leftForce = -leftForce;
  axisMeasuresTable[ 0 ][ CONTROL_FORCE ] = ( leftForce + rightForce ) * BALL_LENGTH;
  axisMeasuresTable[ 1 ][ CONTROL_FORCE ] = ( leftForce - rightForce ) * BALL_BALL_WIDTH / 2.0;
}
