////////////////////////////////////////////////////////////////////////////////
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
//  along with Foobar. If not, see <http://www.gnu.org/licenses/>.            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


#include "actuator_control/interface.h"

#include <string.h>

typedef struct _ControlData
{
  double positionErrorSum, positionSetpointSum;
  double velocitySetpoint;
  double forceError[ 2 ];
  double controlError;
  ControlVariables outputs;
}
ControlData;

DECLARE_MODULE_INTERFACE( ACTUATOR_CONTROL_INTERFACE )

Controller InitController( void )
{
  void* newController = malloc( sizeof(ControlData) );
  memset( newController, 0, sizeof(ControlData) );
  
  return (Controller) newController;
}

ControlVariables* RunControlStep( Controller controller, ControlVariables* measures, ControlVariables* setpoints, double* ref_error )
{
  ControlData* controlData = (ControlData*) controller;
  
  controlData->outputs.position = setpoints->position;
  
  double positionError = measures->position - setpoints->position;
  controlData->positionErrorSum += CONTROL_PASS_INTERVAL * positionError * positionError;
  controlData->positionSetpointSum += CONTROL_PASS_INTERVAL * setpoints->position * setpoints->position;
  
  if( ref_error != NULL )
  {
    if( *ref_error == 0.0 )
    {
      controlData->controlError = ( controlData->positionSetpointSum > 0.0 ) ? controlData->positionErrorSum / controlData->positionSetpointSum : 1.0;
      if( controlData->controlError > 1.0 ) controlData->controlError = 1.0;
    
      controlData->positionErrorSum = 0.0;
      controlData->positionSetpointSum = 0.0;
    }
    
    *ref_error = positionError / setpoints->position;
  }
  
  double forceSetpoint = controlData->controlError * setpoints->force;
  controlData->outputs.force = forceSetpoint;

  controlData->forceError[ 0 ] = forceSetpoint - measures->force;
  
  controlData->velocitySetpoint += 370.0 * ( controlData->forceError[ 0 ] - controlData->forceError[ 1 ] ) + 3.5 * CONTROL_PASS_INTERVAL * controlData->forceError[ 0 ];
  controlData->outputs.velocity = controlData->velocitySetpoint;
  
  //velocitySetpoint[0] = 0.9822 * velocitySetpoint[1] + 0.01407 * velocitySetpoint[2] + 338.6 * forceError[1] - 337.4 * forceError[2]; //5ms
  
  controlData->forceError[ 1 ] = controlData->forceError[ 0 ];

  return  &(controlData->outputs);
}

void EndController( Controller controller )
{
  free( (void*) controller );
}
