#ifndef TRAJECTORY_PLANNING_H
#define TRAJECTORY_PLANNING_H 

#include <math.h>

#include "debug/async_debug.h"

//const size_t TRAJECTORY_CURVE_ORDER = 3;
#define TRAJECTORY_CURVE_ORDER 3

enum { TRAJECTORY_POSITION, TRAJECTORY_VELOCITY, TRAJECTORY_ACCELERATION, TRAJECTORY_TIME, TRAJECTORY_VALUES_NUMBER };

typedef struct _TrajectoryPlanner
{
  double referenceCurve[ TRAJECTORY_CURVE_ORDER + 1 ];
  double initialTime, lastPointTime, curveTimeLength;
  double targetList[ TRAJECTORY_VALUES_NUMBER ];
  double maxLimit, minLimit;
} 
TrajectoryPlanner;

TrajectoryPlanner* TrajectoryPlanner_Init()
{
  TrajectoryPlanner* planner = (TrajectoryPlanner*) malloc( sizeof(TrajectoryPlanner) );
  
  for( size_t i = 0; i <= TRAJECTORY_CURVE_ORDER; i++ )
    planner->referenceCurve[ i ] = 0.0;
  for( size_t i = 0; i < TRAJECTORY_VALUES_NUMBER; i++ )
    planner->targetList[ i ] = 0.0;
  
  planner->lastPointTime = planner->initialTime = Timing_GetExecTimeSeconds();
  planner->maxLimit = planner->minLimit = 0.0;
  
  return planner;
}

void TrajectoryPlanner_End( TrajectoryPlanner* planner )
{
  free( planner );
}

double* TrajectoryPlanner_GetTargetList( TrajectoryPlanner* planner )
{
  double elapsedTime = Timing_GetExecTimeSeconds() - planner->initialTime;
  
  planner->targetList[ TRAJECTORY_POSITION ] = planner->referenceCurve[ 0 ];
  for( size_t i = 1; i <= TRAJECTORY_CURVE_ORDER; i++ )
    planner->targetList[ TRAJECTORY_POSITION ] += planner->referenceCurve[ i ] * pow( elapsedTime, i );

  planner->targetList[ TRAJECTORY_VELOCITY ] = planner->referenceCurve[ 1 ];
  for( size_t i = 2; i <= TRAJECTORY_CURVE_ORDER; i++ )
    planner->targetList[ TRAJECTORY_VELOCITY ] += i * planner->referenceCurve[ i ] * pow( elapsedTime, i - 1 );

  planner->targetList[ TRAJECTORY_ACCELERATION ] = 2 * planner->referenceCurve[ 2 ];
  for( size_t i = 3; i <= TRAJECTORY_CURVE_ORDER; i++ )
    planner->targetList[ TRAJECTORY_ACCELERATION ] += i * ( i - 1 ) * planner->referenceCurve[ i ] * elapsedTime;

  if( planner->targetList[ TRAJECTORY_POSITION ] > planner->maxLimit )
    planner->targetList[ TRAJECTORY_POSITION ] = planner->maxLimit;
  else if( planner->targetList[ TRAJECTORY_POSITION ] < planner->minLimit ) 
    planner->targetList[ TRAJECTORY_POSITION ] = planner->minLimit;
  
  return (double*) planner->targetList;
}

void TrajectoryPlanner_SetCurve( TrajectoryPlanner* planner, double setpoint, double setpointDerivative, double curveTimeLength )
{
  const double DEVIATION_LIMIT = 1.1;
  
  (void) TrajectoryPlanner_GetTargetList( planner );
  
  planner->initialTime = Timing_GetExecTimeSeconds();
  
  planner->referenceCurve[ 0 ] = planner->targetList[ TRAJECTORY_POSITION ];
  
  planner->referenceCurve[ 1 ] = ( planner->targetList[ TRAJECTORY_VELOCITY ] + ( setpoint - planner->referenceCurve[ 0 ] ) / curveTimeLength ) / 2;

  planner->referenceCurve[ 2 ] = ( setpointDerivative - planner->referenceCurve[ 1 ] ) / ( 2 * curveTimeLength );
  
  /*planner->referenceCurve[ 2 ] = 3 * ( setpoint - planner->referenceCurve[ 0 ] ) / pow( curveTimeLength, 2 )
                                 - ( setpointDerivative + 2 * planner->referenceCurve[ 1 ] ) / curveTimeLength;
  
  planner->referenceCurve[ 3 ] = ( setpoint - planner->referenceCurve[ 0 ] ) / pow( curveTimeLength, 3 )
                                 - ( setpointDerivative + planner->referenceCurve[ 1 ] ) / ( 2 * pow( curveTimeLength, 2 ) );*/
  
  planner->curveTimeLength = curveTimeLength;
  
  if( setpoint * DEVIATION_LIMIT > planner->maxLimit ) planner->maxLimit = setpoint * DEVIATION_LIMIT;
  else if( setpoint * DEVIATION_LIMIT < planner->minLimit ) planner->minLimit = setpoint * DEVIATION_LIMIT;
  
  //DEBUG_PRINT( "setpoints: %f %f - interval: %.3f", setpoint, setpointDerivative, curveTimeLength );
  
  //DEBUG_PRINT( "poly: %.3ft^3 %+.3ft^2 %+.3ft %+.3f", planner->referenceCurve[ 3 ], planner->referenceCurve[ 2 ], planner->referenceCurve[ 1 ], planner->referenceCurve[ 0 ] );
}

#endif // TRAJECTORY_PLANNING_H 
