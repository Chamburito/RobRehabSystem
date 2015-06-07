#ifndef TRAJECTORY_PLANNING_H
#define TRAJECTORY_PLANNING_H 

#include "async_debug.h"

const size_t TRAJECTORY_CURVE_ORDER = 3;

enum { TRAJECTORY_POSITION, TRAJECTORY_VELOCITY, TRAJECTORY_ACCELERATION, TRAJECTORY_TIME, TRAJECTORY_VALUES_NUMBER };

typedef struct _TrajectoryPlanner
{
  double referenceCurve[ TRAJECTORY_CURVE_ORDER + 1 ];
  double initialTime, lastPointTime, curveTimeLength;
  double targetList[ TRAJECTORY_VALUES_NUMBER ];
} 
TrajectoryPlanner;

TrajectoryPlanner* TrajectoryPlanner_Init()
{
  TrajectoryPlanner* planner = (TrajectoryPlanner*) malloc( sizeof(TrajectoryPlanner) );
  
  for( size_t i = 0; i <= TRAJECTORY_CURVE_ORDER; i++ )
    planner->referenceCurve[ i ] = 0.0;
  for( size_t i = 0; i < TRAJECTORY_VALUES_NUMBER; i++ )
    planner->targetList[ i ] = 0.0;
  
  planner->lastPointTime = planner->initialTime = ( (double) Timing_GetExecTimeMilliseconds() ) / 1000.0;
  
  return planner;
}

void TrajectoryPlanner_End( TrajectoryPlanner* planner )
{
  free( planner );
}

double* TrajectoryPlanner_GetTargetList( TrajectoryPlanner* planner )
{
  double elapsedTime = ( (double) Timing_GetExecTimeMilliseconds() ) / 1000.0 - planner->initialTime;
  
  /*if( elapsedTime > planner->curveTimeLength )
  {
    planner->referenceCurve[ 2 ] = - 0.1 * planner->referenceCurve[ 1 ] / fabs( planner->referenceCurve[ 1 ] );
    planner->referenceCurve[ 3 ] = 0.0;
  }*/
  
  planner->targetList[ TRAJECTORY_POSITION ] = planner->referenceCurve[ 0 ];
  for( size_t i = 1; i <= TRAJECTORY_CURVE_ORDER; i++ )
    planner->targetList[ TRAJECTORY_POSITION ] += planner->referenceCurve[ i ] * pow( elapsedTime, i );
  
  planner->targetList[ TRAJECTORY_VELOCITY ] = planner->referenceCurve[ 1 ];
  for( size_t i = 2; i <= TRAJECTORY_CURVE_ORDER; i++ )
    planner->targetList[ TRAJECTORY_VELOCITY ] += i * planner->referenceCurve[ i ] * pow( elapsedTime, i - 1 );
  
  planner->targetList[ TRAJECTORY_ACCELERATION ] = 2 * planner->referenceCurve[ 2 ] + 6 * planner->referenceCurve[ 3 ] * elapsedTime;
  
  planner->targetList[ TRAJECTORY_TIME ] = elapsedTime;
  
  return (double*) planner->targetList;
}

void TrajectoryPlanner_SetCurve( TrajectoryPlanner* planner, double setpointsList[ TRAJECTORY_VALUES_NUMBER ] )
{
  (void) TrajectoryPlanner_GetTargetList( planner );
  
  planner->initialTime = ( (double) Timing_GetExecTimeMilliseconds() ) / 1000.0;
  
  planner->referenceCurve[ 0 ] = planner->targetList[ TRAJECTORY_POSITION ];
  planner->referenceCurve[ 1 ] = ( setpointsList[ TRAJECTORY_POSITION ] - planner->referenceCurve[ 0 ] ) / setpointsList[ TRAJECTORY_TIME ];
  planner->referenceCurve[ 2 ] = ( setpointsList[ TRAJECTORY_VELOCITY ] - planner->referenceCurve[ 1 ] ) / ( 2 * setpointsList[ TRAJECTORY_TIME ] );
  
  /*planner->referenceCurve[ 2 ] = 3 * ( setpointsList[ TRAJECTORY_POSITION ] - planner->referenceCurve[ 0 ] ) / pow( setpointsList[ TRAJECTORY_TIME ], 2 )
                                 - ( setpointsList[ TRAJECTORY_VELOCITY ] - 2 * planner->referenceCurve[ 1 ] ) / setpointsList[ TRAJECTORY_TIME ];

  planner->referenceCurve[ 3 ] = ( setpointsList[ TRAJECTORY_VELOCITY ] - planner->referenceCurve[ 1 ] ) / pow( setpointsList[ TRAJECTORY_TIME ], 2 )
                                 - 2 * ( setpointsList[ TRAJECTORY_VELOCITY ] - 2 * planner->referenceCurve[ 1 ] ) / pow( setpointsList[ TRAJECTORY_TIME ], 3 );*/
    
  planner->curveTimeLength = setpointsList[ TRAJECTORY_TIME ];
  
  DEBUG_PRINT( "setpoints: %.3f %.3f %.3f - interval: %.3f", setpointsList[ TRAJECTORY_POSITION ], setpointsList[ TRAJECTORY_VELOCITY ], setpointsList[ TRAJECTORY_ACCELERATION ], setpointsList[ TRAJECTORY_TIME ] );
  
  DEBUG_PRINT( "poly: %.3ft^3 %+.3ft^2 %+.3ft %+.3f", planner->referenceCurve[ 3 ], planner->referenceCurve[ 2 ], planner->referenceCurve[ 1 ], planner->referenceCurve[ 0 ] );
}

#endif // TRAJECTORY_PLANNING_H 
