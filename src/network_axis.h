#ifndef TRAJECTORY_PLANNING_H
#define TRAJECTORY_PLANNING_H 

#include "async_debug.h"

const size_t TRAJECTORY_CURVE_ORDER = 3;

enum { TRAJECTORY_POSITION, TRAJECTORY_VELOCITY, TRAJECTORY_ACCELERATION, TRAJECTORY_TIME, TRAJECTORY_VALUES_NUMBER };

typedef struct _TrajectoryPlanner
{
  double referenceCurve[ TRAJECTORY_CURVE_ORDER + 1 ];
  double lastCurveTime, lastPointTime, deltaCurveTime;
  double lagFactor;
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
  
  planner->deltaCurveTime = 1.0;
  planner->lastPointTime = planner->lastCurveTime = ( (double) Timing_GetExecTimeMilliseconds() ) / 1000.0;
  planner->lagFactor = 1.0;
  
  return planner;
}

void TrajectoryPlanner_End( TrajectoryPlanner* planner )
{
  free( planner );
}

void TrajectoryPlanner_SetCurve( TrajectoryPlanner* planner, double setpointsList[ TRAJECTORY_VALUES_NUMBER ] )
{
  double absoluteTime = ( (double) Timing_GetExecTimeMilliseconds() ) / 1000.0;
  
  if( planner->targetList[ TRAJECTORY_POSITION ] > 0.0 ) 
  {
    double deltaTime = absoluteTime - planner->lastPointTime;
    
    planner->targetList[ TRAJECTORY_TIME ] += deltaTime;
  }
  
  double deltaCurveTime = setpointsList[ TRAJECTORY_TIME ] - planner->targetList[ TRAJECTORY_TIME ];

  if( deltaCurveTime > 0.0 )
  {
    if( planner->deltaCurveTime == 1.0 ) planner->deltaCurveTime = deltaCurveTime;

    planner->lagFactor = 2.0 * ( deltaCurveTime - planner->deltaCurveTime ) / planner->deltaCurveTime;
    if( planner->lagFactor < 0.0 ) planner->lagFactor = 0.0;

    planner->lastPointTime = planner->lastCurveTime = absoluteTime;

    planner->referenceCurve[ 0 ] = planner->targetList[ TRAJECTORY_POSITION ];
    planner->referenceCurve[ 1 ] = planner->targetList[ TRAJECTORY_VELOCITY ];
    planner->referenceCurve[ 2 ] = planner->targetList[ TRAJECTORY_ACCELERATION ] / 2;

    planner->referenceCurve[ 3 ] = ( setpointsList[ TRAJECTORY_POSITION ] - planner->referenceCurve[ 0 ]
                                     - planner->referenceCurve[ 1 ] * deltaCurveTime
                                     - planner->referenceCurve[ 2 ] * deltaCurveTime * deltaCurveTime )
                                   / ( deltaCurveTime * deltaCurveTime * deltaCurveTime );
  }

  if( setpointsList[ TRAJECTORY_TIME ] < planner->targetList[ TRAJECTORY_TIME ] )
  {
    planner->targetList[ TRAJECTORY_TIME ] = 0.0;
    planner->deltaCurveTime = 1.0;
    planner->lagFactor = 1.0;
  }
}

double* TrajectoryPlanner_GetTargetList( TrajectoryPlanner* planner )
{
  double absoluteTime = ( (double) Timing_GetExecTimeMilliseconds() ) / 1000.0;
  
  double deltaTime = ( absoluteTime - planner->lastPointTime ) * planner->lagFactor;
  planner->lastPointTime = absoluteTime;
  
  double elapsedTime = absoluteTime - planner->lastCurveTime;
  
  /*if( elapsedTime > planner->deltaCurveTime )
  {
    planner->referenceCurve[ 1 ] *= 0.9;
    planner->referenceCurve[ 3 ] = planner->referenceCurve[ 2 ] = 0.0;
  }*/
  
  double estimatedTime = elapsedTime + deltaTime;
  
  planner->targetList[ TRAJECTORY_TIME ] += deltaTime;
  
  planner->targetList[ TRAJECTORY_POSITION ] = planner->referenceCurve[ 0 ];
  for( size_t i = 1; i <= TRAJECTORY_CURVE_ORDER; i++ )
    planner->targetList[ TRAJECTORY_POSITION ] += planner->referenceCurve[ i ] * pow( estimatedTime, i );
  
  planner->targetList[ TRAJECTORY_VELOCITY ] = planner->referenceCurve[ 1 ];
  for( size_t i = 2; i <= TRAJECTORY_CURVE_ORDER; i++ )
    planner->targetList[ TRAJECTORY_VELOCITY ] += i * planner->referenceCurve[ i ] * pow( estimatedTime, i - 1 );
  
  planner->targetList[ TRAJECTORY_ACCELERATION ] = 2 * planner->referenceCurve[ 2 ] + 6 * planner->referenceCurve[ 3 ] * estimatedTime;
  
  return (double*) planner->targetList;
}

#endif // TRAJECTORY_PLANNING_H 
