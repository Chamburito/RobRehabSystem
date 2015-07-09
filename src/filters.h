#ifndef FILTERS_H
#define FILTERS_H

#include <math.h>

#include "async_debug.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       GENERIC FILTER  									                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                    SIMPLIFIED KALMAN FILTER	      			                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

enum { KALMAN_VALUE, KALMAN_DERIVATIVE, KALMAN_DERIVATIVE_2, KALMAN_MEASURES_NUMBER };

typedef struct _SimpleKalmanFilter
{
  double state[ KALMAN_MEASURES_NUMBER ];
  double covarianceMatrix[ KALMAN_MEASURES_NUMBER ][ KALMAN_MEASURES_NUMBER ];
}
SimpleKalmanFilter;

SimpleKalmanFilter* SimpleKalman_CreateFilter( double initialValue )
{
  SimpleKalmanFilter* newFilter = (SimpleKalmanFilter*) malloc( sizeof(SimpleKalmanFilter) );
  
  for( size_t dimensionIndex = 0; dimensionIndex < KALMAN_MEASURES_NUMBER; dimensionIndex++ )
  {
    newFilter->state[ dimensionIndex ] = ( dimensionIndex == KALMAN_VALUE ) ? initialValue : 0.0;
    
    for( size_t i = 0; i < KALMAN_MEASURES_NUMBER; i++ )
      newFilter->covarianceMatrix[ dimensionIndex ][ i ] = ( i == dimensionIndex ) ? 1.0 : 0.0;
  }
  
  return newFilter;
}

void SimpleKalman_DiscardFilter( SimpleKalmanFilter* filter )
{
  if( filter != NULL ) free( filter );
}

double* SimpleKalman_Update( SimpleKalmanFilter* filter, double newValue, double timeStamp )
{
  static double error, covarianceResidual;
  static double gains[ KALMAN_MEASURES_NUMBER ];
  
  static double covarianceAux[ KALMAN_MEASURES_NUMBER ][ KALMAN_MEASURES_NUMBER ]; 
  
  if( timeStamp > 0.0 )
  {
    error = newValue - filter->state[ KALMAN_VALUE ];
    
    for( size_t dimensionIndex = 0; dimensionIndex < KALMAN_MEASURES_NUMBER; dimensionIndex++ )
    {
      for( size_t i = dimensionIndex + 1; i < KALMAN_MEASURES_NUMBER; i++ )
        filter->state[ dimensionIndex ] += filter->state[ i ] * pow( timeStamp, i - dimensionIndex ) / ( i - dimensionIndex );
    }
  
    for( size_t column = 0; column < KALMAN_MEASURES_NUMBER; column++ )
    {
      covarianceAux[ 0 ][ column ] = filter->covarianceMatrix[ 0 ][ column ] + filter->covarianceMatrix[ 1 ][ column ] * timeStamp + filter->covarianceMatrix[ 2 ][ column ] * timeStamp * timeStamp / 2;
      covarianceAux[ 1 ][ column ] = filter->covarianceMatrix[ 1 ][ column ] + filter->covarianceMatrix[ 2 ][ column ] * timeStamp;
      covarianceAux[ 2 ][ column ] = filter->covarianceMatrix[ 2 ][ column ];
      /*for( size_t line = 0; line < KALMAN_MEASURES_NUMBER; line++ )
      {
        covarianceAux[ line ][ column ] = filter->covarianceMatrix[ line ][ column ];
        for( size_t i = line + 1; i < KALMAN_MEASURES_NUMBER; i++ )
          covarianceAux[ line ][ column ] += filter->covarianceMatrix[ i ][ column ] * pow( timeStamp, i - line ) / ( i - line );
      }*/
    }
  
    for( size_t line = 0; line < KALMAN_MEASURES_NUMBER; line++ )
    {
      filter->covarianceMatrix[ line ][ 0 ] = covarianceAux[ line ][ 0 ] + covarianceAux[ line ][ 1 ] * timeStamp + covarianceAux[ line ][ 2 ] * timeStamp * timeStamp / 2;
      filter->covarianceMatrix[ line ][ 1 ] = covarianceAux[ line ][ 1 ] + covarianceAux[ line ][ 2 ] * timeStamp;
      filter->covarianceMatrix[ line ][ 2 ] = covarianceAux[ line ][ 2 ];
      /*for( size_t column = 0; column < KALMAN_MEASURES_NUMBER; column++ )
      {
        filter->covarianceMatrix[ line ][ column ] = covarianceAux[ line ][ column ];
        for( size_t i = column + 1; i < KALMAN_MEASURES_NUMBER; i++ )
          filter->covarianceMatrix[ line ][ column ] += covarianceAux[ line ][ i ] * pow( timeStamp, i - column ) / ( i - column );
      }*/
    }
  
    for( size_t dimensionIndex = 0; dimensionIndex < KALMAN_MEASURES_NUMBER; dimensionIndex++ )
      filter->covarianceMatrix[ dimensionIndex ][ dimensionIndex ] += 1.0;
  
    covarianceResidual = filter->covarianceMatrix[ 0 ][ 0 ] + 1.0;
  
    if( covarianceResidual != 0.0 )
    {
      for( size_t dimensionIndex = 0; dimensionIndex < KALMAN_MEASURES_NUMBER; dimensionIndex++ )
      {
        gains[ dimensionIndex ] = filter->covarianceMatrix[ dimensionIndex ][ 0 ] / covarianceResidual;
        filter->state[ dimensionIndex ] += error * gains[ dimensionIndex ];
      }
    }
  
    for( int line = KALMAN_MEASURES_NUMBER - 1; line >= 0 ; line-- )
    {
      for( size_t column = 0; column < KALMAN_MEASURES_NUMBER; column++ )
        filter->covarianceMatrix[ line ][ column ] -= gains[ line ] * filter->covarianceMatrix[ 0 ][ column ];
    }
  }
  
  return (double*) filter->state;
}

#endif  // FILTERS_H
