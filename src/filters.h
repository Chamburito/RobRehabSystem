#ifndef FILTERS_H
#define FILTERS_H

#include "interface.h"

#include "debug/async_debug.h"

#include <math.h>
#include <stdlib.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       GENERIC FILTER                                        /////
///////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                    SIMPLIFIED KALMAN FILTER                                 /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _KalmanFilterData
{
  double* state;
  double* gains;
  double** covarianceMatrix;
  double** covarianceAux;
  size_t dimensionsNumber;
}
KalmanFilterData;

typedef KalmanFilterData* KalmanFilter;


#define SIMPLE_KALMAN_FUNCTIONS( namespace, function_init ) \
        function_init( KalmanFilter, namespace, CreateFilter, size_t, double ) \
        function_init( void, namespace, DiscardFilter, KalmanFilter ) \
        function_init( double*, namespace, Update, KalmanFilter, double, double )

INIT_NAMESPACE_INTERFACE( SimpleKalman, SIMPLE_KALMAN_FUNCTIONS )


KalmanFilter SimpleKalman_CreateFilter( size_t dimensionsNumber, double initialValue )
{
  KalmanFilter newFilter = (KalmanFilter) malloc( sizeof(KalmanFilterData) );
  
  newFilter->state = (double*) calloc( dimensionsNumber, sizeof(double) );
  newFilter->gains = (double*) calloc( dimensionsNumber, sizeof(double) );
  newFilter->covarianceMatrix = (double**) calloc( dimensionsNumber, sizeof(double*) );
  newFilter->covarianceAux = (double**) calloc( dimensionsNumber, sizeof(double*) );
  for( size_t dimensionIndex = 0; dimensionIndex < dimensionsNumber; dimensionIndex++ )
  {
    newFilter->state[ dimensionIndex ] = ( dimensionIndex == 0 ) ? initialValue : 0.0;
    
    newFilter->covarianceMatrix[ dimensionIndex ] = (double*) calloc( dimensionsNumber, sizeof(double) );
    newFilter->covarianceAux[ dimensionIndex ] = (double*) calloc( dimensionsNumber, sizeof(double) );
    for( size_t i = 0; i < dimensionsNumber; i++ )
      newFilter->covarianceMatrix[ dimensionIndex ][ i ] = ( i == dimensionIndex ) ? 1.0 : 0.0;
  }
  
  newFilter->dimensionsNumber = dimensionsNumber;
  
  return newFilter;
}

void SimpleKalman_DiscardFilter( KalmanFilter filter )
{
  if( filter == NULL ) return;
    
  free( filter->state );
  free( filter->gains );
  for( size_t dimensionIndex = 0; dimensionIndex < filter->dimensionsNumber; dimensionIndex++ )
  {
    free( filter->covarianceAux[ dimensionIndex ] );
    free( filter->covarianceMatrix[ dimensionIndex ] );
  }
  free( filter->covarianceAux );
  free( filter->covarianceMatrix );
  
  free( filter );
}

double* SimpleKalman_Update( KalmanFilter filter, double newValue, double timeStamp )
{
  if( filter == NULL ) return NULL;
  
  double error, covarianceResidual;
  
  if( timeStamp > 0.0 )
  {
    error = newValue - filter->state[ 0 ];
    
    for( size_t dimensionIndex = 0; dimensionIndex < filter->dimensionsNumber; dimensionIndex++ )
    {
      for( size_t i = dimensionIndex + 1; i < filter->dimensionsNumber; i++ )
        filter->state[ dimensionIndex ] += filter->state[ i ] * pow( timeStamp, i - dimensionIndex ) / ( i - dimensionIndex );
    }
  
    for( size_t column = 0; column < filter->dimensionsNumber; column++ )
    {
      for( size_t line = 0; line < filter->dimensionsNumber; line++ )
      {
        filter->covarianceAux[ line ][ column ] = filter->covarianceMatrix[ line ][ column ];
        for( size_t i = line + 1; i < filter->dimensionsNumber; i++ )
          filter->covarianceAux[ line ][ column ] += filter->covarianceMatrix[ i ][ column ] * pow( timeStamp, i - line ) / ( i - line );
      }
    }
  
    for( size_t line = 0; line < filter->dimensionsNumber; line++ )
    {
      for( size_t column = 0; column < filter->dimensionsNumber; column++ )
      {
        filter->covarianceMatrix[ line ][ column ] = filter->covarianceAux[ line ][ column ];
        for( size_t i = column + 1; i < filter->dimensionsNumber; i++ )
          filter->covarianceMatrix[ line ][ column ] += filter->covarianceAux[ line ][ i ] * pow( timeStamp, i - column ) / ( i - column );
      }
    }
  
    for( size_t dimensionIndex = 0; dimensionIndex < filter->dimensionsNumber; dimensionIndex++ )
      filter->covarianceMatrix[ dimensionIndex ][ dimensionIndex ] += 1.0;
  
    covarianceResidual = filter->covarianceMatrix[ 0 ][ 0 ] + 1.0;
  
    if( covarianceResidual != 0.0 )
    {
      for( size_t dimensionIndex = 0; dimensionIndex < filter->dimensionsNumber; dimensionIndex++ )
      {
        filter->gains[ dimensionIndex ] = filter->covarianceMatrix[ dimensionIndex ][ 0 ] / covarianceResidual;
        filter->state[ dimensionIndex ] += error * filter->gains[ dimensionIndex ];
      }
    }
  
    for( int line = filter->dimensionsNumber - 1; line >= 0 ; line-- )
    {
      for( size_t column = 0; column < filter->dimensionsNumber; column++ )
        filter->covarianceMatrix[ line ][ column ] -= filter->gains[ line ] * filter->covarianceMatrix[ 0 ][ column ];
    }
  }
  
  return filter->state;
}

#endif  // FILTERS_H
