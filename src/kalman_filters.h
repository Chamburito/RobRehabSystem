#ifndef KALMAN_FILTERS_H
#define KALMAN_FILTERS_H

#include "interfaces.h"

#include "matrices.h"

#include "debug/async_debug.h"

#include <math.h>
#include <stdlib.h>


typedef struct _KalmanFilterData
{
  Matrix measures;                                    // z
  Matrix state;                                       // x
  Matrix innovation;                                  // y
  Matrix gains;                                       // K
  Matrix prediction;                                  // F
  Matrix predictionCovariance;                        // P
  Matrix predictionCovarianceNoise;                   // Q
  Matrix innovationCovariance;                        // S
  Matrix innovationCovarianceNoise;                   // R
}
KalmanFilterData;

typedef KalmanFilterData* KalmanFilter;


#define KALMAN_FUNCTIONS( namespace, function_init ) \
        function_init( KalmanFilter, namespace, CreateFilter, size_t ) \
        function_init( void, namespace, DiscardFilter, KalmanFilter ) \
        function_init( void, namespace, SetMeasure, KalmanFilter, size_t, double ) \
        function_init( void, namespace, SetVariablesCoupling, KalmanFilter, size_t, size_t, double ) \
        function_init( void, namespace, SetMeasureMaxError, KalmanFilter, size_t, double ) \
        function_init( double*, namespace, Predict, KalmanFilter ) \
        function_init( double*, namespace, Update, KalmanFilter ) \
        function_init( void, namespace, Reset, KalmanFilter )

INIT_NAMESPACE_INTERFACE( Kalman, KALMAN_FUNCTIONS )


KalmanFilter Kalman_CreateFilter( size_t dimensionsNumber )
{
  KalmanFilter newFilter = (KalmanFilter) malloc( sizeof(KalmanFilterData) );
  
  newFilter->measures = Matrices_Create( NULL, dimensionsNumber, 1 );
  newFilter->state = Matrices_Create( NULL, dimensionsNumber, 1 );
  newFilter->innovation = Matrices_Create( NULL, dimensionsNumber, 1 );
  
  newFilter->gains = Matrices_CreateSquare( dimensionsNumber, MATRIX_ZERO );
  newFilter->prediction = Matrices_CreateSquare( dimensionsNumber, MATRIX_IDENTITY );
  newFilter->predictionCovariance = Matrices_CreateSquare( dimensionsNumber, MATRIX_ZERO );
  newFilter->predictionCovarianceNoise = Matrices_CreateSquare( dimensionsNumber, MATRIX_IDENTITY );
  newFilter->innovationCovariance = Matrices_CreateSquare( dimensionsNumber, MATRIX_ZERO );
  newFilter->innovationCovarianceNoise = Matrices_CreateSquare( dimensionsNumber, MATRIX_IDENTITY );
  
  Kalman_Reset( newFilter );
  
  return newFilter;
}

void Kalman_DiscardFilter( KalmanFilter filter )
{
  if( filter == NULL ) return;
    
  Matrices_Discard( filter->measures );
  Matrices_Discard( filter->state );
  Matrices_Discard( filter->innovation );
  
  Matrices_Discard( filter->gains );
  Matrices_Discard( filter->prediction );
  Matrices_Discard( filter->predictionCovariance );
  Matrices_Discard( filter->predictionCovarianceNoise );
  Matrices_Discard( filter->innovationCovariance );
  Matrices_Discard( filter->innovationCovarianceNoise );
  
  free( filter );
}

void Kalman_SetMeasure( KalmanFilter filter, size_t measureIndex, double value )
{
  if( filter == NULL ) return;
  
  Matrices_SetElement( filter->measures, measureIndex, 0, value );
}

void Kalman_SetVariablesCoupling( KalmanFilter filter, size_t inputIndex, size_t outputIndex, double ratio )
{
  if( filter == NULL ) return;
  
  Matrices_SetElement( filter->prediction, inputIndex, outputIndex, ratio );
}

void Kalman_SetMeasureMaxError( KalmanFilter filter, size_t measureIndex, double maxError )
{
  if( filter == NULL ) return;
  
  Matrices_SetElement( filter->innovationCovarianceNoise, measureIndex, measureIndex, maxError * maxError );
}

double* Kalman_Predict( KalmanFilter filter )
{
  if( filter == NULL ) return NULL;
  
  // x = F*x_old
  Matrices_Dot( filter->prediction, 1.0, filter->state, 1.0, filter->state );
  
  // P = F*P_old*F' + Q
  Matrices_Dot( filter->prediction, MATRIX_KEEP, filter->predictionCovariance, MATRIX_KEEP, filter->predictionCovariance );
  Matrices_Dot( filter->predictionCovariance, MATRIX_KEEP, filter->prediction, MATRIX_TRANSPOSE, filter->predictionCovariance );
  Matrices_Sum( filter->predictionCovariance, 1.0, filter->predictionCovarianceNoise, 1.0, filter->predictionCovariance );
  
  //DEBUG_PRINT( "x = [%.3f;%.3f], P = [%.3f %.3f;%.3f %.3f]", Matrices_GetElement( filter->state, 0, 0 ), Matrices_GetElement( filter->state, 1, 0 ),
  //                                                           Matrices_GetElement( filter->predictionCovariance, 0, 0 ), Matrices_GetElement( filter->predictionCovariance, 0, 1 ),
  //                                                           Matrices_GetElement( filter->predictionCovariance, 1, 0 ), Matrices_GetElement( filter->predictionCovariance, 1, 1 ) );
  
  return Matrices_GetAsVector( filter->state );
}

double* Kalman_Update( KalmanFilter filter )
{
  if( filter == NULL ) return NULL;
  
  // y = z - [H*]x
  Matrices_Sum( filter->measures, 1.0, filter->state, -1.0, filter->innovation );
  
  // K = P[*H'] * ( [H*]P[*H'] + R )^(-1)
  Matrices_Sum( filter->predictionCovariance, 1.0, filter->innovationCovarianceNoise, 1.0, filter->innovationCovariance );
  Matrices_Inverse( filter->innovationCovariance, filter->innovationCovariance );
  Matrices_Dot( filter->predictionCovariance, MATRIX_KEEP, filter->innovationCovariance, MATRIX_KEEP, filter->gains );
  
  // x' = x + K*y
  Matrices_Dot( filter->gains, MATRIX_KEEP, filter->innovation, MATRIX_KEEP, filter->innovation );
  Matrices_Sum( filter->state, 1.0, filter->innovation, 1.0, filter->state );
  
  // P' = P - K*P
  Matrices_Dot( filter->gains, MATRIX_KEEP, filter->predictionCovariance, MATRIX_KEEP, filter->gains );
  Matrices_Sum( filter->predictionCovariance, 1.0, filter->gains, -1.0, filter->predictionCovariance );
  
  //DEBUG_PRINT( "x = [%.3f;%.3f], P = [%.3f %.3f;%.3f %.3f]", Matrices_GetElement( filter->state, 0, 0 ), Matrices_GetElement( filter->state, 1, 0 ),
  //                                                           Matrices_GetElement( filter->predictionCovariance, 0, 0 ), Matrices_GetElement( filter->predictionCovariance, 0, 1 ),
  //                                                           Matrices_GetElement( filter->predictionCovariance, 1, 0 ), Matrices_GetElement( filter->predictionCovariance, 1, 1 ) );
  
  return Matrices_GetAsVector( filter->state );
}

void Kalman_Reset( KalmanFilter filter )
{
  if( filter == NULL ) return;
  
  Matrices_Clear( filter->measures );
  Matrices_Clear( filter->state );
  Matrices_Clear( filter->innovation );
  
  Matrices_Clear( filter->gains );
  Matrices_Clear( filter->predictionCovariance );
  Matrices_Clear( filter->innovationCovariance );
}

#endif  // KALMAN_FILTERS_H
