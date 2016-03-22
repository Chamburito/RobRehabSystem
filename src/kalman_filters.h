#ifndef KALMAN_FILTERS_H
#define KALMAN_FILTERS_H

#include "interfaces.h"

#include "matrices.h"

#include "debug/async_debug.h"

#include <math.h>
#include <stdlib.h>


typedef struct _KalmanFilterData
{
  Matrix input;                                       // y
  Matrix state;                                       // x
  Matrix error;                                       // e
  Matrix inputModel;                                  // H
  Matrix gain;                                        // K
  Matrix prediction;                                  // F
  Matrix predictionCovariance;                        // P
  Matrix predictionCovarianceNoise;                   // Q
  Matrix errorCovariance;                             // S
  Matrix errorCovarianceNoise;                        // R
}
KalmanFilterData;

typedef KalmanFilterData* KalmanFilter;


#define KALMAN_FUNCTIONS( namespace, function_init ) \
        function_init( KalmanFilter, namespace, CreateFilter, size_t ) \
        function_init( void, namespace, DiscardFilter, KalmanFilter ) \
        function_init( void, namespace, AddInput, KalmanFilter, size_t ) \
        function_init( void, namespace, SetInput, KalmanFilter, size_t, double ) \
        function_init( void, namespace, SetVariablesCoupling, KalmanFilter, size_t, size_t, double ) \
        function_init( void, namespace, SetInputMaxError, KalmanFilter, size_t, double ) \
        function_init( double*, namespace, Predict, KalmanFilter ) \
        function_init( double*, namespace, Update, KalmanFilter ) \
        function_init( void, namespace, Reset, KalmanFilter )

INIT_NAMESPACE_INTERFACE( Kalman, KALMAN_FUNCTIONS )


KalmanFilter Kalman_CreateFilter( size_t dimensionsNumber )
{
  KalmanFilter newFilter = (KalmanFilter) malloc( sizeof(KalmanFilterData) );
  memset( newFilter, 0, sizeof(KalmanFilterData) );
  
  newFilter->state = Matrices_Create( NULL, dimensionsNumber, 1 );
  newFilter->error = Matrices_Create( NULL, dimensionsNumber, 1 );
  
  newFilter->gain = Matrices_CreateSquare( dimensionsNumber, MATRIX_ZERO );
  
  newFilter->prediction = Matrices_CreateSquare( dimensionsNumber, MATRIX_IDENTITY );
  newFilter->predictionCovariance = Matrices_CreateSquare( dimensionsNumber, MATRIX_ZERO );
  newFilter->predictionCovarianceNoise = Matrices_CreateSquare( dimensionsNumber, MATRIX_IDENTITY );
  
  Kalman_Reset( newFilter );
  
  return newFilter;
}

void Kalman_DiscardFilter( KalmanFilter filter )
{
  if( filter == NULL ) return;
    
  Matrices_Discard( filter->input );
  Matrices_Discard( filter->state );
  Matrices_Discard( filter->error );
  
  Matrices_Discard( filter->gain );
  Matrices_Discard( filter->prediction );
  Matrices_Discard( filter->predictionCovariance );
  Matrices_Discard( filter->predictionCovarianceNoise );
  Matrices_Discard( filter->errorCovariance );
  Matrices_Discard( filter->errorCovarianceNoise );
  
  free( filter );
}

void Kalman_AddInput( KalmanFilter filter, size_t dimensionIndex )
{
  if( filter == NULL ) return;
  
  size_t dimensionsNumber = Matrices_GetHeight( filter->state );
  
  if( dimensionIndex >= dimensionsNumber ) return;
  
  size_t newInputIndex = Matrices_GetHeight( filter->input );
  size_t newInputsNumber = newInputIndex + 1;
  
  Matrices_Resize( filter->input, newInputsNumber, 1 );
  Matrices_Resize( filter->inputModel, newInputsNumber, dimensionsNumber );
  
  Matrices_SetElement( filter->inputModel, newInputIndex, dimensionIndex, 1 );
  
  Matrices_Resize( filter->errorCovariance, newInputsNumber, newInputsNumber );
  Matrices_Resize( filter->errorCovarianceNoise, newInputsNumber, newInputsNumber );
  
  Matrices_SetElement( filter->errorCovarianceNoise, newInputIndex, newInputIndex, 1.0 );
}

void Kalman_SetInput( KalmanFilter filter, size_t inputIndex, double value )
{
  if( filter == NULL ) return;
  
  Matrices_SetElement( filter->input, inputIndex, 0, value );
}

void Kalman_SetVariablesCoupling( KalmanFilter filter, size_t inputIndex, size_t outputIndex, double ratio )
{
  if( filter == NULL ) return;
  
  Matrices_SetElement( filter->prediction, inputIndex, outputIndex, ratio );
}

void Kalman_SetInputMaxError( KalmanFilter filter, size_t inputIndex, double maxError )
{
  if( filter == NULL ) return;
  
  Matrices_SetElement( filter->errorCovarianceNoise, inputIndex, inputIndex, maxError * maxError );
}

double* Kalman_Predict( KalmanFilter filter )
{
  if( filter == NULL ) return NULL;
  
  // x = F*x_old
  Matrices_Dot( filter->prediction, 1.0, filter->state, 1.0, filter->state );                                                       // F[nxn] * x_old[nx1] -> x[nx1]
  
  // P = F*P_old*F' + Q
  Matrices_Dot( filter->prediction, MATRIX_KEEP, filter->predictionCovariance, MATRIX_KEEP, filter->predictionCovariance );         // F[nxn] * P_old[nxn] -> P[nxn]
  Matrices_Dot( filter->predictionCovariance, MATRIX_KEEP, filter->prediction, MATRIX_TRANSPOSE, filter->predictionCovariance );    // P[nxn] * F'[nxn] -> P[nxn]
  Matrices_Sum( filter->predictionCovariance, 1.0, filter->predictionCovarianceNoise, 1.0, filter->predictionCovariance );          // P[nxn] + Q[nxn] -> P[nxn]
  
  //DEBUG_PRINT( "x = [%.3f;%.3f], P = [%.3f %.3f;%.3f %.3f]", Matrices_GetElement( filter->state, 0, 0 ), Matrices_GetElement( filter->state, 1, 0 ),
  //                                                           Matrices_GetElement( filter->predictionCovariance, 0, 0 ), Matrices_GetElement( filter->predictionCovariance, 0, 1 ),
  //                                                           Matrices_GetElement( filter->predictionCovariance, 1, 0 ), Matrices_GetElement( filter->predictionCovariance, 1, 1 ) );
  
  return Matrices_GetAsVector( filter->state );
}

double* Kalman_Update( KalmanFilter filter )
{
  if( filter == NULL ) return NULL;
  
  // e = y - H*x
  Matrices_Dot( filter->inputModel, MATRIX_KEEP, filter->state, MATRIX_KEEP, filter->error );                             // H[mxn] * x[nx1] -> e[mx1]
  Matrices_Sum( filter->input, 1.0, filter->error, -1.0, filter->error );                                                 // y[mx1] - e[mx1] -> e[mx1]
  
  // K = P*H' * ( H*P*H' + R )^(-1)
  Matrices_Dot( filter->predictionCovariance, MATRIX_KEEP, filter->inputModel, MATRIX_TRANSPOSE, filter->gain );          // P[nxn] * H'[nxm] -> K[nxm]
  Matrices_Dot( filter->inputModel, MATRIX_KEEP, filter->predictionCovariance, MATRIX_KEEP, filter->errorCovariance );    // H[mxn] * P[nxn] -> S[mxn]
  Matrices_Dot( filter->errorCovariance, MATRIX_KEEP, filter->inputModel, MATRIX_TRANSPOSE, filter->errorCovariance );    // S[mxn] * H'[nxm] -> S[mxm]
  Matrices_Sum( filter->errorCovariance, 1.0, filter->errorCovarianceNoise, 1.0, filter->errorCovariance );               // S[mxm] + R[mxm] -> S[mxm]
  Matrices_Inverse( filter->errorCovariance, filter->errorCovariance );                                                   // S^(-1)[mxm] -> S[mxm]
  Matrices_Dot( filter->gain, MATRIX_KEEP, filter->errorCovariance, MATRIX_KEEP, filter->gain );                          // K[nxm] * S[mxm] -> K[nxm]
  
  // x' = x + K*e
  Matrices_Dot( filter->gain, MATRIX_KEEP, filter->error, MATRIX_KEEP, filter->error );                                   // K[nxm] * e[mx1] -> e[nx1]
  Matrices_Sum( filter->state, 1.0, filter->error, 1.0, filter->state );                                                  // x[nx1] + e[nx1] -> x[nx1]
  
  // P' = P - K*H*P
  Matrices_Dot( filter->gain, MATRIX_KEEP, filter->inputModel, MATRIX_KEEP, filter->gain );                               // K[nxm] * H[mxn] -> K[nxn]
  Matrices_Dot( filter->gain, MATRIX_KEEP, filter->predictionCovariance, MATRIX_KEEP, filter->gain );                     // K[nxn] * P[nxn] -> K[nxn]
  Matrices_Sum( filter->predictionCovariance, 1.0, filter->gain, -1.0, filter->predictionCovariance );                    // P[nxn] - K[nxn] -> P[nxn]
  
  //DEBUG_PRINT( "x = [%.3f;%.3f], P = [%.3f %.3f;%.3f %.3f]", Matrices_GetElement( filter->state, 0, 0 ), Matrices_GetElement( filter->state, 1, 0 ),
  //                                                           Matrices_GetElement( filter->predictionCovariance, 0, 0 ), Matrices_GetElement( filter->predictionCovariance, 0, 1 ),
  //                                                           Matrices_GetElement( filter->predictionCovariance, 1, 0 ), Matrices_GetElement( filter->predictionCovariance, 1, 1 ) );
  
  return Matrices_GetAsVector( filter->state );
}

void Kalman_Reset( KalmanFilter filter )
{
  if( filter == NULL ) return;
  
  Matrices_Clear( filter->input );
  Matrices_Clear( filter->state );
  Matrices_Clear( filter->error );
  
  Matrices_Clear( filter->gain );
  Matrices_Clear( filter->predictionCovariance );
  Matrices_Clear( filter->errorCovariance );
}

#endif  // KALMAN_FILTERS_H
