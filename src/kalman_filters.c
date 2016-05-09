#include <string.h>
#include <stdlib.h>

#include "matrices.h"

#include "kalman_filters.h"


struct _KalmanFilterData
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
};

DEFINE_NAMESPACE_INTERFACE( Kalman, KALMAN_INTERFACE )


KalmanFilter Kalman_CreateFilter( size_t dimensionsNumber )
{
  KalmanFilter newFilter = (KalmanFilter) malloc( sizeof(KalmanFilterData) );
  memset( newFilter, 0, sizeof(KalmanFilterData) );
  
  newFilter->state = Matrices.Create( NULL, dimensionsNumber, 1 );
  newFilter->error = Matrices.Create( NULL, dimensionsNumber, 1 );
  
  newFilter->gain = Matrices.CreateSquare( dimensionsNumber, MATRIX_ZERO );
  
  newFilter->prediction = Matrices.CreateSquare( dimensionsNumber, MATRIX_IDENTITY );
  newFilter->predictionCovariance = Matrices.CreateSquare( dimensionsNumber, MATRIX_ZERO );
  newFilter->predictionCovarianceNoise = Matrices.CreateSquare( dimensionsNumber, MATRIX_IDENTITY );
  
  Kalman_Reset( newFilter );
  
  return newFilter;
}

void Kalman_DiscardFilter( KalmanFilter filter )
{
  if( filter == NULL ) return;
    
  Matrices.Discard( filter->input );
  Matrices.Discard( filter->state );
  Matrices.Discard( filter->error );
  
  Matrices.Discard( filter->inputModel );
  Matrices.Discard( filter->gain );
  Matrices.Discard( filter->prediction );
  Matrices.Discard( filter->predictionCovariance );
  Matrices.Discard( filter->predictionCovarianceNoise );
  filter->errorCovariance = Matrices.Resize( filter->errorCovariance, 0, 0 );
  // Matrices.Discard( filter->errorCovariance ); // This causes a weird crash ("free(): invalid next size (fast)")
  Matrices.Discard( filter->errorCovarianceNoise );
  
  free( filter );
}

void Kalman_AddInput( KalmanFilter filter, size_t dimensionIndex )
{
  if( filter == NULL ) return;
  
  size_t dimensionsNumber = Matrices.GetHeight( filter->state );
  
  if( dimensionIndex >= dimensionsNumber ) return;
  
  size_t newInputIndex = Matrices.GetHeight( filter->input );
  size_t newInputsNumber = newInputIndex + 1;
  
  filter->input = Matrices.Resize( filter->input, newInputsNumber, 1 );
  
  filter->inputModel = Matrices.Resize( filter->inputModel, newInputsNumber, dimensionsNumber );
  for( size_t stateIndex = 0; stateIndex < dimensionsNumber; stateIndex++ )
    Matrices.SetElement( filter->inputModel, newInputIndex, stateIndex, 0.0 );
  Matrices.SetElement( filter->inputModel, newInputIndex, dimensionIndex, 1.0 );
  
  Matrices.Discard( filter->errorCovariance );
  filter->errorCovariance = Matrices.CreateSquare( newInputsNumber, MATRIX_ZERO );
  Matrices.Discard( filter->errorCovarianceNoise );  
  filter->errorCovarianceNoise = Matrices.CreateSquare( newInputsNumber, MATRIX_IDENTITY );
  
  if( newInputsNumber > dimensionsNumber ) 
  {
    filter->gain = Matrices.Resize( filter->gain, newInputsNumber, newInputsNumber );
    filter->error = Matrices.Resize( filter->error, newInputsNumber, 1 );
  }
}

void Kalman_SetInput( KalmanFilter filter, size_t inputIndex, double value )
{
  if( filter == NULL ) return;
  
  Matrices.SetElement( filter->input, inputIndex, 0, value );
}

void Kalman_SetVariablesCoupling( KalmanFilter filter, size_t outputIndex, size_t inputIndex, double ratio )
{
  if( filter == NULL ) return;
  
  Matrices.SetElement( filter->prediction, outputIndex, inputIndex, ratio );
}

void Kalman_SetInputMaxError( KalmanFilter filter, size_t inputIndex, double maxError )
{
  if( filter == NULL ) return;
  
  Matrices.SetElement( filter->errorCovarianceNoise, inputIndex, inputIndex, maxError * maxError );
}

double* Kalman_Predict( KalmanFilter filter, double* result )
{
  if( filter == NULL ) return NULL;
  
  // x = F*x
  Matrices.Dot( filter->prediction, MATRIX_KEEP, filter->state, MATRIX_KEEP, filter->state );                                       // F[nxn] * x[nx1] -> x[nx1]
  
  // P = F*P*F' + Q
  Matrices.Dot( filter->prediction, MATRIX_KEEP, filter->predictionCovariance, MATRIX_KEEP, filter->predictionCovariance );         // F[nxn] * P[nxn] -> P[nxn]
  Matrices.Dot( filter->predictionCovariance, MATRIX_KEEP, filter->prediction, MATRIX_TRANSPOSE, filter->predictionCovariance );    // P[nxn] * F'[nxn] -> P[nxn]
  Matrices.Sum( filter->predictionCovariance, 1.0, filter->predictionCovarianceNoise, 1.0, filter->predictionCovariance );          // P[nxn] + Q[nxn] -> P[nxn]
  
  if( result == NULL ) return NULL;
  
  return Matrices.GetData( filter->state, result );
}

double* Kalman_Update( KalmanFilter filter, double* inputsList, double* result )
{
  if( filter == NULL ) return NULL;
  
  if( inputsList != NULL ) Matrices.SetData( filter->input, inputsList );
  
  // e = y - H*x
  Matrices.Dot( filter->inputModel, MATRIX_KEEP, filter->state, MATRIX_KEEP, filter->error );                             // H[mxn] * x[nx1] -> e[mx1]
  Matrices.Sum( filter->input, 1.0, filter->error, -1.0, filter->error );                                                 // y[mx1] - e[mx1] -> e[mx1]
  
  // S = H*P*H' + R
  Matrices.Dot( filter->inputModel, MATRIX_KEEP, filter->predictionCovariance, MATRIX_KEEP, filter->errorCovariance );    // H[mxn] * P[nxn] -> S[mxn]
  Matrices.Dot( filter->errorCovariance, MATRIX_KEEP, filter->inputModel, MATRIX_TRANSPOSE, filter->errorCovariance );    // S[mxn] * H'[nxm] -> S[mxm]
  Matrices.Sum( filter->errorCovariance, 1.0, filter->errorCovarianceNoise, 1.0, filter->errorCovariance );               // S[mxm] + R[mxm] -> S[mxm]
  
  // K = P*H' * S^(-1)
  Matrices.Dot( filter->predictionCovariance, MATRIX_KEEP, filter->inputModel, MATRIX_TRANSPOSE, filter->gain );          // P[nxn] * H'[nxm] -> K[nxm]
  if( Matrices.Inverse( filter->errorCovariance, filter->errorCovariance ) != NULL )                                      // S^(-1)[mxm] -> S[mxm]
  {
    Matrices.Dot( filter->gain, MATRIX_KEEP, filter->errorCovariance, MATRIX_KEEP, filter->gain );                          // K[nxm] * S[mxm] -> K[nxm]
    
    // x = x + K*e
    Matrices.Dot( filter->gain, MATRIX_KEEP, filter->error, MATRIX_KEEP, filter->error );                                   // K[nxm] * e[mx1] -> e[nx1]
    Matrices.Sum( filter->state, 1.0, filter->error, 1.0, filter->state );                                                  // x[nx1] + e[nx1] -> x[nx1]
    
    // P' = P - K*H*P
    Matrices.Dot( filter->gain, MATRIX_KEEP, filter->inputModel, MATRIX_KEEP, filter->gain );                               // K[nxm] * H[mxn] -> K[nxn]
    Matrices.Dot( filter->gain, MATRIX_KEEP, filter->predictionCovariance, MATRIX_KEEP, filter->gain );                     // K[nxn] * P[nxn] -> K[nxn]
    Matrices.Sum( filter->predictionCovariance, 1.0, filter->gain, -1.0, filter->predictionCovariance );                    // P[nxn] - K[nxn] -> P[nxn]
  }
  
  if( result == NULL ) return NULL;
  
  return Matrices.GetData( filter->state, result );
}

void Kalman_Reset( KalmanFilter filter )
{
  if( filter == NULL ) return;
  
  Matrices.Clear( filter->input );
  Matrices.Clear( filter->state );
  Matrices.Clear( filter->error );
  
  Matrices.Clear( filter->gain );
  Matrices.Clear( filter->predictionCovariance );
  Matrices.Clear( filter->errorCovariance );
}
