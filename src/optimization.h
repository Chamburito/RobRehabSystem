#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "matrices.h"
      
#include <math.h>
      
enum OptimizationGoals { OPTIMIZATION_MINIMIZE, OPTIMIZATION_MAXIMIZE };

typedef double (*ObjectiveFunction)( double* parameters, const size_t parametersNumber, void* input );

#define OPTIMIZATION_FUNCTIONS( namespace, function_init ) \
        function_init( bool, namespace, Run, double*, size_t, ObjectiveFunction, void*, enum OptimizationGoals, size_t )

INIT_NAMESPACE_INTERFACE( Optimization, OPTIMIZATION_FUNCTIONS )

double CalculateGradient( double*, size_t, ObjectiveFunction, void*, double );
void CalculateHessian( size_t );
void UpdateParameters( double*, size_t, ObjectiveFunction, void*, double[ 2 ] );

static Matrix gradient = NULL;
static Matrix deltaParameters = NULL;
static Matrix deltaGradient = NULL;
static double** hessian;

bool Optimization_Run( double* parameters, size_t parametersNumber, ObjectiveFunction ref_Evaluate, void* input, enum OptimizationGoals goal, size_t maxIterations )
{
  bool optimizationSuccess = true;
  
  double iterationUpdateFactors[ 2 ] = { ( ( goal == OPTIMIZATION_MAXIMIZE ) ? 1 : -1 ), ( ( goal == OPTIMIZATION_MAXIMIZE ) ? 0.0001 : -0.0001 ) };
  
  deltaParameters = Matrices_Create( NULL, parametersNumber, 1 ); 
  deltaGradient = Matrices_Create( NULL, parametersNumber, 1 ); 
  gradient = Matrices_Create( NULL, parametersNumber, 1 );
  hessian = Matrices_CreateSquare( parametersNumber, MATRIX_IDENTITY );
  
  for( size_t iterationsCount = 0; iterationsCount < maxIterations; iterationsCount++ )
  {
    double currentResult = ref_Evaluate( parameters, parametersNumber, input );
    double changeRate = CalculateGradient( parameters, parametersNumber, ref_Evaluate, input, currentResult );
    CalculateHessian( parametersNumber );
    
    UpdateParameters( parameters, parametersNumber, ref_Evaluate, input, iterationUpdateFactors );
    
    DEBUG_PRINT( "iteration %u (current: %g - change: %g * %g)", iterationsCount, currentResult, iterationUpdateFactors[ 0 ], changeRate );
  }
  
  Matrices_Discard( deltaParameters );
  Matrices_Discard( deltaGradient );
  Matrices_Discard( gradient );
  Matrices_Discard( hessian );
  
  return optimizationSuccess;
}

const double ITERATION_PARAMETER_DELTA = 0.001;//0.00001;
double CalculateGradient( double* parametersList, size_t parametersNumber, ObjectiveFunction ref_Evaluate, void* input, double currentValue )
{
  double changeRate = 0.0;
  
  double* gradientList = Matrices_GetAsVector( gradient );
  double* deltaGradientList = Matrices_GetAsVector( deltaGradient );
  
  for( size_t derivativeIndex = 0; derivativeIndex < parametersNumber; derivativeIndex++ )
  {
    parametersList[ derivativeIndex ] += ITERATION_PARAMETER_DELTA;
    
    double functionDelta = ref_Evaluate( parametersList, parametersNumber, input ) - currentValue;
    double functionDerivative = functionDelta / ITERATION_PARAMETER_DELTA;
    deltaGradientList[ derivativeIndex ] = functionDerivative - gradientList[ derivativeIndex ];
    gradientList[ derivativeIndex ] = functionDerivative;
    
    changeRate += gradientList[ derivativeIndex ] * gradientList[ derivativeIndex ];
    
    parametersList[ derivativeIndex ] -= ITERATION_PARAMETER_DELTA;
  }
  
  return changeRate;
}

void CalculateHessian( size_t parametersNumber )
{
  //ddJ = H + ((1+q'*H*q)/(q'*q))*((pddJ*pddJ')/(pddJ'*q)) - ((pddJ*q'*H+H*(q*q'))/(q'*pddJ));
  
  Matrix auxMatrix = Matrices_Transpose( deltaGradient, NULL );                                     // q[Nx1]' = [1xN]
  Matrices_Dot( auxMatrix, hessian, auxMatrix );                                                    // q'[1xN]*H[NxN] = [1xN]
  Matrices_Dot( auxMatrix, deltaGradient, auxMatrix );                                              // (q'*H)[1xN]*q[Nx1] = [1x1]
  double aux = 1.0 + Matrices_GetElement( auxMatrix, 0, 0 );                                        // 1[1x1] + (q'*H*q)[1x1] = [1x1]
  Matrices_Transpose( deltaGradient, auxMatrix );                                                   // q[Nx1]' = [1xN]
  Matrices_Dot( auxMatrix, deltaGradient, auxMatrix );                                              // q'[1xN]*q[Nx1] = [1x1]
  aux /= Matrices_GetElement( auxMatrix, 0, 0 );                                                    // (1+q'*H*q)[1x1]/(q'*q)[1x1] = [1x1]
  Matrices_Transpose( deltaParameters, auxMatrix );                                                 // pddJ[Nx1]' = [1xN]
  Matrices_Dot( auxMatrix, deltaGradient, auxMatrix );                                              // pddJ'[1xN]*q[Nx1] = [1x1]
  aux /= Matrices_GetElement( auxMatrix, 0, 0 );                                                    // ((1+q'*H*q)/(q'*q))[1x1]*(1/(pddJ'*q))[1x1] = [1x1]
  Matrices_Transpose( deltaParameters, auxMatrix );                                                 // pddJ[Nx1]' = [1xN]
  Matrices_Dot( deltaParameters, auxMatrix, auxMatrix );                                            // pddJ[Nx1]*pddJ'[1xN] = [NxN]
  
  Matrix partialResult = Matrices_Sum( hessian, auxMatrix, NULL );                                  // H[NxN] + ((1+q'*H*q)/(q'*q))*(1/(pddJ'*q))[1x1]*(pddJ*pddJ')[NxN] = [NxN]
  
  Matrices_Transpose( deltaGradient, auxMatrix );                                                   // q[Nx1]' = [1xN]
  Matrices_Dot( auxMatrix, deltaParameters, auxMatrix );                                            // q'[1xN]*pddJ[Nx1] = [1x1]
  aux = Matrices_GetElement( auxMatrix, 0, 0 );                                                     // (q'*pddJ)[1x1]
  Matrices_Transpose( deltaGradient, auxMatrix );                                                   // q[Nx1]' = [1xN]
  Matrices_Dot( deltaParameters, auxMatrix, auxMatrix );                                            // pddJ[Nx1]*q'[1xN] = [NxN]
  Matrices_Dot( auxMatrix, hessian, auxMatrix );                                                    // (pddJ*q')[NxN]*H[NxN] = [NxN]
  
  Matrices_Scale( auxMatrix, -1.0/aux, auxMatrix );
  Matrices_Sum( partialResult, auxMatrix, partialResult );                                          // (H + ((1+q'*H*q)/(q'*q))*((pddJ*pddJ')/(pddJ'*q)))[NxN] - ((pddJ*q'*H)[NxN]/(q'*pddJ)[1x1]) = [NxN]
  
  Matrices_Transpose( deltaGradient, auxMatrix );                                                   // q[Nx1]' = [1xN]
  Matrices_Dot( deltaGradient, auxMatrix, auxMatrix );                                              // q[Nx1]*q'[1xN] = [NxN]
  Matrices_Dot( auxMatrix, hessian, auxMatrix );                                                    // H[NxN]*(q*q')[NxN] = [NxN]
  
  Matrices_Scale( auxMatrix, -1.0/aux, auxMatrix );
  Matrices_Sum( partialResult, auxMatrix, hessian );                                                // Result [NxN]
  
  Matrices_Discard( auxMatrix );
  Matrices_Discard( partialResult );
}

void UpdateParameters( double* parametersList, size_t parametersNumber, ObjectiveFunction ref_Evaluate, void* input, double updateFactors[ 2 ] )
{
  static double newEstimation[ 2 ];
  
  Matrix auxVector = Matrices_Transpose( gradient, NULL );                                          // q[Nx1]' = [1xN]
  Matrix auxMatrix = Matrices_Inverse( hessian, auxMatrix );                                        // H[NxN]^(-1) = invH[NxN]
  Matrices_Dot( auxVector, auxMatrix, auxVector );                                                  // q'[1xN]*invH[NxN] = [1xN]
  
  double* parametersTestList = Matrices_GetAsVector( auxMatrix );
  double* changeRatesList = Matrices_GetAsVector( auxVector );
  
  for( size_t estimationIndex = 0; estimationIndex < 2; estimationIndex++ )
  {
    for( size_t parameterIndex = 0; parameterIndex < parametersNumber; parameterIndex++ )
    {
      double parameterEstimation = parametersList[ parameterIndex ] + updateFactors[ estimationIndex ] * changeRatesList[ parameterIndex ];
      parametersTestList[ estimationIndex * parametersNumber + parameterIndex ] = parameterEstimation;
    }
    newEstimation[ estimationIndex ] = ref_Evaluate( parametersTestList + estimationIndex * parametersNumber ), parametersNumber, input );
  }
  
  size_t updateFactorIndex;
  if( updateFactors[ 0 ] * newEstimation[ 0 ] > updateFactors[ 0 ] * newEstimation[ 1 ] )
  {
    updateFactorIndex = 0;
    updateFactors[ 1 ] = ( updateFactors[ 0 ] + updateFactors[ 1 ] ) / 2; 
  }
  else
  {
    updateFactorIndex = 1;
    updateFactors[ 0 ] = ( updateFactors[ 0 ] + updateFactors[ 1 ] ) / 2;
  }
  
  for( size_t parameterIndex = 0; parameterIndex < parametersNumber; parameterIndex++ )
  {
    deltaParameters[ parameterIndex ] = parametersTestList[ updateFactorIndex * parametersNumber + parameterIndex ] - parametersList[ parameterIndex ];
    parametersList[ parameterIndex ] = parametersTestList[ updateFactorIndex * parametersNumber + parameterIndex ];
  }
  
  Matrix_Discard( auxVector );
  Matrix_Discard( auxMatrix );
}

#ifdef __cplusplus
    }
#endif

#endif  // OPTIMIZATION_H
