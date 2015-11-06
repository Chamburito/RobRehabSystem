#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#ifdef __cplusplus
    extern "C" {
#endif

#include <math.h>
      
#define MATRIX_SIZE_MAX 100
      
enum OptimizationGoals { OPTIMIZATION_MINIMIZE, OPTIMIZATION_MAXIMIZE };

typedef double (*ObjectiveFunction)( double* parameters, const size_t parametersNumber, void* input );

#define OPTIMIZATION_FUNCTIONS( namespace, function_init ) \
        function_init( bool, namespace, Run, double*, size_t, ObjectiveFunction, void*, enum OptimizationGoals, size_t )

INIT_NAMESPACE_INTERFACE( Optimization, OPTIMIZATION_FUNCTIONS )

double CalculateGradient( double*, size_t, ObjectiveFunction, void*, double );
void CalculateHessian( size_t );
void UpdateParameters( double*, size_t, ObjectiveFunction, void*, double[ 2 ] );

static double* gradient;
static double* deltaParameters;
static double* deltaGradient;
static double** hessian;

bool Optimization_Run( double* parameters, size_t parametersNumber, ObjectiveFunction ref_Evaluate, void* input, enum OptimizationGoals goal, size_t maxIterations )
{
  bool optimizationSuccess = true;
  
  double iterationUpdateFactors[ 2 ] = { ( ( goal == OPTIMIZATION_MAXIMIZE ) ? 1 : -1 ), ( ( goal == OPTIMIZATION_MAXIMIZE ) ? 0.0001 : -0.0001 ) };
  
  deltaParameters = (double*) calloc( parametersNumber, sizeof(double) ); 
  deltaGradient = (double*) calloc( parametersNumber, sizeof(double) ); 
  gradient = (double*) calloc( parametersNumber, sizeof(double) );
  hessian = (double**) calloc( parametersNumber, sizeof(double*) );
  for( size_t parameterIndex = 0; parameterIndex < parametersNumber; parameterIndex++ )
  {
    hessian[ parameterIndex ] = (double*) calloc( parametersNumber, sizeof(double) );
    memset( hessian[ parameterIndex ], 0, parametersNumber * sizeof(double) );
    hessian[ parameterIndex ][ parameterIndex ] = 1.0;
  }
  
  for( size_t iterationsCount = 0; iterationsCount < maxIterations; iterationsCount++ )
  {
    double currentResult = ref_Evaluate( parameters, parametersNumber, input );
    double changeRate = CalculateGradient( parameters, parametersNumber, ref_Evaluate, input, currentResult );
    CalculateHessian( parametersNumber );
    
    UpdateParameters( parameters, parametersNumber, ref_Evaluate, input, iterationUpdateFactors );
    
    DEBUG_PRINT( "iteration %u (current: %g - change: %g * %g)", iterationsCount, currentResult, iterationUpdateFactors[ 0 ], changeRate );
  }
  
  free( deltaParameters );
  free( deltaGradient );
  
  free( gradient );
  for( size_t parameterIndex = 0; parameterIndex < parametersNumber; parameterIndex++ )
    free( hessian[ parameterIndex ] );
  free( hessian );
  
  return optimizationSuccess;
}

void Matrix_Dot( double** matrix_1, double** matrix_2, size_t rowsNumber, size_t size, size_t columnsNumber, double** result )
{
  static double auxMatrix[ MATRIX_SIZE_MAX ][ MATRIX_SIZE_MAX ];
  
  for( size_t i = 0; i < rowsNumber; i++ )
  {
    for( size_t j = 0; j < columnsNumber; j++ )
    {
      auxMatrix[ i ][ j ] = 0.0;
      for( size_t k = 0; k < size; k++ )
        auxMatrix[ i ][ j ] += matrix_1[ i ][ k ] * matrix_2[ k ][ j ];
    }
  }
  
  for( size_t row = 0; row < rowsNumber; row++ )
  {
    for( size_t column = 0; column < columnsNumber; column++ )
      result[ row ][ column ] = auxMatrix[ row ][ column ];
  }
}

void CofactorMatrix( double** matrix, size_t size, size_t row, size_t column, double** result )
{
  size_t m = 0, n = 0;
  for( size_t i = 0; i < size; i++ )
  {
    for( size_t j = 0 ; j < size; j++ )
    {
      if( i != row && j != column )
      {
        result[ m ][ n ] = matrix[ i ][ j ];

        if( n < size - 2 ) n++;
        else
        {
          n = 0;
          m++;
        }
      }
    }
  }
}

double Matrix_Determinant( double** matrix, size_t size )
{
  static double auxMatrix[ MATRIX_SIZE_MAX ][ MATRIX_SIZE_MAX ];
  
  if( size == 1 ) return matrix[ 0 ][ 0 ];
  
  double result = 0.0;
  for( size_t c = 0; c < size; c++ )
  {
    CofactorMatrix( matrix, size, 0, c, (double**) auxMatrix );

    result += pow( -1, c ) * matrix[ 0 ][ c ] * Matrix_Determinant( (double**) auxMatrix, size - 1 );
  }
    
  return result;
}

double Cofactor( double** matrix, size_t size, size_t row, size_t column )
{
  static double auxMatrix[ MATRIX_SIZE_MAX ][ MATRIX_SIZE_MAX ];
  
  CofactorMatrix( matrix, size, row, column, (double**) auxMatrix );
  
  double result = pow( -1, row + column ) * matrix[ row ][ column ] * Matrix_Determinant( (double**) auxMatrix, size - 1 );
  
  return result;
}

void Matrix_Transpose( double** matrix, size_t rowsNumber, size_t columnsNumber, double** result )
{
  static double auxMatrix[ MATRIX_SIZE_MAX ][ MATRIX_SIZE_MAX ];
  
  for( size_t row = 0; row < rowsNumber; row++ )
  {
    for( size_t column = 0; column < columnsNumber; column++ )
      auxMatrix[ row ][ column ] = matrix[ column ][ row ];
  }
  
  for( size_t row = 0; row < rowsNumber; row++ )
  {
    for( size_t column = 0; column < columnsNumber; column++ )
      result[ row ][ column ] = auxMatrix[ row ][ column ];
  }
}

void Matrix_Inverse( double** matrix, size_t size, double** result )
{
  static double auxMatrix[ MATRIX_SIZE_MAX ][ MATRIX_SIZE_MAX ];
  
  double det = Matrix_Determinant( matrix, size );
  
  if( det != 0.0 )
  {
    for( size_t row = 0; row < size; row++ )
    {
      for( size_t column = 0; column < size; column++ )
        auxMatrix[ row ][ column ] = Cofactor( matrix, size, column, row ) / det;
    }

    for( size_t row = 0; row < size; row++ )
    {
      for( size_t column = 0; column < size; column++ )
        result[ row ][ column ] = auxMatrix[ row ][ column ];
    }
  }
}

const double ITERATION_PARAMETER_DELTA = 0.001;//0.00001;
double CalculateGradient( double* parametersList, size_t parametersNumber, ObjectiveFunction ref_Evaluate, void* input, double currentValue )
{
  double changeRate = 0.0;
  
  for( size_t derivativeIndex = 0; derivativeIndex < parametersNumber; derivativeIndex++ )
  {
    parametersList[ derivativeIndex ] += ITERATION_PARAMETER_DELTA;
    
    double functionDelta = ref_Evaluate( parametersList, parametersNumber, input ) - currentValue;
    double functionDerivative = functionDelta / ITERATION_PARAMETER_DELTA;
    deltaGradient[ derivativeIndex ] = functionDerivative - gradient[ derivativeIndex ];
    gradient[ derivativeIndex ] = functionDerivative;
    
    changeRate += gradient[ derivativeIndex ] * gradient[ derivativeIndex ];
    
    parametersList[ derivativeIndex ] -= ITERATION_PARAMETER_DELTA;
  }
  
  return changeRate;
}

void CalculateHessian( size_t parametersNumber )
{
  static double auxMatrix[ MATRIX_SIZE_MAX ][ MATRIX_SIZE_MAX ], partialResult[ MATRIX_SIZE_MAX ][ MATRIX_SIZE_MAX ];
  
  //ddJ = H + ((1+q'*H*q)/(q'*q))*((pddJ*pddJ')/(pddJ'*q)) - ((pddJ*q'*H+H*(q*q'))/(q'*pddJ));
  
  Matrix_Transpose( (double**) deltaGradient, parametersNumber, 1, (double**) auxMatrix );                                      // q[Nx1]' = [1xN]
  Matrix_Dot( (double**) auxMatrix, hessian, 1, parametersNumber, parametersNumber, (double**) auxMatrix );                     // q'[1xN]*H[NxN] = [1xN]
  Matrix_Dot( (double**) auxMatrix, (double**) deltaGradient, 1, parametersNumber, 1, (double**) auxMatrix );                   // (q'*H)[1xN]*q[Nx1] = [1x1]
  double aux = 1.0 + auxMatrix[ 0 ][ 0 ];                                                                                       // 1[1x1] + (q'*H*q)[1x1] = [1x1]
  Matrix_Transpose( (double**) deltaGradient, parametersNumber, 1, (double**) auxMatrix );                                      // q[Nx1]' = [1xN]
  Matrix_Dot( (double**) auxMatrix, (double**) deltaGradient, 1, parametersNumber, 1, (double**) auxMatrix );                   // q'[1xN]*q[Nx1] = [1x1]
  aux /= auxMatrix[ 0 ][ 0 ];                                                                                                   // (1+q'*H*q)[1x1]/(q'*q)[1x1] = [1x1]
  Matrix_Transpose( (double**) deltaParameters, parametersNumber, 1, (double**) auxMatrix );                                    // pddJ[Nx1]' = [1xN]
  Matrix_Dot( (double**) auxMatrix, (double**) deltaGradient, 1, parametersNumber, 1, (double**) auxMatrix );                   // pddJ'[1xN]*q[Nx1] = [1x1]
  aux /= auxMatrix[ 0 ][ 0 ];                                                                                                   // ((1+q'*H*q)/(q'*q))[1x1]*(1/(pddJ'*q))[1x1] = [1x1]
  Matrix_Transpose( (double**) deltaParameters, parametersNumber, 1, (double**) auxMatrix );                                    // pddJ[Nx1]' = [1xN]
  Matrix_Dot( (double**) deltaParameters, (double**) auxMatrix, parametersNumber, 1, parametersNumber, (double**) auxMatrix );  // pddJ[Nx1]*pddJ'[1xN] = [NxN]
  for( size_t row = 0; row < parametersNumber; row++ )
  {
    for( size_t column = 0; column < parametersNumber; column++ )
      partialResult[ row ][ column ] = hessian[ row ][ column ] + auxMatrix[ row ][ column ] * aux;                          // H[NxN] + ((1+q'*H*q)/(q'*q))*(1/(pddJ'*q))[1x1]*(pddJ*pddJ')[NxN] = [NxN]
  }
  Matrix_Transpose( (double**) deltaGradient, parametersNumber, 1, (double**) auxMatrix );                                      // q[Nx1]' = [1xN]
  Matrix_Dot( (double**) auxMatrix, (double**) deltaParameters, 1, parametersNumber, 1, (double**) auxMatrix );                 // q'[1xN]*pddJ[Nx1] = [1x1]
  aux = auxMatrix[ 0 ][ 0 ];                                                                                                    // (q'*pddJ)[1x1]
  Matrix_Transpose( (double**) deltaGradient, parametersNumber, 1, (double**) auxMatrix );                                      // q[Nx1]' = [1xN]
  Matrix_Dot( (double**) deltaParameters, (double**) auxMatrix, parametersNumber, 1, parametersNumber, (double**) auxMatrix );  // pddJ[Nx1]*q'[1xN] = [NxN]
  Matrix_Dot( (double**) auxMatrix, hessian, parametersNumber, parametersNumber, parametersNumber, (double**) auxMatrix );      // (pddJ*q')[NxN]*H[NxN] = [NxN]
  for( size_t row = 0; row < parametersNumber; row++ )
  {
    for( size_t column = 0; column < parametersNumber; column++ )
      partialResult[ row ][ column ] -= auxMatrix[ row ][ column ] / aux;                          // (H + ((1+q'*H*q)/(q'*q))*((pddJ*pddJ')/(pddJ'*q)))[NxN] - ((pddJ*q'*H)[NxN]/(q'*pddJ)[1x1]) = [NxN]
  }
  Matrix_Transpose( (double**) deltaGradient, parametersNumber, 1, (double**) auxMatrix );                                      // q[Nx1]' = [1xN]
  Matrix_Dot( (double**) deltaGradient, (double**) auxMatrix, parametersNumber, 1, parametersNumber, (double**) auxMatrix );    // q[Nx1]*q'[1xN] = [NxN]
  Matrix_Dot( (double**) auxMatrix, hessian, parametersNumber, parametersNumber, parametersNumber, (double**) auxMatrix );      // H[NxN]*(q*q')[NxN] = [NxN]
  for( size_t row = 0; row < parametersNumber; row++ )
  {
    for( size_t column = 0; column < parametersNumber; column++ )
      hessian[ row ][ column ] = partialResult[ row ][ column ] - auxMatrix[ row ][ column ] / aux;                             // Result [NxN]
  }
}

void UpdateParameters( double* parametersList, size_t parametersNumber, ObjectiveFunction ref_Evaluate, void* input, double updateFactors[ 2 ] )
{
  static double auxMatrix[ MATRIX_SIZE_MAX ][ MATRIX_SIZE_MAX ], auxVector[ MATRIX_SIZE_MAX ][ MATRIX_SIZE_MAX ];
  static double newEstimation[ 2 ];
  
  Matrix_Transpose( (double**) gradient, parametersNumber, 1, (double**) auxVector );                                           // q[Nx1]' = [1xN]
  Matrix_Inverse( hessian, parametersNumber, (double**) auxMatrix );                                                            // H[NxN]^(-1) = invH[NxN]
  Matrix_Dot( (double**) auxVector, (double**) auxMatrix, 1, parametersNumber, parametersNumber, (double**) auxMatrix );        // q'[1xN]*invH[NxN] = [1xN]
  
  for( size_t estimationIndex = 0; estimationIndex < 2; estimationIndex++ )
  {
    for( size_t parameterIndex = 0; parameterIndex < parametersNumber; parameterIndex++ )
      auxVector[ estimationIndex ][ parameterIndex ] = parametersList[ parameterIndex ] + updateFactors[ estimationIndex ] * auxMatrix[ 0 ][ parameterIndex ];
    newEstimation[ estimationIndex ] = ref_Evaluate( (double*) &(auxVector[ estimationIndex ]), parametersNumber, input );
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
    deltaParameters[ parameterIndex ] = auxVector[ updateFactorIndex ][ parameterIndex ] - parametersList[ parameterIndex ];
    parametersList[ parameterIndex ] = auxVector[ updateFactorIndex ][ parameterIndex ];
  }
}

#ifdef __cplusplus
    }
#endif

#endif  // OPTIMIZATION_H
