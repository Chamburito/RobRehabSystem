#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#ifdef __cplusplus
    extern "C" {
#endif
      
enum OptimizationGoals { OPTIMIZATION_MINIMIZE, OPTIMIZATION_MAXIMIZE };

typedef double (*ObjectiveFunction)( double* parameters, const size_t parametersNumber, void* input );

#define OPTIMIZATION_FUNCTIONS( namespace, function_init ) \
        function_init( bool, namespace, Run, double*, size_t, ObjectiveFunction, void*, enum OptimizationGoals, size_t )

INIT_NAMESPACE_INTERFACE( Optimization, OPTIMIZATION_FUNCTIONS )

double CalculateGradient( double*, size_t, ObjectiveFunction, void*, double, double* );
void CalculateHessian( double*, size_t, ObjectiveFunction, void*, double, double*, double** );
void UpdateParameters( double*, size_t, double*, double* );

const double ITERATION_UPDATE_FACTOR_INIT = 1;//0.002;
bool Optimization_Run( double* parameters, size_t parametersNumber, ObjectiveFunction ref_Evaluate, void* input, enum OptimizationGoals goal, size_t maxIterations )
{
  bool optimizationSuccess = true;
  
  double iterationUpdateFactor = ITERATION_UPDATE_FACTOR_INIT * ( ( goal == OPTIMIZATION_MAXIMIZE ) ? 1 : -1 );
  
  double* gradient = (double*) calloc( parametersNumber, sizeof(double) );
  //double** hessian = (double**) calloc( parametersNumber, sizeof(double*) );
  //for( size_t parameterIndex = 0; parameterIndex < parametersNumber; parameterIndex++ )
  //  hessian[ parameterIndex ] = (double*) calloc( parametersNumber, sizeof(double) );
  
  for( size_t iterationsCount = 0; iterationsCount < maxIterations; iterationsCount++ )
  {
    double currentResult = ref_Evaluate( parameters, parametersNumber, input );
    double changeRate = CalculateGradient( parameters, parametersNumber, ref_Evaluate, input, currentResult, gradient );
    //CalculateHessian( parameters, parametersNumber, ref_Evaluate, input, currentResult, gradient, hessian );
    
    UpdateParameters( parameters, parametersNumber, gradient, &iterationUpdateFactor );
    
    DEBUG_PRINT( "iteration %u (current: %g - change: %g * %g)", iterationsCount, currentResult, iterationUpdateFactor, changeRate );
  }
  
  free( gradient );
  //for( size_t parameterIndex = 0; parameterIndex < parametersNumber; parameterIndex++ )
  //  free( hessian[ parameterIndex ] );
  //free( hessian );
  
  return optimizationSuccess;
}

const double ITERATION_PARAMETER_DELTA = 0.001;//0.00001;
double CalculateGradient( double* variablesList, size_t variablesNumber, ObjectiveFunction ref_Evaluate, void* input, double currentValue, double* result )
{
  double changeRate = 0.0;
  
  for( size_t derivativeIndex = 0; derivativeIndex < variablesNumber; derivativeIndex++ )
  {
    variablesList[ derivativeIndex ] += ITERATION_PARAMETER_DELTA;
    
    double functionDelta = ref_Evaluate( variablesList, variablesNumber, input ) - currentValue;
    result[ derivativeIndex ] = functionDelta / ITERATION_PARAMETER_DELTA;
    
    changeRate += result[ derivativeIndex ] * result[ derivativeIndex ];
    
    variablesList[ derivativeIndex ] -= ITERATION_PARAMETER_DELTA;
  }
  
  return changeRate;
}

void CalculateHessian( double* variablesList, size_t variablesNumber, ObjectiveFunction ref_Evaluate, void* input, double currentValue, double* gradient, double** result )
{
  double nextValue[ 2 ], functionDerivative[ 2 ];
  
  for( size_t derivativeRow = 0; derivativeRow < variablesNumber; derivativeRow++ )
  {
    variablesList[ derivativeRow ] += ITERATION_PARAMETER_DELTA;
    
    nextValue[ 0 ] = ref_Evaluate( variablesList, variablesNumber, input );
    functionDerivative[ 0 ] = ( nextValue[ 0 ] - currentValue ) / ITERATION_PARAMETER_DELTA;
    
    for( size_t derivativeColumn = 0; derivativeColumn < variablesNumber; derivativeColumn++ )
    {
      variablesList[ derivativeColumn ] += ITERATION_PARAMETER_DELTA;

      nextValue[ 1 ] = ref_Evaluate( variablesList, variablesNumber, input );
      functionDerivative[ 1 ] = ( nextValue[ 1 ] - nextValue[ 0 ] ) / ITERATION_PARAMETER_DELTA;
      
      double gradientChange = functionDerivative[ 1 ] - functionDerivative[ 0 ];
      result[ derivativeRow ][ derivativeColumn ] = gradientChange / ITERATION_PARAMETER_DELTA;

      variablesList[ derivativeColumn ] -= ITERATION_PARAMETER_DELTA;
    }
    
    variablesList[ derivativeRow ] -= ITERATION_PARAMETER_DELTA;
  }
}

const double ITERATION_UPDATE_FACTOR_CHANGE = 0.98;
void UpdateParameters( double* parametersList, size_t parametersNumber, double* changeRatesList, double* ref_updateFactor )
{
  double updateFactor = (*ref_updateFactor);
  
  for( size_t parameterIndex = 0; parameterIndex < parametersNumber; parameterIndex++ )
    parametersList[ parameterIndex ] += updateFactor * changeRatesList[ parameterIndex ];
  
  (*ref_updateFactor) = updateFactor * ITERATION_UPDATE_FACTOR_CHANGE;
}

#ifdef __cplusplus
    }
#endif

#endif  // OPTIMIZATION_H
