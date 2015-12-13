#include "control_interface.h"

enum { NEGATIVE_BIG, NEGATIVE_SMALL, ZERO, POSITIVE_SMALL, POSITIVE_BIG, FUZZY_SETS_NUMBER };

typedef struct _FuzzySetData
{
  double medianValue;
  double variance;
}
FuzzySetData;

typedef FuzzySetData* FuzzySet;

const FuzzySetData POSITION_ERROR_SETS[ FUZZY_SETS_NUMBER ] = { { -1.0, 0.4 }, { -0.5, 0.4 }, { 0.0, 0.4 }, { 0.5, 0.4 }, { 1.0, 0.4 } };
const FuzzySetData FORCE_ERROR_SETS[ FUZZY_SETS_NUMBER ] = { { -100.0, 20.0 }, { -50.0, 20.0 }, { 0.0, 20.0 }, { 50.0, 20.0 }, { 100.0, 20.0 } };

const FuzzySetData VELOCITY_OUTPUT_SETS[ FUZZY_SETS_NUMBER ] = { { -500.0, 160.0 }, { -250.0, 160.0 }, { 0.0, 160.0 }, { 250.0, 160.0 }, { 500.0, 160.0 } };

//                     Force error
//              __NB___NS___ZE___PS___PB__
//           NB | PS | NS | NS | NS | NB |
// Position  NS | PS | ZE | ZE | NS | NS |
// error     ZE | PS | ZE | ZE | ZE | NS |
//           PS | PS | PS | ZE | ZE | NS |
//           PB | PB | PS | PS | PS | NS |
const int INFERENCE_RULES[ FUZZY_SETS_NUMBER ][ FUZZY_SETS_NUMBER ] = { { POSITIVE_SMALL, NEGATIVE_SMALL, NEGATIVE_SMALL, NEGATIVE_SMALL, NEGATIVE_BIG },
                                                                        { POSITIVE_SMALL, ZERO, ZERO, NEGATIVE_SMALL, NEGATIVE_SMALL },
                                                                        { POSITIVE_SMALL, ZERO, ZERO, ZERO, NEGATIVE_SMALL },
                                                                        { POSITIVE_SMALL, POSITIVE_SMALL, ZERO, ZERO, NEGATIVE_SMALL },
                                                                        { POSITIVE_BIG, POSITIVE_SMALL, POSITIIVE_SMALL, POSITIVE_SMALL, NEGATIVE_SMALL } };

IMPLEMENT_INTERFACE( CONTROL_FUNCTIONS )

double* Run( double measuresList[ CONTROL_VARS_NUMBER ], double setpointsList[ CONTROL_VARS_NUMBER ], double deltaTime, double* ref_error )
{
  static double outputsList[ CONTROL_VARS_NUMBER ];
  
  double positionError = measuresList[ CONTROL_POSITION ] - setpointsList[ CONTROL_POSITION ];
  double forceError = measuresList[ CONTROL_FORCE ] - setpointsList[ CONTROL_FORCE ];
  
  *ref_error = positionError;
  
  double velocityOutputSlices[ FUZZY_SETS_NUMBER ] = { 0.0 };
  
  for( size_t positionErrorSetIndex = 0; positionErrorSetIndex < FUZZY_SETS_NUMBER; positionErrorSetIndex++ )
  {
    for( size_t forceErrorSetIndex = 0; forceErrorSetIndex < FUZZY_SETS_NUMBER; forceErrorSetIndex++ )
    {
      FuzzySet positionErrorSet = POSITION_ERROR_SETS + positionErrorSetIndex;
      FuzzySet forceErrorSet = FORCE_ERROR_SETS + forceErrorSetIndex;
      
      double positionErrorSetActivation = exp( -pow( positionError - positionErrorSet->medianValue, 2 ) / positionErrorSet->variance );
      double forceErrorSetActivation = exp( -pow( forceError - forceErrorSet->medianValue, 2 ) / forceErrorSet->variance );
      
      size_t velocityOutputSetIndex = INFERENCE_RULES[ positionErrorSetIndex ][ forceErrorSetIndex ];
      
      double velocityOutputActivation = ( positionErrorSetActivation < forceErrorSetActivation ) ? positionErrorSetActivation : forceErrorSetActivation;
      
      if( velocityOutputActivation > velocityOutputSlices[ velocityOutputSetIndex ] ) velocityOutputSlices[ velocityOutputSetIndex ] = velocityOutputActivation;
    }
  }
  
  double velocityOutputsSum = 0.0;
  double velocityOutputsWeightedSum = 0.0;
  for( int velocityOutput = -500; velocityOutputSet <= 500; velocityOutputSet++ )
  {
    double maxOutputSetActivation = 0.0;
    
    for( size_t velocityOutputSetIndex = 0; velocityOutputSetIndex < FUZZY_SETS_NUMBER; velocityOutputSetIndex++ )
    {
      FuzzySet velocityOutputSet = VELOCITY_OUTPUT_SETS + velocityOutputSetIndex;
      
      double velocityOutputActivation = exp( -pow( velocityOutput - velocityOutputSet->medianValue, 2 ) / velocityOutputSet->variance );
      
      if( velocityOutputActivation > velocityOutputSlices[ velocityOutputSetIndex ] ) velocityOutputActivation = velocityOutputSlices[ velocityOutputSetIndex ];
      
      if( velocityOutputActivation > maxOutputSetActivation ) maxOutputSetActivation = velocityOutputActivation;
    }
    
    velocityOutputsSum += maxOutputSetActivation;
    velocityOutputsWeightedSum += velocityOutput * maxOutputSetActivation;
  }
  
  outputsList[ CONTROL_VELOCITY ] = velocityOutputsWeightedSum / velocityOutputsSum;
  
  return (double*) outputsList;
}
