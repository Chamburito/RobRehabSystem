#ifndef EMG_OPTIMIZATION_H
#define EMG_OPTIMIZATION_H

#include "optimization.h"

#ifdef __cplusplus
    extern "C" {
#endif

typedef _SamplingData
{
  double* emgSignalList;
  double* ikTorquesList;
  size_t samplesNumber;
}
SamplingData;
      
static double CalculateMuscleParametersError( double* parametersList, size_t parametersNumber, void* ref_samplingData )
{
  SamplingData* samplingData = (SamplingData*) ref_samplingData; 
  
  double squaredErrorSum = 0;
  for( size_t sampleIndex = 0; sampleIndex < samplesNumber; sampleIndex++ )
  {
    
  }
  
  return squaredErrorSum / samplesNumber;
}

#ifdef __cplusplus
    }
#endif

#endif  // EMG_OPTIMIZATION_H
