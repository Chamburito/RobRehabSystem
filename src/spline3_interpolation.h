#ifndef SPLINE3_INTERP_H
#define SPLINE3_INTERP_H

#include <math.h>

const size_t CURVE_NAME_MAX_LENGTH = 16;

const size_t SPLINE_COEFFS_NUMBER = 4;
typedef double SplineCoeffs[ SPLINE_COEFFS_NUMBER ];
typedef double SplineBounds[ 2 ];

typedef struct _Splined3Curve
{
  SplineCoeffs* splinesList;
  SplineBounds* splineTimes;
  size_t splinesNumber;
  double totalTimeLength;
}
Splined3Curve;

Splined3Curve* Spline3Interp_LoadCurve( const char* curveName )
{
  char readBuffer[ CURVE_NAME_MAX_LENGTH ];
  
  Splined3Curve* newCurveData = NULL;
  
  FILE* configFile = fopen( "../config/splined3_curves_data.txt", "r" );
  
  if( configFile != NULL ) 
  {
    while( fscanf( configFile, "%s", readBuffer ) != EOF )
    {
      if( strcmp( readBuffer, "BEGIN_CURVE" ) == 0 )
      {
        fscanf( configFile, "%s", readBuffer );
        
        if( strncmp( readBuffer, curveName, CURVE_NAME_MAX_LENGTH ) == 0 )
        {
          newCurveData = (Splined3Curve*) malloc( sizeof(Splined3Curve) );
        
          DEBUG_EVENT( 0, "found curve %s", readBuffer );
        
          newCurveData->splinesList = NULL;
          newCurveData->splineTimes = NULL;
          newCurveData->splinesNumber = 0;
          newCurveData->totalTimeLength = 1000.0;
        }
      }
      
      if( newCurveData == NULL ) continue;
      
      if( strcmp( readBuffer, "phase:" ) == 0 )
      {
        double initialTime, initialValue, initialDerivative;
        double finalTime, finalValue, finalDerivative;
        
        fscanf( configFile, "%lf %lf %lf", &initialTime, &initialValue, &initialDerivative );
        fscanf( configFile, " -> %lf %lf %lf", &finalTime, &finalValue, &finalDerivative );
        
        double splineTimeLength = finalTime - initialTime;
        
        if( splineTimeLength > 0.0 )
        {
          newCurveData->splinesList = (SplineCoeffs*) realloc( newCurveData->splinesList, ( newCurveData->splinesNumber + 1 ) * sizeof(SplineCoeffs) );
          newCurveData->splineTimes = (SplineBounds*) realloc( newCurveData->splineTimes, ( newCurveData->splinesNumber + 1 ) * sizeof(SplineBounds) );
        
          double* splineCoeffs = newCurveData->splinesList[ newCurveData->splinesNumber ];
          double* splineBounds = newCurveData->splineTimes[ newCurveData->splinesNumber ];
        
          splineBounds[ 0 ] = initialTime;
          splineBounds[ 1 ] = finalTime;
        
          // Spline ( x = d + c*t + b*t^2 + a*t^3 ) coefficients calculation
          splineCoeffs[ 0 ] = initialValue;
          splineCoeffs[ 1 ] = initialDerivative;
          splineCoeffs[ 2 ] = ( 3 * ( finalValue - initialValue ) - splineTimeLength * ( 2 * initialDerivative + finalDerivative ) ) / pow( splineTimeLength, 2 );
          splineCoeffs[ 3 ] = ( 2 * ( initialValue - finalValue ) + splineTimeLength * ( initialDerivative + finalDerivative ) ) / pow( splineTimeLength, 3 );
        
          DEBUG_EVENT( 1, " found spline: %g %g %g %g", splineCoeffs[ 0 ], splineCoeffs[ 1 ], splineCoeffs[ 2 ], splineCoeffs[ 3 ] );
        
          newCurveData->splinesNumber++;
        }
      }
      else if( strcmp( readBuffer, "duration:" ) == 0 ) 
      {
        fscanf( configFile, "%lf", &(newCurveData->totalTimeLength) );
        
        DEBUG_EVENT( 2, " found curve duration: %g", newCurveData->totalTimeLength );
      }
      else if( strcmp( readBuffer, "END_CURVE" ) == 0 ) 
      {
        if( newCurveData->splinesNumber == 0 )
        {
          free( newCurveData );
          newCurveData = NULL;
        }
        
        break;
      }
      else if( strcmp( readBuffer, "#" ) == 0 )
      {
        char dummyChar;
        
        do { 
          if( fscanf( configFile, "%c", &dummyChar ) == EOF ) 
            break; 
        } while( dummyChar != '\n' ); 
      }
    }
    
    fclose( configFile );
  }
  
  return newCurveData;
}

void Spline3Interp_UnloadCurve( Splined3Curve* curveData )
{
  if( curveData != NULL )
  {
    free( curveData->splinesList );
    free( curveData->splineTimes );
    
    free( curveData );
  }
}

double Spline3Interp_GetValue( Splined3Curve* curveData, double valuePoint )
{
  if( curveData == NULL ) return 1.0;
  
  double* splineCoeffs = NULL;
  
  double relativePoint = fmod( valuePoint, curveData->totalTimeLength );
  
  for( size_t splineID = 0; splineID < curveData->splinesNumber; splineID++ )
  {
    if( relativePoint >= curveData->splineTimes[ splineID ][ 0 ] && relativePoint < curveData->splineTimes[ splineID ][ 1 ] )
    {
      splineCoeffs = curveData->splinesList[ splineID ];
      
      if( relativePoint < curveData->splineTimes[ splineID ][ 1 ] )
      {
        relativePoint = relativePoint - curveData->splineTimes[ splineID ][ 0 ]; 
        break;
      }
      else
        relativePoint = curveData->splineTimes[ splineID ][ 1 ] - curveData->splineTimes[ splineID ][ 0 ]; 
    }
  }
  
  double curveValue = 0.0;
  
  if( splineCoeffs != NULL )
  {
    for( size_t coeffIndex = 0; coeffIndex < SPLINE_COEFFS_NUMBER; coeffIndex++ ) 
      curveValue += splineCoeffs[ coeffIndex ] * pow( relativePoint, coeffIndex );
  }
  
  return curveValue;
}

#endif  /* SPLINE3_INTERP_H */
