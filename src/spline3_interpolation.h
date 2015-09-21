#ifndef SPLINE3_INTERP_H
#define SPLINE3_INTERP_H

#include <math.h>

const size_t CURVE_NAME_MAX_LENGTH = 32;

const size_t SPLINE_COEFFS_NUMBER = 4;
typedef double SplineCoeffs[ SPLINE_COEFFS_NUMBER ];
typedef double SplineBounds[ 2 ];

typedef struct _Splined3Curve
{
  SplineCoeffs* splinesList;
  SplineBounds* splineBoundsList;
  size_t splinesNumber;
  double scaleFactor, maxScaleFactor;
  double totalLength;
}
Splined3Curve;

static Splined3Curve* LoadCurve( const char* );
static void UnloadCurve( Splined3Curve* );
static void SetScale( Splined3Curve*, double );
static double GetValue( Splined3Curve*, double );

const struct 
{
  Splined3Curve* (*LoadCurve)( const char* );
  void (*UnloadCurve)( Splined3Curve* );
  void (*SetScale)( Splined3Curve*, double );
  double (*GetValue)( Splined3Curve*, double );
}
Spline3Interp = { .LoadCurve = LoadCurve, .UnloadCurve = UnloadCurve, .SetScale = SetScale, .GetValue = GetValue };

static Splined3Curve* LoadCurve( const char* curveName )
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
        
          /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "found curve %s", readBuffer );
        
          newCurveData->splinesList = NULL;
          newCurveData->splineBoundsList = NULL;
          newCurveData->splinesNumber = 0;
          newCurveData->scaleFactor = 1.0;
          newCurveData->totalLength = 1000.0;
        }
      }
      
      if( newCurveData == NULL ) continue;
      
      if( strcmp( readBuffer, "phase:" ) == 0 )
      {
        double initialPosition, initialValue, initialDerivative;
        double finalPosition, finalValue, finalDerivative;
        
        fscanf( configFile, "%lf %lf %lf", &initialPosition, &initialValue, &initialDerivative );
        fscanf( configFile, " -> %lf %lf %lf", &finalPosition, &finalValue, &finalDerivative );
        
        double splineLength = finalPosition - initialPosition;
        
        if( splineLength > 0.0 )
        {
          newCurveData->splinesList = (SplineCoeffs*) realloc( newCurveData->splinesList, ( newCurveData->splinesNumber + 1 ) * sizeof(SplineCoeffs) );
          newCurveData->splineBoundsList = (SplineBounds*) realloc( newCurveData->splineBoundsList, ( newCurveData->splinesNumber + 1 ) * sizeof(SplineBounds) );
        
          double* splineCoeffs = newCurveData->splinesList[ newCurveData->splinesNumber ];
          double* splineBounds = newCurveData->splineBoundsList[ newCurveData->splinesNumber ];
        
          splineBounds[ 0 ] = initialPosition;
          splineBounds[ 1 ] = finalPosition;
        
          // Spline ( x = d + c*t + b*t^2 + a*t^3 ) coefficients calculation
          splineCoeffs[ 0 ] = initialValue;
          splineCoeffs[ 1 ] = initialDerivative;
          splineCoeffs[ 2 ] = ( 3 * ( finalValue - initialValue ) - splineLength * ( 2 * initialDerivative + finalDerivative ) ) / pow( splineLength, 2 );
          splineCoeffs[ 3 ] = ( 2 * ( initialValue - finalValue ) + splineLength * ( initialDerivative + finalDerivative ) ) / pow( splineLength, 3 );
        
          /*DEBUG_EVENT( 1,*/DEBUG_PRINT( " found spline: %g %g %g %g", splineCoeffs[ 0 ], splineCoeffs[ 1 ], splineCoeffs[ 2 ], splineCoeffs[ 3 ] );
        
          newCurveData->splinesNumber++;
        }
      }
      else if( strcmp( readBuffer, "duration:" ) == 0 ) 
      {
        fscanf( configFile, "%lf", &(newCurveData->totalLength) );
        
        /*DEBUG_EVENT( 2,*/DEBUG_PRINT( " found curve duration: %g", newCurveData->totalLength );
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

static void UnloadCurve( Splined3Curve* curveData )
{
  DEBUG_PRINT( "unloading curve %p data", curveData );
  
  if( curveData != NULL )
  {
    free( curveData->splinesList );
    free( curveData->splineBoundsList );
    
    free( curveData );
  }
}

static void SetScale( Splined3Curve* curveData, double scaleFactor )
{
  if( curveData == NULL ) return;
  
  curveData->scaleFactor = scaleFactor;
}

static double GetValue( Splined3Curve* curveData, double valuePosition )
{
  if( curveData == NULL ) return 1.0;
  
  if( curveData->splinesNumber == 0 ) return curveData->scaleFactor;
  
  double curveValue = 0.0;
  
  valuePosition = fmod( valuePosition, curveData->totalLength );
  //if( valuePosition < curveData->splineBoundsList[ 0 ][ 0 ] ) valuePosition += curveData->totalLength;

  double* splineCoeffs = curveData->splinesList[ 0 ];
  double relativePosition = curveData->splineBoundsList[ 0 ][ 0 ];

  for( size_t splineID = 0; splineID < curveData->splinesNumber; splineID++ )
  {
    if( valuePosition >= curveData->splineBoundsList[ splineID ][ 0 ] )
    {
      splineCoeffs = curveData->splinesList[ splineID ];

      if( valuePosition < curveData->splineBoundsList[ splineID ][ 1 ] )
      {
        relativePosition = valuePosition - curveData->splineBoundsList[ splineID ][ 0 ];
        break;
      }
      else
        relativePosition = curveData->splineBoundsList[ splineID ][ 1 ] - curveData->splineBoundsList[ splineID ][ 0 ];
    }
  }

  for( size_t coeffIndex = 0; coeffIndex < SPLINE_COEFFS_NUMBER; coeffIndex++ )
    curveValue += splineCoeffs[ coeffIndex ] * pow( relativePosition, coeffIndex );
  
  return curveData->scaleFactor * curveValue;
}

#endif  /* SPLINE3_INTERP_H */
