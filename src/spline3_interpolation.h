#ifndef SPLINE3_INTERP_H
#define SPLINE3_INTERP_H

#include <math.h>

#include "file_parsing/json_parser.h"

const size_t CURVE_NAME_MAX_LENGTH = 32;

const size_t SPLINE_COEFFS_NUMBER = 4;
typedef double SplineCoeffs[ SPLINE_COEFFS_NUMBER ];
typedef double SplineBounds[ 2 ];

typedef struct _Splined3CurveData
{
  SplineCoeffs* splinesList;
  SplineBounds* splineBoundsList;
  size_t splinesNumber;
  double scaleFactor, maxScaleFactor;
  double totalLength;
}
Splined3CurveData;

typedef Splined3CurveData* Splined3Curve;

/*static Splined3Curve LoadCurve( const char* );
static void UnloadCurve( Splined3Curve );
static void SetScale( Splined3Curve, double );
static double GetValue( Splined3Curve, double );

const struct 
{
  Splined3Curve (*LoadCurve)( const char* );
  void (*UnloadCurve)( Splined3Curve );
  void (*SetScale)( Splined3Curve, double );
  double (*GetValue)( Splined3Curve, double );
}
Spline3Interp = { .LoadCurve = LoadCurve, .UnloadCurve = UnloadCurve, .SetScale = SetScale, .GetValue = GetValue };*/

#define NAMESPACE Spline3Interp

#define NAMESPACE_FUNCTIONS( namespace ) \
        NAMESPACE_FUNCTION( Splined3Curve, namespace, LoadCurve, const char* ) \
        NAMESPACE_FUNCTION( void, namespace, UnloadCurve, Splined3Curve ) \
        NAMESPACE_FUNCTION( void, namespace, SetScale, Splined3Curve, double ) \
        NAMESPACE_FUNCTION( double, namespace, GetValue, Splined3Curve, double )

#define NAMESPACE_FUNCTION( rvalue, namespace, name, ... ) static rvalue namespace##_##name( __VA_ARGS__ );
NAMESPACE_FUNCTIONS( NAMESPACE )
#undef NAMESPACE_FUNCTION

#define NAMESPACE_FUNCTION( rvalue, namespace, name, ... ) rvalue (*name)( __VA_ARGS__ );
const struct { NAMESPACE_FUNCTIONS( NAMESPACE ) }
#undef NAMESPACE_FUNCTION
#define NAMESPACE_FUNCTION( rvalue, namespace, name, ... ) .name = namespace##_##name,
NAMESPACE = { NAMESPACE_FUNCTIONS( NAMESPACE ) };
#undef NAMESPACE_FUNCTION

#undef NAMESPACE_FUNCTIONS
#undef NAMESPACE

static Splined3Curve Spline3Interp_LoadCurve( const char* curveName )
{
  static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  Splined3Curve newCurve = (Splined3Curve) malloc( sizeof(Splined3CurveData) );
  memset( newCurve, 0, sizeof(Splined3CurveData) );
  
  if( curveName != NULL )
  {
    FileParser parser = JSONParser;
    int configFileID = parser.LoadFile( curveName );
  
    if( configFileID != -1 )
    {
      newCurve->maxScaleFactor = parser.GetRealValue( configFileID, "max_scale" );
      newCurve->totalLength = parser.GetRealValue( configFileID, "total_length" );

      if( (newCurve->splinesNumber = (size_t) parser.GetListSize( configFileID, "phases" )) > 0 )
      {
        newCurve->splinesList = (SplineCoeffs*) calloc( newCurve->splinesNumber + 1, sizeof(SplineCoeffs) );
        newCurve->splineBoundsList = (SplineBounds*) calloc( newCurve->splinesNumber + 1, sizeof(SplineBounds) );

        for( size_t phaseIndex = 0; phaseIndex < newCurve->splinesNumber; phaseIndex++ )
        {
          double* splineCoeffs = (double*) newCurve->splinesList[ phaseIndex ];
          double* splineBounds = (double*) newCurve->splineBoundsList[ phaseIndex ];

          sprintf( searchPath, "phases.%u", phaseIndex );
          parser.SetBaseKey( configFileID, searchPath );

          splineBounds[ 0 ] = parser.GetRealValue( configFileID, "begin.point" );
          splineBounds[ 1 ] = parser.GetRealValue( configFileID, "end.point" );

          double splineLength = splineBounds[ 1 ] - splineBounds[ 0 ];

          double initialValue = parser.GetRealValue( configFileID, "begin.value" );
          double initialDerivative = parser.GetRealValue( configFileID, "begin.derivative" );

          double finalValue = parser.GetRealValue( configFileID, "end.value" );
          double finalDerivative = parser.GetRealValue( configFileID, "end.derivative" );

          // Spline ( x = d + c*t + b*t^2 + a*t^3 ) coefficients calculation
          splineCoeffs[ 0 ] = initialValue;
          splineCoeffs[ 1 ] = initialDerivative;
          splineCoeffs[ 2 ] = ( 3 * ( finalValue - initialValue ) - splineLength * ( 2 * initialDerivative + finalDerivative ) ) / pow( splineLength, 2 );
          splineCoeffs[ 3 ] = ( 2 * ( initialValue - finalValue ) + splineLength * ( initialDerivative + finalDerivative ) ) / pow( splineLength, 3 );

          /*DEBUG_EVENT( 1,*/
          DEBUG_PRINT( " found spline: %g %g %g %g", splineCoeffs[ 0 ], splineCoeffs[ 1 ], splineCoeffs[ 2 ], splineCoeffs[ 3 ] );
        }
      }
    
      parser.UnloadFile( configFileID );
    }
    else
    {
      DEBUG_PRINT( "data file for curve %s not found", curveName );
      Spline3Interp_UnloadCurve( newCurve );
      return NULL;
    }
  }
  else
  {
    newCurve->scaleFactor = 1.0;
    newCurve->maxScaleFactor = -1.0;
  }
  
  return newCurve;
}

static void Spline3Interp_UnloadCurve( Splined3Curve curve )
{
  DEBUG_PRINT( "unloading curve %p data", curve );
  
  if( curve != NULL )
  {
    if( curve->splinesList != NULL ) free( curve->splinesList );
    if( curve->splineBoundsList != NULL ) free( curve->splineBoundsList );
    
    free( curve );
  }
}

static void Spline3Interp_SetScale( Splined3Curve curve, double scaleFactor )
{
  if( curve == NULL ) return;
  
  curve->scaleFactor = scaleFactor;
  
  if( curve->maxScaleFactor >= 0.0 )
  {
    if( curve->scaleFactor > curve->maxScaleFactor ) curve->scaleFactor = curve->maxScaleFactor;
    else if( curve->scaleFactor < -curve->maxScaleFactor ) curve->scaleFactor = -curve->maxScaleFactor;
  }
}

static double Spline3Interp_GetValue( Splined3Curve curveData, double valuePosition )
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
