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
  double scaleFactor, offset;
  double maxAbsoluteValue;
  double totalLength;
}
Splined3CurveData;

typedef Splined3CurveData* Splined3Curve;


#define SPLINE3_INTERP_FUNCTIONS( namespace, function_init ) \
        function_init( Splined3Curve, namespace, LoadCurve, const char* ) \
        function_init( void, namespace, UnloadCurve, Splined3Curve ) \
        function_init( void, namespace, SetScale, Splined3Curve, double ) \
        function_init( void, namespace, SetOffset, Splined3Curve, double ) \
        function_init( double, namespace, GetValue, Splined3Curve, double )

INIT_NAMESPACE_INTERFACE( Spline3Interp, SPLINE3_INTERP_FUNCTIONS )


Splined3Curve Spline3Interp_LoadCurve( const char* curveName )
{
  static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  Splined3Curve newCurve = (Splined3Curve) malloc( sizeof(Splined3CurveData) );
  memset( newCurve, 0, sizeof(Splined3CurveData) );
  
  newCurve->scaleFactor = 1.0;
  newCurve->maxAbsoluteValue = -1.0;
  
  if( curveName != NULL )
  {
    FileParserOperations parser = JSONParser;
    int configFileID = parser.LoadFile( curveName );
  
    if( configFileID != -1 )
    {
      if( parser.HasKey( configFileID, "max_value" ) ) newCurve->maxAbsoluteValue = parser.GetRealValue( configFileID, "max_value" );
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
  
  return newCurve;
}

void Spline3Interp_UnloadCurve( Splined3Curve curve )
{
  DEBUG_PRINT( "unloading curve %p data", curve );
  
  if( curve != NULL )
  {
    if( curve->splinesList != NULL ) free( curve->splinesList );
    if( curve->splineBoundsList != NULL ) free( curve->splineBoundsList );
    
    free( curve );
  }
}

inline void Spline3Interp_SetScale( Splined3Curve curve, double scaleFactor )
{
  if( curve == NULL ) return;
  
  curve->scaleFactor = scaleFactor;
}

inline void Spline3Interp_SetOffset( Splined3Curve curve, double offset )
{
  if( curve == NULL ) return;
  
  curve->offset = offset;
}

double Spline3Interp_GetValue( Splined3Curve curve, double valuePosition )
{
  if( curve == NULL ) return 0.0;

  double curveValue = 0.0;
  
  valuePosition = fmod( valuePosition, curve->totalLength );
  //if( valuePosition < curve->splineBoundsList[ 0 ][ 0 ] ) valuePosition += curve->totalLength;

  if( curve->splinesNumber > 0 )
  {
    double* splineCoeffs = curve->splinesList[ 0 ];
    double relativePosition = curve->splineBoundsList[ 0 ][ 0 ];

    for( size_t splineID = 0; splineID < curve->splinesNumber; splineID++ )
    {
      if( valuePosition >= curve->splineBoundsList[ splineID ][ 0 ] )
      {
        splineCoeffs = curve->splinesList[ splineID ];

        if( valuePosition < curve->splineBoundsList[ splineID ][ 1 ] )
        {
          relativePosition = valuePosition - curve->splineBoundsList[ splineID ][ 0 ];
          break;
        }
        else
          relativePosition = curve->splineBoundsList[ splineID ][ 1 ] - curve->splineBoundsList[ splineID ][ 0 ];
      }
    }

    for( size_t coeffIndex = 0; coeffIndex < SPLINE_COEFFS_NUMBER; coeffIndex++ )
      curveValue += splineCoeffs[ coeffIndex ] * pow( relativePosition, coeffIndex );
  }
  
  curveValue = curve->scaleFactor * curveValue + curve->offset;
  if( curve->maxAbsoluteValue > 0.0 )
  {
    if( curveValue > curve->maxAbsoluteValue ) curveValue = curve->maxAbsoluteValue;
    else if( curveValue < -curve->maxAbsoluteValue ) curveValue = -curve->maxAbsoluteValue;
  }
  
  return curveValue;
}

#endif  /* SPLINE3_INTERP_H */
