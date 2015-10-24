#ifndef CURVE_INTERPOLATION_H
#define CURVE_INTERPOLATION_H

#include <math.h>

#include "config_parser.h"

const size_t CURVE_NAME_MAX_LENGTH = 32;

const size_t SPLINE3_COEFFS_NUMBER = 4;

typedef struct _SegmentData
{
  double* coeffs;
  size_t coeffsNumber;
  double bounds[ 2 ];
}
SegmentData;

typedef SegmentData* Segment;

typedef struct _CurveData
{
  Segment* segmentsList;
  size_t segmentsNumber;
  double scaleFactor, offset;
  double maxAbsoluteValue;
  double totalLength;
}
CurveData;

typedef CurveData* Curve;


#define CURVE_INTERPOLATION_FUNCTIONS( namespace, function_init ) \
        function_init( Curve, namespace, LoadCurve, const char* ) \
        function_init( void, namespace, UnloadCurve, Curve ) \
        function_init( void, namespace, AddSpline3Segment, Curve, double[ SPLINE3_COEFFS_NUMBER ], double[ 2 ] ) \
        function_init( void, namespace, AddPolySegment, Curve, double*, size_t, double[ 2 ] ) \
        function_init( void, namespace, SetScale, Curve, double ) \
        function_init( void, namespace, SetOffset, Curve, double ) \
        function_init( double, namespace, GetValue, Curve, double )

INIT_NAMESPACE_INTERFACE( CurveInterpolation, CURVE_INTERPOLATION_FUNCTIONS )


Curve CurveInterpolation_LoadCurve( const char* curveName )
{
  static char searchPath[ PARSER_MAX_FILE_PATH_LENGTH ];
  
  Curve newCurve = (Curve) malloc( sizeof(CurveData) );
  memset( newCurve, 0, sizeof(CurveData) );
  
  newCurve->scaleFactor = 1.0;
  newCurve->maxAbsoluteValue = -1.0;
  
  if( curveName != NULL )
  {
    int configFileID = ConfigParser.LoadFile( curveName );
    if( configFileID != -1 )
    {
      if( ConfigParser.HasKey( configFileID, "max_value" ) ) newCurve->maxAbsoluteValue = ConfigParser.GetRealValue( configFileID, "max_value" );
      newCurve->totalLength = ConfigParser.GetRealValue( configFileID, "total_length" );

      if( (newCurve->curvesNumber = (size_t) ConfigParser.GetListSize( configFileID, "phases" )) > 0 )
      {
        newCurve->curvesList = (SplineCoeffs*) calloc( newCurve->curvesNumber + 1, sizeof(SplineCoeffs) );
        newCurve->curveBoundsList = (CurveBounds*) calloc( newCurve->curvesNumber + 1, sizeof(CurveBounds) );

        for( size_t phaseIndex = 0; phaseIndex < newCurve->curvesNumber; phaseIndex++ )
        {
          double* curveCoeffs = newCurve->curvesList[ phaseIndex ];
          double* curveBounds = (double*) newCurve->curveBoundsList[ phaseIndex ];

          sprintf( searchPath, "phases.%u", phaseIndex );
          ConfigParser.SetBaseKey( configFileID, searchPath );

          curveBounds[ 0 ] = ConfigParser.GetRealValue( configFileID, "begin.point" );
          curveBounds[ 1 ] = ConfigParser.GetRealValue( configFileID, "end.point" );

          double curveLength = curveBounds[ 1 ] - curveBounds[ 0 ];

          double initialValue = ConfigParser.GetRealValue( configFileID, "begin.value" );
          double initialDerivative = ConfigParser.GetRealValue( configFileID, "begin.derivative" );

          double finalValue = ConfigParser.GetRealValue( configFileID, "end.value" );
          double finalDerivative = ConfigParser.GetRealValue( configFileID, "end.derivative" );

          // Curve ( x = d + c*t + b*t^2 + a*t^3 ) coefficients calculation
          curveCoeffs[ 0 ] = initialValue;
          curveCoeffs[ 1 ] = initialDerivative;
          curveCoeffs[ 2 ] = ( 3 * ( finalValue - initialValue ) - curveLength * ( 2 * initialDerivative + finalDerivative ) ) / pow( curveLength, 2 );
          curveCoeffs[ 3 ] = ( 2 * ( initialValue - finalValue ) + curveLength * ( initialDerivative + finalDerivative ) ) / pow( curveLength, 3 );
        }
      }
    
      ConfigParser.UnloadFile( configFileID );
    }
    else
    {
      DEBUG_PRINT( "data file for curve %s not found", curveName );
      CurveInterpolation_UnloadCurve( newCurve );
      return NULL;
    }
  }
  
  return newCurve;
}

void CurveInterpolation_UnloadCurve( Curve curve )
{
  DEBUG_PRINT( "unloading curve %p data", curve );
  
  if( curve != NULL )
  {
    if( curve->curvesList != NULL ) free( curve->curvesList );
    if( curve->curveBoundsList != NULL ) free( curve->curveBoundsList );
    
    free( curve );
  }
}

void CurveInterpolation_AddSpline3Segment( Curve curve, double splineData[ SPLINE3_COEFFS_NUMBER ], double splineBounds[ 2 ] )
{
  double splineLength = splineBounds[ 1 ] - splineBounds[ 0 ];
  
  double initialValue = splineData[ 0 ];
  double initialDerivative = splineData[ 1 ];
  double finalValue = splineData[ 2 ];
  double finalDerivative = splineData[ 3 ];
  
  // Curve ( x = d + c*t + b*t^2 + a*t^3 ) coefficients calculation
  splineData[ 2 ] = ( 3 * ( finalValue - initialValue ) - splineLength * ( 2 * initialDerivative + finalDerivative ) ) / pow( splineLength, 2 );
  splineData[ 3 ] = ( 2 * ( initialValue - finalValue ) + splineLength * ( initialDerivative + finalDerivative ) ) / pow( splineLength, 3 );
  
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( " found spline: %g %g %g %g", splineData[ 0 ], splineData[ 1 ], splineData[ 2 ], splineData[ 3 ] );
  
  CurveInterpolation_AddPolySegment( curve, (double*) splineData, SPLINE3_COEFFS_NUMBER, splineBounds );
}

void CurveInterpolation_AddPolySegment( Curve curve, double* polyCoeffs, size_t coeffsNumber, double polyBounds[ 2 ] )
{
  if( curve == NULL ) return;
  
  if( polyLength == 0 ) return;
  
  curve->curvesList = (double**) realloc( curve->curvesList, ( curve->curvesNumber + 1 ) * sizeof(double*) );
  curve->curvesList[ curve->curvesNumber ] = (double*) realloc( curve->curvesList[ curve->curvesNumber ], coeffsNumber * sizeof(double) );
  curve->curveBoundsList = (CurveBounds*) realloc( curve->curveBoundsList, ( curve->curvesNumber + 1 ) * sizeof(CurveBounds) );

  curve->curveBoundsList[ curve->curvesNumber ][ 0 ] = polyBounds[ 0 ];
  curve->curveBoundsList[ curve->curvesNumber ][ 1 ] = polyBounds[ 1 ];
  double polyLength = polyBounds[ 1 ] - polyBounds[ 0 ];
  
  memcpy( curve->curvesList[ curve->curvesNumber ], polyCoeffs, coeffsNumber * sizeof(double) );
  
  curve->curvesNumber++;
}

inline void CurveInterpolation_SetScale( Curve curve, double scaleFactor )
{
  if( curve == NULL ) return;
  
  curve->scaleFactor = scaleFactor;
}

inline void CurveInterpolation_SetOffset( Curve curve, double offset )
{
  if( curve == NULL ) return;
  
  curve->offset = offset;
}

inline void CurveInterpolation_SetMaxAmplitude( Curve curve, double maxAmplitude )
{
  if( curve == NULL ) return;
  
  curve->maxAbsoluteValue = maxAmplitude;
}

double CurveInterpolation_GetValue( Curve curve, double valuePosition, double defaultValue )
{
  if( curve == NULL ) return 0.0;

  double curveValue = 0.0;
  
  valuePosition = fmod( valuePosition, curve->totalLength );
  //if( valuePosition < curve->curveBoundsList[ 0 ][ 0 ] ) valuePosition += curve->totalLength;

  if( curve->curvesNumber > 0 )
  {
    double* curveCoeffs = curve->curvesList[ 0 ];
    double relativePosition = curve->curveBoundsList[ 0 ][ 0 ];

    for( size_t curveID = 0; curveID < curve->curvesNumber; curveID++ )
    {
      if( valuePosition >= curve->curveBoundsList[ curveID ][ 0 ] )
      {
        curveCoeffs = curve->curvesList[ curveID ];

        if( valuePosition < curve->curveBoundsList[ curveID ][ 1 ] )
        {
          relativePosition = valuePosition - curve->curveBoundsList[ curveID ][ 0 ];
          break;
        }
        else
          relativePosition = curve->curveBoundsList[ curveID ][ 1 ] - curve->curveBoundsList[ curveID ][ 0 ];
      }
    }

    for( size_t coeffIndex = 0; coeffIndex < SPLINE3_COEFFS_NUMBER; coeffIndex++ )
      curveValue += curveCoeffs[ coeffIndex ] * pow( relativePosition, coeffIndex );
  }
  
  curveValue = curve->scaleFactor * curveValue + curve->offset;
  if( curve->maxAbsoluteValue > 0.0 )
  {
    if( curveValue > curve->maxAbsoluteValue ) curveValue = curve->maxAbsoluteValue;
    else if( curveValue < -curve->maxAbsoluteValue ) curveValue = -curve->maxAbsoluteValue;
  }
  
  return curveValue;
}

#endif  /* CURVE_INTERPOLATION_H */
