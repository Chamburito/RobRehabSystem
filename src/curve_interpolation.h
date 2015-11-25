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
  double offset;
}
SegmentData;

typedef SegmentData* Segment;

typedef struct _CurveData
{
  Segment segmentsList;
  size_t segmentsNumber;
  double scaleFactor, offset;
  double maxAbsoluteValue;
}
CurveData;

typedef CurveData* Curve;


#define CURVE_INTERPOLATION_FUNCTIONS( namespace, function_init ) \
        function_init( Curve, namespace, LoadCurveFile, const char* ) \
        function_init( Curve, namespace, LoadCurveString, const char* ) \
        function_init( void, namespace, UnloadCurve, Curve ) \
        function_init( void, namespace, SetScale, Curve, double ) \
        function_init( void, namespace, SetOffset, Curve, double ) \
        function_init( double, namespace, GetValue, Curve, double, double )

INIT_NAMESPACE_INTERFACE( CurveInterpolation, CURVE_INTERPOLATION_FUNCTIONS )

void AddSpline3Segment( Curve, double*, double[ 2 ] );
Segment AddPolySegment( Curve, double*, size_t, double[ 2 ] );

Curve LoadCurveData( int configDataID )
{
  Curve newCurve = (Curve) malloc( sizeof(CurveData) );
  memset( newCurve, 0, sizeof(CurveData) );

  ParserInterface parser = ConfigParsing.GetParser();
  
  newCurve->scaleFactor = 1.0;
  newCurve->maxAbsoluteValue = parser.GetRealValue( configDataID, -1.0, "max_amplitude" );

  size_t segmentsNumber = parser.GetListSize( configDataID, "segments" );

  for( size_t segmentIndex = 0; segmentIndex < segmentsNumber; segmentIndex++ )
  {
    double curveBounds[ 2 ];
    curveBounds[ 0 ] = parser.GetRealValue( configDataID, 0.0, "segments.%lu.bounds.0", segmentIndex );
    curveBounds[ 1 ] = parser.GetRealValue( configDataID, 1.0, "segments.%lu.bounds.1", segmentIndex );

    int parametersNumber = (int) parser.GetListSize( configDataID, "segments.%lu.parameters", segmentIndex );

    double* curveParameters = (double*) calloc( parametersNumber, sizeof(double) );
    for( int parameterIndex = 0; parameterIndex < parametersNumber; parameterIndex++ )
      curveParameters[ parametersNumber - parameterIndex - 1 ] = parser.GetRealValue( configDataID, 0.0, "segments.%lu.parameters.%d", segmentIndex, parameterIndex );

    char* curveType = parser.GetStringValue( configDataID, "", "segments.%lu.type", segmentIndex );
    if( strcmp( curveType, "cubic_spline" ) == 0 && parametersNumber == SPLINE3_COEFFS_NUMBER ) 
      (void) AddSpline3Segment( newCurve, curveParameters, curveBounds );
    else if( strcmp( curveType, "polynomial" ) == 0 ) 
      (void) AddPolySegment( newCurve, curveParameters, parametersNumber, curveBounds );

    free( curveParameters );
  }

  newCurve->segmentsNumber = segmentsNumber;

  parser.UnloadData( configDataID );
  
  return newCurve;
}

Curve CurveInterpolation_LoadCurveFile( const char* curveName )
{
  int configFileID = ConfigParsing.LoadConfigFile( curveName );
  return LoadCurveData( configFileID );
}

Curve CurveInterpolation_LoadCurveString( const char* curveString )
{
  int configDataID = ConfigParsing.LoadConfigString( curveString );
  return LoadCurveData( configDataID );
}

void CurveInterpolation_UnloadCurve( Curve curve )
{
  DEBUG_PRINT( "unloading curve %p data", curve );
  
  if( curve != NULL )
  {
    if( curve->segmentsList != NULL )
    {
      //for( size_t segmentIndex = 0; segmentIndex < curve->segmentsNumber; segmentIndex++ )
      //  free( curve->segmentsList[ curve->segmentsNumber ].coeffs );
      free( curve->segmentsList );
    }
    
    free( curve );
  }
}

void AddSpline3Segment( Curve curve, double* splineValues, double splineBounds[ 2 ] )
{
  double splineLength = splineBounds[ 1 ] - splineBounds[ 0 ];
  
  double initialValue = splineValues[ 3 ];
  double initialDerivative = splineValues[ 2 ];
  double finalValue = splineValues[ 1 ];
  double finalDerivative = splineValues[ 0 ];
  
  // Curve ( x = d + c*t + b*t^2 + a*t^3 ) coefficients calculation
  splineValues[ 0 ] = initialValue;
  splineValues[ 1 ] = initialDerivative;
  splineValues[ 2 ] = ( 3 * ( finalValue - initialValue ) - splineLength * ( 2 * initialDerivative + finalDerivative ) ) / pow( splineLength, 2 );
  splineValues[ 3 ] = ( 2 * ( initialValue - finalValue ) + splineLength * ( initialDerivative + finalDerivative ) ) / pow( splineLength, 3 );
  
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( " adding spline: %g %g %g %g", splineValues[ 0 ], splineValues[ 1 ], splineValues[ 2 ], splineValues[ 3 ] );
  
  Segment newSegment = AddPolySegment( curve, (double*) splineValues, SPLINE3_COEFFS_NUMBER, splineBounds );
  
  if( newSegment != NULL ) newSegment->offset = newSegment->bounds[ 0 ];
}

Segment AddPolySegment( Curve curve, double* polyCoeffs, size_t coeffsNumber, double polyBounds[ 2 ] )
{
  if( curve == NULL ) return NULL;
  
  if( coeffsNumber == 0 ) return NULL;
  
  curve->segmentsList = (Segment) realloc( curve->segmentsList, ( curve->segmentsNumber + 1 ) * sizeof(SegmentData) );
  
  Segment newSegment = &(curve->segmentsList[ curve->segmentsNumber++ ]);
  
  newSegment->coeffs = (double*) calloc( coeffsNumber, sizeof(double) );
  newSegment->coeffsNumber = coeffsNumber;

  newSegment->bounds[ 0 ] = polyBounds[ 0 ];
  newSegment->bounds[ 1 ] = polyBounds[ 1 ];
  
  memcpy( newSegment->coeffs, polyCoeffs, coeffsNumber * sizeof(double) );
  
  return newSegment;
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
  double curveValue = defaultValue;
  
  if( curve != NULL )
  {
    if( curve->segmentsNumber > 0 )
    {
      for( size_t segmentIndex = 0; segmentIndex < curve->segmentsNumber; segmentIndex++ )
      {
        double* segmentBounds = (double*) curve->segmentsList[ segmentIndex ].bounds;
        if( valuePosition >= segmentBounds[ 0 ] && valuePosition < segmentBounds[ 1 ] )
        {
          double* curveCoeffs = curve->segmentsList[ segmentIndex ].coeffs;
          size_t coeffsNumber = curve->segmentsList[ segmentIndex ].coeffsNumber;
          double positionOffset = curve->segmentsList[ segmentIndex ].offset;
          
          curveValue = 0.0;
          double relativePosition = valuePosition - positionOffset;
          for( size_t coeffIndex = 0; coeffIndex < coeffsNumber; coeffIndex++ )
            curveValue += curveCoeffs[ coeffIndex ] * pow( relativePosition, coeffIndex );
          
          break;
        }
      }
    }
    
    curveValue = curve->scaleFactor * curveValue + curve->offset;
    if( curve->maxAbsoluteValue > 0.0 )
    {
      if( curveValue > curve->maxAbsoluteValue ) curveValue = curve->maxAbsoluteValue;
      else if( curveValue < -curve->maxAbsoluteValue ) curveValue = -curve->maxAbsoluteValue;
    }
  }
  
  return curveValue;
}

#endif  /* CURVE_INTERPOLATION_H */
