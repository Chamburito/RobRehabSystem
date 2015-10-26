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
  Segment segmentsList;
  size_t segmentsNumber;
  double scaleFactor, offset;
  double maxAbsoluteValue;
  double totalLength;
}
CurveData;

typedef CurveData* Curve;


#define CURVE_INTERPOLATION_FUNCTIONS( namespace, function_init ) \
        function_init( Curve, namespace, LoadCurveFile, const char* ) \
        function_init( Curve, namespace, LoadCurveString, const char* ) \
        function_init( void, namespace, UnloadCurve, Curve ) \
        function_init( void, namespace, AddSpline3Segment, Curve, double[ SPLINE3_COEFFS_NUMBER ], double[ 2 ] ) \
        function_init( void, namespace, AddPolySegment, Curve, double*, size_t, double[ 2 ] ) \
        function_init( void, namespace, SetScale, Curve, double ) \
        function_init( void, namespace, SetOffset, Curve, double ) \
        function_init( double, namespace, GetValue, Curve, double, double )

INIT_NAMESPACE_INTERFACE( CurveInterpolation, CURVE_INTERPOLATION_FUNCTIONS )


Curve LoadCurveData( int configDataID )
{
  static char searchPath[ PARSER_MAX_FILE_PATH_LENGTH ];
  
  if( configDataID == -1 ) return NULL;
  
  Curve newCurve = (Curve) malloc( sizeof(CurveData) );
  memset( newCurve, 0, sizeof(CurveData) );
  
  newCurve->scaleFactor = 1.0;
  
  newCurve->maxAbsoluteValue = ConfigParser.GetRealValue( configDataID, "max_value", -1.0 );
  
  if( (newCurve->segmentsNumber = (size_t) ConfigParser.GetListSize( configDataID, "segments" )) > 0 )
  {
    for( size_t segmentIndex = 0; segmentIndex < newCurve->segmentsNumber; segmentIndex++ )
    {
      sprintf( searchPath, "segments.%lu", segmentIndex );
      ConfigParser.SetBaseKey( configDataID, searchPath );
      
      double curveBounds[ 2 ];
      curveBounds[ 0 ] = ConfigParser.GetRealValue( configDataID, "bounds.0", 0.0 );
      curveBounds[ 1 ] = ConfigParser.GetRealValue( configDataID, "bounds.1", 1.0 );
      
      char* curveType = ConfigParser.GetStringValue( configDataID, "type", NULL );
      if( strcmp( curveType, "cubic_spline" ) == 0  )
      {
        double splineValues[ SPLINE3_COEFFS_NUMBER ];
        splineValues[ 0 ] = ConfigParser.GetRealValue( configDataID, "begin.value", 0.0 );
        splineValues[ 1 ] = ConfigParser.GetRealValue( configDataID, "begin.derivative", 0.0 );
        splineValues[ 2 ] = ConfigParser.GetRealValue( configDataID, "end.value", 0.0 );
        splineValues[ 3 ] = ConfigParser.GetRealValue( configDataID, "end.derivative", 0.0 );
        
        CurveInterpolation_AddSpline3Segment( newCurve, splineValues, curveBounds );
      }
      else if( strcmp( curveType, "polynomial" ) == 0  )
      {
        int coeffsNumber = (int) ConfigParser.GetListSize( configDataID, "coeffs" );
        
        double* curveCoeffs = (double*) calloc( coeffsNumber, sizeof(double) );
        for( int coeffIndex = 0; coeffIndex < coeffsNumber; coeffIndex++ )
        {
          sprintf( searchPath, "coeffs.%d", coeffIndex );
          curveCoeffs[ coeffsNumber - coeffIndex - 1 ] = ConfigParser.GetRealValue( configDataID, searchPath, 0.0 ); 
        }
        free( curveCoeffs );
        
        CurveInterpolation_AddPolySegment( newCurve, curveCoeffs, coeffsNumber, curveBounds );
      }
    }
  }
  
  ConfigParser.UnloadData( configDataID );
  
  return newCurve;
}

Curve CurveInterpolation_LoadCurveString( const char* curveName )
{
  int configFileID = ConfigParser.LoadFileData( curveName );
  return LoadCurveData( configFileID );
}

Curve CurveInterpolation_LoadCurveFile( const char* curveString )
{
  int configDataID = ConfigParser.LoadStringData( curveString );
  return LoadCurveData( configDataID );
}

void CurveInterpolation_UnloadCurve( Curve curve )
{
  DEBUG_PRINT( "unloading curve %p data", curve );
  
  if( curve != NULL )
  {
    if( curve->segmentsList != NULL )
    {
      for( size_t segmentIndex = 0; segmentIndex < curve->segmentsNumber; segmentIndex++ )
        free( curve->segmentsList[ curve->segmentsNumber ].coeffs );
      free( curve->segmentsList );
    }
    
    free( curve );
  }
}

void CurveInterpolation_AddSpline3Segment( Curve curve, double splineValues[ SPLINE3_COEFFS_NUMBER ], double splineBounds[ 2 ] )
{
  double splineLength = splineBounds[ 1 ] - splineBounds[ 0 ];
  
  double initialValue = splineValues[ 0 ];
  double initialDerivative = splineValues[ 1 ];
  double finalValue = splineValues[ 2 ];
  double finalDerivative = splineValues[ 3 ];
  
  // Curve ( x = d + c*t + b*t^2 + a*t^3 ) coefficients calculation
  splineValues[ 2 ] = ( 3 * ( finalValue - initialValue ) - splineLength * ( 2 * initialDerivative + finalDerivative ) ) / pow( splineLength, 2 );
  splineValues[ 3 ] = ( 2 * ( initialValue - finalValue ) + splineLength * ( initialDerivative + finalDerivative ) ) / pow( splineLength, 3 );
  
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( " adding spline: %g %g %g %g", splineValues[ 0 ], splineValues[ 1 ], splineValues[ 2 ], splineValues[ 3 ] );
  
  CurveInterpolation_AddPolySegment( curve, (double*) splineValues, SPLINE3_COEFFS_NUMBER, splineBounds );
}

void CurveInterpolation_AddPolySegment( Curve curve, double* polyCoeffs, size_t coeffsNumber, double polyBounds[ 2 ] )
{
  if( curve == NULL ) return;
  
  if( coeffsNumber == 0 ) return;
  
  curve->segmentsList = (Segment) realloc( curve->segmentsList, ( curve->segmentsNumber + 1 ) * sizeof(SegmentData) );
  curve->segmentsList[ curve->segmentsNumber ].coeffs = (double*) calloc( coeffsNumber, sizeof(double) );

  curve->segmentsList[ curve->segmentsNumber ].bounds[ 0 ] = polyBounds[ 0 ];
  curve->segmentsList[ curve->segmentsNumber ].bounds[ 1 ] = polyBounds[ 1 ];
  
  memcpy( curve->segmentsList[ curve->segmentsNumber ].coeffs, polyCoeffs, coeffsNumber * sizeof(double) );
  
  curve->segmentsNumber++;
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
    valuePosition = fmod( valuePosition, curve->totalLength );
    //if( valuePosition < curve->curveBoundsList[ 0 ][ 0 ] ) valuePosition += curve->totalLength;

    if( curve->segmentsNumber > 0 )
    {
      for( size_t segmentIndex = 0; segmentIndex < curve->segmentsNumber; segmentIndex++ )
      {
        double* segmentBounds = (double*) curve->segmentsList[ segmentIndex ].bounds;
        if( valuePosition >= segmentBounds[ 0 ] && valuePosition < segmentBounds[ 1 ] )
        {
          double* curveCoeffs = curve->segmentsList[ segmentIndex ].coeffs;
          size_t coeffsNumber = curve->segmentsList[ segmentIndex ].coeffsNumber;
          
          curveValue = 0.0;
          double relativePosition = valuePosition - segmentBounds[ 0 ];
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
