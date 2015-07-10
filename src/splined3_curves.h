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
        
          DEBUG_PRINT( "found curve %s", readBuffer );
        
          newCurveData->splinesList = NULL;
          newCurveData->splineTimes = NULL;
          newCurveData->splinesNumber = 0;
        }
      }
      
      if( newCurveData == NULL ) continue;
      
      if( strcmp( readBuffer, "phase:" ) == 0 )
      {
        newCurveData->splinesList = (SplineCoeffs*) realloc( newCurveData->splinesList, ( newCurveData->splinesNumber + 1 ) * sizeof(SplineCoeffs) );
        newCurveData->splineTimes = (SplineBounds*) realloc( newCurveData->splineTimes, ( newCurveData->splinesNumber + 1 ) * sizeof(SplineBounds) );
        
        SplineCoeffs* splineCoeffs = &(newCurveData->splinesList[ newCurveData->splinesNumber ]);
        SplineBounds* splineBounds = &(newCurveData->splineTimes[ newCurveData->splinesNumber ]);
        
        fscanf( configFile, "%lf %lf", &(splineBounds[ 0 ]), &(splineBounds[ 1 ]) );
        
        for( size_t coeffIndex = 0; coeffIndex < SPLINE_COEFFS_NUMBER; coeffIndex++ )
          fscanf( configFile, "%lf", &(splineCoeffs[ coeffIndex ]) );
        
        DEBUG_PRINT( " found spline: %g %g %g %g", splineCoeffs[ 0 ], splineCoeffs[ 1 ], splineCoeffs[ 2 ], splineCoeffs[ 3 ] );
        
        newCurveData->splinesNumber++;
      }
      else if( strcmp( readBuffer, "END_CURVE" ) == 0 ) 
      {
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
  }
}

double Spline3Interp_GetValue( Splined3Curve* curveData, double valuePoint )
{
  if( curveData == NULL ) return 1.0;
  
  SplineCoeffs* splineCoeffs = NULL;
  double relativePoint = 0.0;
  
  for( size_t splineID = 0; splineID < curveData->splinesList[ splineID ].splinesNumber; splineID++ )
  {
    if( valuePoint > curveData->splineTimes[ splineID ][ 0 ] )
    {
      splineCoeffs = &(curveData->splinesList[ splineID ]);
      
      if( valuePoint <= curveData->splineTimes[ splineID ][ 1 ] )
        relativePoint = valuePoint - curveData->splineTimes[ splineID ][ 0 ];
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
