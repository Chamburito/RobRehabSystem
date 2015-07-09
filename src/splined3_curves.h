#ifndef SPLINED3_CURVES_H
#define SPLINED3_CURVES_H

#include <math.h>

const size_t CURVE_NAME_MAX_LENGTH = 16;

const size_t SPLINE_COEFFS_NUMBER = 4;
typedef double SplineCoeffs[ CURVE_COEFFS_NUMBER ];
typedef double SplineBounds[ 2 ];

typedef struct _Splined3CurveData
{
  SplineCoeffs* splinesList;
  SplineBounds* splineTimes;
  size_t splinesNumber;
}
Splined3CurveData;

Splined3CurveData* Splined3Curves_Load( const char* curveName )
{
  char readBuffer[ CURVE_NAME_MAX_LENGTH ];
  
  Splined3CurveData* newCurveData = NULL;
  
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
          newCurveData = (Splined3CurveData*) malloc( sizeof(Splined3CurveData) );
        
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

void Splined3Curves_Unload( Splined3CurveData* curveData )
{
  if( curveData != NULL )
  {
    free( curveData->splinesList );
    free( curveData->splineTimes );
  }
}

double Splined3Curves_GetValue( Splined3CurveData* curveData, double valuePoint, double defaultValue )
{
  if( curveData == NULL ) return defaultValue;
  
  SplineCoeffs* splineCoeffs = NULL;
  for( size_t splineID = 0; splineID < splinesNumber; splineID++ )
  {
    if( valuePoint > curveData->splineTimes[ splineID ][ 0 ] && valuePoint <= curveData->splineTimes[ splineID ][ 1 ] )
    {
      double relativePoint = valuePoint - curveData->splineTimes[ splineID ][ 0 ];
      
      splineCoeffs = &(curveData->splinesList[ splineID ]);
      
      double curveValue = splineCoeffs[ 0 ]; 
      for( size_t coeffIndex = 1; coeffIndex < SPLINE_COEFFS_NUMBER; coeffIndex++ ) 
        curveValue += splineCoeffs[ coeffIndex ] * pow( relativePoint, coeffIndex );
      
      return curveValue;
    }
  }
  
  return defaultValue;
}

#endif  /* SPLINED3_CURVES_H */
