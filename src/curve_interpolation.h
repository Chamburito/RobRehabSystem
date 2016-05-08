#ifndef CURVE_INTERPOLATION_H
#define CURVE_INTERPOLATION_H

#include "namespaces.h"


typedef struct _CurveData CurveData;
typedef CurveData* Curve;

#define CURVE_INTERPOLATION_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( Curve, Namespace, LoadCurveFile, const char* ) \
        INIT_FUNCTION( Curve, Namespace, LoadCurveString, const char* ) \
        INIT_FUNCTION( void, Namespace, UnloadCurve, Curve ) \
        INIT_FUNCTION( void, Namespace, SetScale, Curve, double ) \
        INIT_FUNCTION( void, Namespace, SetOffset, Curve, double ) \
        INIT_FUNCTION( double, Namespace, GetValue, Curve, double, double )

DECLARE_NAMESPACE_INTERFACE( CurveInterpolation, CURVE_INTERPOLATION_INTERFACE )


#endif  /* CURVE_INTERPOLATION_H */
