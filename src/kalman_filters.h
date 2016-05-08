#ifndef KALMAN_FILTERS_H
#define KALMAN_FILTERS_H

#include "namespaces.h"


typedef struct _KalmanFilterData KalmanFilterData;
typedef KalmanFilterData* KalmanFilter;

#define KALMAN_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( KalmanFilter, Namespace, CreateFilter, size_t ) \
        INIT_FUNCTION( void, Namespace, DiscardFilter, KalmanFilter ) \
        INIT_FUNCTION( void, Namespace, AddInput, KalmanFilter, size_t ) \
        INIT_FUNCTION( void, Namespace, SetInput, KalmanFilter, size_t, double ) \
        INIT_FUNCTION( void, Namespace, SetVariablesCoupling, KalmanFilter, size_t, size_t, double ) \
        INIT_FUNCTION( void, Namespace, SetInputMaxError, KalmanFilter, size_t, double ) \
        INIT_FUNCTION( double*, Namespace, Predict, KalmanFilter, double* ) \
        INIT_FUNCTION( double*, Namespace, Update, KalmanFilter, double*, double* ) \
        INIT_FUNCTION( void, Namespace, Reset, KalmanFilter )

DECLARE_NAMESPACE_INTERFACE( Kalman, KALMAN_INTERFACE )


#endif  // KALMAN_FILTERS_H
