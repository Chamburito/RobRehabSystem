#ifndef KALMAN_FILTERS_H
#define KALMAN_FILTERS_H

#include "interfaces.h"


typedef struct _KalmanFilterData KalmanFilterData;
typedef KalmanFilterData* KalmanFilter;

#define KALMAN_INTERFACE( namespace, function_init ) \
        function_init( KalmanFilter, namespace, CreateFilter, size_t ) \
        function_init( void, namespace, DiscardFilter, KalmanFilter ) \
        function_init( void, namespace, AddInput, KalmanFilter, size_t ) \
        function_init( void, namespace, SetInput, KalmanFilter, size_t, double ) \
        function_init( void, namespace, SetVariablesCoupling, KalmanFilter, size_t, size_t, double ) \
        function_init( void, namespace, SetInputMaxError, KalmanFilter, size_t, double ) \
        function_init( double*, namespace, Predict, KalmanFilter, double* ) \
        function_init( double*, namespace, Update, KalmanFilter, double*, double* ) \
        function_init( void, namespace, Reset, KalmanFilter )

DECLARE_NAMESPACE_INTERFACE( Kalman, KALMAN_INTERFACE )


#endif  // KALMAN_FILTERS_H
