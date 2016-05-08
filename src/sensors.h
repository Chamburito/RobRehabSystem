#ifndef SENSORS_H
#define SENSORS_H

#include "interfaces.h"

#include "signal_processing.h"


typedef struct _SensorData SensorData;    
typedef SensorData* Sensor;

#define SENSOR_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( Sensor, Namespace, Init, const char* ) \
        INIT_FUNCTION( void, Namespace, End, Sensor ) \
        INIT_FUNCTION( double, Namespace, Update, Sensor ) \
        INIT_FUNCTION( bool, Namespace, HasError, Sensor ) \
        INIT_FUNCTION( void, Namespace, Reset, Sensor ) \
        INIT_FUNCTION( void, Namespace, SetState, Sensor, enum SignalProcessingPhase ) \
        INIT_FUNCTION( SignalProcessor, Namespace, GetSignalProcessor, Sensor )

DECLARE_NAMESPACE_INTERFACE( Sensors, SENSOR_INTERFACE )


#endif // SENSORS_H
