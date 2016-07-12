#ifndef DATA_LOGGING_H
#define DATA_LOGGING_H

#include <stdbool.h>

#include "namespaces.h"

#define LOG_FILE_PATH_MAX_LEN 256
#define DATA_LOG_INVALID_ID -1

#define DATA_LOG_MAX_PRECISION 15

typedef struct _LogData LogData;
typedef LogData* Log;

#define DATA_LOGGING_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( int, Namespace, InitLog, const char*, size_t, size_t ) \
        INIT_FUNCTION( void, Namespace, EndLog, int ) \
        INIT_FUNCTION( char*, Namespace, GetBaseDirectory, char* ) \
        INIT_FUNCTION( void, Namespace, SetBaseDirectory, const char* ) \
        INIT_FUNCTION( void, Namespace, SaveData, int, double*, size_t ) \
        INIT_FUNCTION( void, Namespace, RegisterValues, int, size_t, ... ) \
        INIT_FUNCTION( void, Namespace, RegisterList, int, size_t, double* ) \
        INIT_FUNCTION( void, Namespace, SetDataPrecision, int, size_t )

DECLARE_NAMESPACE_INTERFACE( DataLogging, DATA_LOGGING_INTERFACE )


#endif /* DATA_LOGGING_H */
