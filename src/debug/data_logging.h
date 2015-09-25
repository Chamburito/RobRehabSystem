#ifndef DATA_LOGGING_H
#define DATA_LOGGING_H

#ifdef _CVI_
  #include <ansi_c.h>
#else
  #include <stdio.h>
  #include <stdlib.h>
  #include <stdarg.h>
  #include <time.h>
#endif

#include "klib/khash.h"

#include "async_debug.h"

typedef struct _LogData
{
  FILE* file;
  size_t valuesNumber;
  double* memoryBuffer;
  size_t memoryBufferLength;
  size_t memoryValuesCount;
  size_t dataPrecision;
}
LogData;

typedef LogData* Log;

KHASH_MAP_INIT_INT( LogInt, Log );
static khash_t( LogInt )* logsList = NULL;

/*static int DataLogging_InitLog( const char*, size_t );
static void DataLogging_EndLog( int ); 
static void DataLogging_SaveData( int , double*, size_t );
static void DataLogging_RegisterValues( int, size_t, ... );

const struct
{
  int (*InitLog)( const char*, size_t );
  void (*EndLog)( int ); 
  void (*SaveData)( int , double*, size_t );
  void (*RegisterValues)( int, size_t, ... );
}
DataLogging = { .InitLog = DataLogging_InitLog, .EndLog = DataLogging_EndLog, .SaveData = DataLogging_SaveData, .RegisterValues = DataLogging_RegisterValues };*/

#define NAMESPACE DataLogging

#define NAMESPACE_FUNCTIONS( namespace ) \
        NAMESPACE_FUNCTION( int, namespace, InitLog, const char*, size_t, size_t ) \
        NAMESPACE_FUNCTION( void, namespace, EndLog, int ) \
        NAMESPACE_FUNCTION( void, namespace, SaveData, int, double*, size_t ) \
        NAMESPACE_FUNCTION( void, namespace, RegisterValues, int, size_t, ... ) \
        NAMESPACE_FUNCTION( void, namespace, RegisterList, int, size_t, double* ) \
        NAMESPACE_FUNCTION( void, namespace, SetDataPrecision, int, size_t )

#define NAMESPACE_FUNCTION( rvalue, namespace, name, ... ) static rvalue namespace##_##name( __VA_ARGS__ );
NAMESPACE_FUNCTIONS( NAMESPACE )
#undef NAMESPACE_FUNCTION

#define NAMESPACE_FUNCTION( rvalue, namespace, name, ... ) rvalue (*name)( __VA_ARGS__ );
const struct { NAMESPACE_FUNCTIONS( NAMESPACE ) }
#undef NAMESPACE_FUNCTION
#define NAMESPACE_FUNCTION( rvalue, namespace, name, ... ) .name = namespace##_##name,
NAMESPACE = { NAMESPACE_FUNCTIONS( NAMESPACE ) };
#undef NAMESPACE_FUNCTION

#undef NAMESPACE_FUNCTIONS
#undef NAMESPACE

const size_t LOG_FILE_PATH_MAX_LEN = 256;
static int DataLogging_InitLog( const char* logFilePath, size_t logValuesNumber, size_t memoryBufferLength )
{
  static char logFilePathExt[ LOG_FILE_PATH_MAX_LEN ];
  
  if( logsList == NULL ) logsList = kh_init( LogInt );
  
  int logKey = (int) kh_str_hash_func( logFilePath );
  
  sprintf( logFilePathExt, "../logs/%s-%u.log", logFilePath, time( NULL ) );
  
  int insertionStatus;
  khint_t newLogID = kh_put( LogInt, logsList, logKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( logsList, newLogID ) = (Log) malloc( sizeof(LogData) );
    
    Log newLog = kh_value( logsList, newLogID );
    
    DEBUG_PRINT( "trying to open file %s", logFilePathExt );
    if( (newLog->file = fopen( logFilePathExt, "w+" )) == NULL )
    {
      perror( "error opening file" );
      DataLogging_EndLog( (int) newLogID );
      return -1;
    }
    
    newLog->valuesNumber = logValuesNumber;
    newLog->memoryBuffer = (double*) calloc( memoryBufferLength, sizeof(double) );
    newLog->memoryBufferLength = memoryBufferLength;
    newLog->memoryValuesCount = 0;
  }
  
  return (int) newLogID;
}

static void DataLogging_EndLog( int logID )
{
  if( !kh_exist( logsList, (khint_t) logID ) ) return;
  
  Log log = kh_value( logsList, (khint_t) logID );
  
  DataLogging_SaveData( logID, (double*) log->memoryBuffer, log->memoryValuesCount );
  
  if( log->file != NULL ) fclose( log->file );
  
  free( log );
  
  kh_del( LogInt, logsList, (khint_t) logID );
  
  if( kh_size( logsList ) == 0 )
  {
    kh_destroy( LogInt, logsList );
    
    logsList = NULL;
  }
}

static void DataLogging_SaveData( int logID, double* dataList, size_t dataListSize )
{
  if( !kh_exist( logsList, (khint_t) logID ) ) return;
  
  Log log = kh_value( logsList, (khint_t) logID );
  
  size_t linesNumber = dataListSize / log->valuesNumber;
  for( size_t lineIndex = 0; lineIndex < linesNumber; lineIndex++ )
  {
    for( size_t lineValueIndex = 0; lineValueIndex < log->valuesNumber; lineValueIndex++ )
      fprintf( log->file, "%.*lf\t", log->dataPrecision, dataList[ lineIndex * log->valuesNumber + lineValueIndex ] );
    fprintf( log->file, "\n" );
  }
}

static void DataLogging_RegisterValues( int logID, size_t valuesNumber, ... )
{
  if( !kh_exist( logsList, (khint_t) logID ) ) return;
  
  Log log = kh_value( logsList, (khint_t) logID );
  
  va_list logValues;
  
  va_start( logValues, valuesNumber );

  if( log->memoryValuesCount + log->valuesNumber >= log->memoryBufferLength )
  {
    DataLogging_SaveData( logID, log->memoryBuffer, log->memoryValuesCount );
    log->memoryValuesCount = 0;
  }

  for( size_t valueLineIndex = 0; valueLineIndex < valuesNumber; valueLineIndex++ )
    log->memoryBuffer[ log->memoryValuesCount + valueLineIndex ] = va_arg( logValues, double );

  log->memoryValuesCount += valuesNumber;

  va_end( logValues );
}

static void DataLogging_RegisterList( int logID, size_t valuesNumber, double* valuesList )
{
  if( !kh_exist( logsList, (khint_t) logID ) ) return;
  
  Log log = kh_value( logsList, (khint_t) logID );

  if( log->memoryValuesCount + log->valuesNumber >= log->memoryBufferLength )
  {
    DataLogging_SaveData( logID, log->memoryBuffer, log->memoryValuesCount );
    log->memoryValuesCount = 0;
  }

  for( size_t valueLineIndex = 0; valueLineIndex < valuesNumber; valueLineIndex++ )
    log->memoryBuffer[ log->memoryValuesCount + valueLineIndex ] = valuesList[ valueLineIndex ];

  log->memoryValuesCount += valuesNumber;
}

static void DataLogging_SetDataPrecision( int logID, size_t decimalPlacesNumber )
{
  const size_t MAX_PRECISION = 6;
  
  if( !kh_exist( logsList, (khint_t) logID ) ) return;
  
  kh_value( logsList, (khint_t) logID )->dataPrecision = ( decimalPlacesNumber < MAX_PRECISION ) ? decimalPlacesNumber : MAX_PRECISION;
}

#endif /* DATA_LOGGING_H */
