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

const size_t DATA_LOG_MAX_PRECISION = 15;

const int DATA_LOG_INVALID_ID = -1;

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

#define DATA_LOGGING_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, InitLog, const char*, size_t, size_t ) \
        function_init( void, namespace, EndLog, int ) \
        function_init( void, namespace, SaveData, int, double*, size_t ) \
        function_init( void, namespace, RegisterValues, int, size_t, ... ) \
        function_init( void, namespace, RegisterList, int, size_t, double* ) \
        function_init( void, namespace, SetDataPrecision, int, size_t )

INIT_NAMESPACE_INTERFACE( DataLogging, DATA_LOGGING_FUNCTIONS )

const size_t LOG_FILE_PATH_MAX_LEN = 256;
static int DataLogging_InitLog( const char* logFilePath, size_t logValuesNumber, size_t memoryBufferLength )
{
  static char logFilePathExt[ LOG_FILE_PATH_MAX_LEN ];
  
  if( logsList == NULL ) logsList = kh_init( LogInt );
  
  int logKey = (int) kh_str_hash_func( logFilePath );
  
  sprintf( logFilePathExt, "logs/%s-%lu.log", logFilePath, time( NULL ) );
  //sprintf( logFilePathExt, "logs/test-%lu.log", time( NULL ) );
  
  int insertionStatus;
  khint_t newLogIndex = kh_put( LogInt, logsList, logKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( logsList, newLogIndex ) = (Log) malloc( sizeof(LogData) );
    
    Log newLog = kh_value( logsList, newLogIndex );
    
    DEBUG_PRINT( "trying to open file %s", logFilePathExt );
    if( (newLog->file = fopen( logFilePathExt, "w+" )) == NULL )
    {
      perror( "error opening file" );
      DataLogging_EndLog( (int) newLogIndex );
      return DATA_LOG_INVALID_ID;
    }
    
    newLog->valuesNumber = logValuesNumber;
    newLog->memoryBuffer = (double*) calloc( memoryBufferLength, sizeof(double) );
    newLog->memoryBufferLength = memoryBufferLength;
    newLog->memoryValuesCount = 0;
    
    newLog->dataPrecision = 3;
  }
  
  return (int) kh_key( logsList, newLogIndex );
}

static void DataLogging_EndLog( int logID )
{
  khint_t logIndex = kh_get( LogInt, logsList, (khint_t) logID );
  if( logIndex == kh_end( logsList ) ) return;
  
  Log log = kh_value( logsList, logIndex );
  
  DataLogging_SaveData( logID, (double*) log->memoryBuffer, log->memoryValuesCount );
  
  if( log->file != NULL ) fclose( log->file );
  
  free( log );
  
  kh_del( LogInt, logsList, logIndex );
  
  if( kh_size( logsList ) == 0 )
  {
    kh_destroy( LogInt, logsList );
    
    logsList = NULL;
  }
}

static void DataLogging_SaveData( int logID, double* dataList, size_t dataListSize )
{
  khint_t logIndex = kh_get( LogInt, logsList, (khint_t) logID );
  if( logIndex == kh_end( logsList ) ) return;
  
  Log log = kh_value( logsList, logIndex );
  
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
  khint_t logIndex = kh_get( LogInt, logsList, (khint_t) logID );
  if( logIndex == kh_end( logsList ) ) return;
  
  Log log = kh_value( logsList, logIndex );
  
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
  khint_t logIndex = kh_get( LogInt, logsList, (khint_t) logID );
  if( logIndex == kh_end( logsList ) ) return;
  
  Log log = kh_value( logsList, logIndex );

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
  khint_t logIndex = kh_get( LogInt, logsList, (khint_t) logID );
  if( logIndex == kh_end( logsList ) ) return;
  
  Log log = kh_value( logsList, logIndex );
  
  log->dataPrecision = ( decimalPlacesNumber < DATA_LOG_MAX_PRECISION ) ? decimalPlacesNumber : DATA_LOG_MAX_PRECISION;
}

#endif /* DATA_LOGGING_H */
