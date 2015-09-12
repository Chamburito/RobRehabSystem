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

const size_t MEMORY_BUFFER_SIZE = 1000;

typedef struct _DataLog
{
  FILE* file;
  size_t valuesNumber;
  double memoryValues[ MEMORY_BUFFER_SIZE ];
  size_t memoryValuesCount;
}
DataLog;

KHASH_MAP_INIT_INT( LogInt, DataLog );
static khash_t( LogInt )* dataLogsList = NULL;

static int DataLogging_InitLog( const char*, size_t );
static void DataLogging_EndLog( int ); 
static void DataLogging_SaveData( int , double*, size_t );
static void DataLogging_RegisterValues( int, ... );

const struct
{
  int (*InitLog)( const char*, size_t );
  void (*EndLog)( int ); 
  void (*SaveData)( int , double*, size_t );
  void (*RegisterValues)( int, ... );
}
DataLogging = { .InitLog = DataLogging_InitLog, .EndLog = DataLogging_EndLog, .SaveData = DataLogging_SaveData, .RegisterValues = DataLogging_RegisterValues };

const size_t LOG_FILE_PATH_MAX_LEN = 256;
static int DataLogging_InitLog( const char* logFilePath, size_t logValuesNumber )
{
  static char logFilePathExt[ LOG_FILE_PATH_MAX_LEN ];
  
  if( dataLogsList == NULL ) dataLogsList = kh_init( LogInt );
  
  int logKey = (int) kh_str_hash_func( logFilePath );
  
  sprintf( logFilePathExt, "%s-%u.log", logFilePath, time( NULL ) );
  
  int insertionStatus;
  khint_t newLogID = kh_put( LogInt, dataLogsList, logKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    DEBUG_PRINT( "trying to open file %s", logFilePathExt );
    kh_value( dataLogsList, newLogID ).file = fopen( logFilePathExt, "w+" );
    if( kh_value( dataLogsList, newLogID ).file == NULL )
    {
      perror( "error opening file" );
      DataLogging_EndLog( (int) newLogID );
      return -1;
    }
  
    kh_value( dataLogsList, newLogID ).valuesNumber = logValuesNumber;
    kh_value( dataLogsList, newLogID ).memoryValuesCount = 0;
  }
  
  return (int) newLogID;
}

static void DataLogging_EndLog( int logID )
{
  if( !kh_exist( dataLogsList, (khint_t) logID ) ) return;
  
  DataLog* log = &(kh_value( dataLogsList, (khint_t) logID ));
  
  DataLogging_SaveData( logID, (double*) log->memoryValues, log->memoryValuesCount );
  
  if( log->file != NULL ) fclose( log->file );
  
  kh_del( LogInt, dataLogsList, (khint_t) logID );
  
  if( kh_size( dataLogsList ) == 0 )
  {
    kh_destroy( LogInt, dataLogsList );
    
    dataLogsList = NULL;
  }
}

static void DataLogging_SaveData( int logID, double* dataList, size_t dataListSize )
{
  if( !kh_exist( dataLogsList, (khint_t) logID ) ) return;
  
  DataLog* log = &(kh_value( dataLogsList, (khint_t) logID ));
  
  size_t linesNumber = dataListSize / log->valuesNumber;
  for( size_t lineIndex = 0; lineIndex < linesNumber; lineIndex++ )
  {
    for( size_t lineValueIndex = 0; lineValueIndex < log->valuesNumber; lineValueIndex++ )
      fprintf( log->file, "%.3lf\t", dataList[ lineIndex * log->valuesNumber + lineValueIndex ] );
    fprintf( log->file, "\n" );
  }
}

static void DataLogging_RegisterValues( int logID, ... )
{
  if( !kh_exist( dataLogsList, (khint_t) logID ) ) return;
  
  DataLog* log = &(kh_value( dataLogsList, (khint_t) logID ));
  
  va_list logValues;
  
  va_start( logValues, logID );

  if( log->memoryValuesCount + log->valuesNumber >= MEMORY_BUFFER_SIZE )
  {
    DataLogging_SaveData( logID, log->memoryValues, log->memoryValuesCount );
    log->memoryValuesCount = 0;
  }

  for( size_t valueLineIndex = 0; valueLineIndex < log->valuesNumber; valueLineIndex++ )
    log->memoryValues[ log->memoryValuesCount + valueLineIndex ] = va_arg( logValues, double );

  log->memoryValuesCount += log->valuesNumber;

  va_end( logValues );
}

#endif /* DATA_LOGGING_H */
