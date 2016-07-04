#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>

#include "klib/khash.h"

#include "debug/async_debug.h"

#include "debug/data_logging.h"

#define TIME_STAMP_STRING_LENGTH 32


struct _LogData
{
  FILE* file;
  size_t valuesNumber;
  double* memoryBuffer;
  size_t memoryBufferLength;
  size_t memoryValuesCount;
  int dataPrecision;
};

static char baseDirectoryPath[ LOG_FILE_PATH_MAX_LEN ] = "logs";
static char timeStampString[ TIME_STAMP_STRING_LENGTH ] = "";

KHASH_MAP_INIT_INT( LogInt, Log );
static khash_t( LogInt )* logsList = NULL;

DEFINE_NAMESPACE_INTERFACE( DataLogging, DATA_LOGGING_INTERFACE )


int DataLogging_InitLog( const char* logFilePath, size_t logValuesNumber, size_t memoryBufferLength )
{
  static char logFilePathExt[ LOG_FILE_PATH_MAX_LEN ];
  
  if( logsList == NULL ) logsList = kh_init( LogInt );
  
  int logKey = (int) kh_str_hash_func( logFilePath );
  
  sprintf( logFilePathExt, "%s/%s-%s.log", baseDirectoryPath, logFilePath, timeStampString );
  
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

void DataLogging_EndLog( int logID )
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

void DataLogging_SetBaseDirectory( const char* directoryPath )
{
  time_t timeStamp = time( NULL );
  strncpy( timeStampString, asctime( localtime( &timeStamp ) ), TIME_STAMP_STRING_LENGTH );
  for( size_t charIndex = 0; charIndex < TIME_STAMP_STRING_LENGTH; charIndex++ )
  {
    char c = timeStampString[ charIndex ];
    if( c == ' ' || c == ':' ) timeStampString[ charIndex ] = '_';
    else if( c == '\n' || c == '\r' ) timeStampString[ charIndex ] = '\0';
  }
  
  sprintf( baseDirectoryPath, "logs/%s", ( directoryPath != NULL ) ? directoryPath : "" );
}

void DataLogging_SaveData( int logID, double* dataList, size_t dataListSize )
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

void DataLogging_RegisterValues( int logID, size_t valuesNumber, ... )
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

void DataLogging_RegisterList( int logID, size_t valuesNumber, double* valuesList )
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

void DataLogging_SetDataPrecision( int logID, size_t decimalPlacesNumber )
{
  khint_t logIndex = kh_get( LogInt, logsList, (khint_t) logID );
  if( logIndex == kh_end( logsList ) ) return;
  
  Log log = kh_value( logsList, logIndex );
  
  log->dataPrecision = ( decimalPlacesNumber < DATA_LOG_MAX_PRECISION ) ? decimalPlacesNumber : DATA_LOG_MAX_PRECISION;
}
