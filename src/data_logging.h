#ifndef DATA_LOGGING_H
#define DATA_LOGGING_H

#ifdef _CVI_
  #include <ansi_c.h>
#else
  #include <fcntl.h>
  #include <malloc.h>
  #include <stdarg.h>
  #include <time.h>
#endif

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

static DataLog* dataLogsList = NULL;
static size_t dataLogsNumber = 0;

const size_t LOG_FILE_NAME_MAX_LEN = 128;

int DataLogging_CreateLog( const char* dirName, const char* logName, size_t logValuesNumber )
{
  dataLogsList = (DataLog*) realloc( dataLogsList, sizeof(DataLog) * ( dataLogsNumber + 1 ) );
  
  static char logFileName[ LOG_FILE_NAME_MAX_LEN ];
  
  sprintf( logFileName, "%s/%s_data-%u.log", ( dirName != NULL ) ? dirName : ".", logName, time( NULL ) );
  
  if( (dataLogsList[ dataLogsNumber ].file = fopen( logFileName, "w+" )) == NULL )
    return -1;
  
  dataLogsList[ dataLogsNumber ].valuesNumber = logValuesNumber;
  dataLogsList[ dataLogsNumber ].memoryValuesCount = 0;
  
  return dataLogsNumber++;
} 

void DataLogging_SaveData( int logFileID, double* dataList, size_t dataListSize )
{
  if( logFileID > -1 && logFileID < (int) dataLogsNumber )
  {
    FILE* logFile = dataLogsList[ logFileID ].file;
  
    size_t lineValuesNumber = dataLogsList[ logFileID ].valuesNumber;
    size_t linesNumber = dataListSize / lineValuesNumber;
  
    for( size_t lineIndex = 0; lineIndex < linesNumber; lineIndex++ )
    {
      for( size_t valueLineIndex = 0; valueLineIndex < lineValuesNumber; valueLineIndex++ )
        fprintf( logFile, "%.3lf\t", dataList[ lineIndex * lineValuesNumber + valueLineIndex ] );
      fprintf( logFile, "\n" );
    }
  }
}

void DataLogging_RegisterValues( int logFileID, ... )
{
  va_list logValues;
  
  if( logFileID > -1 && logFileID < (int) dataLogsNumber )
  {
    va_start( logValues, logFileID );
    
    double* memoryValues = (double*) dataLogsList[ logFileID ].memoryValues;
    
    size_t lastTotalValuesCount = dataLogsList[ logFileID ].memoryValuesCount;
    size_t logLineValuesNumber = dataLogsList[ logFileID ].valuesNumber;
    
    if( lastTotalValuesCount + logLineValuesNumber >= MEMORY_BUFFER_SIZE )
    {
      DataLogging_SaveData( logFileID, memoryValues, lastTotalValuesCount );
      lastTotalValuesCount = 0;
    }
    
    for( size_t valueLineIndex = 0; valueLineIndex < logLineValuesNumber; valueLineIndex++ )
      memoryValues[ lastTotalValuesCount + valueLineIndex ] = va_arg( logValues, double );
    
    dataLogsList[ logFileID ].memoryValuesCount = lastTotalValuesCount + logLineValuesNumber;
    
    va_end( logValues );
  }
}

void DataLogging_CloseLog( int logFileID )
{
  if( logFileID > -1 && logFileID < (int) dataLogsNumber )
  {
    DataLogging_SaveData( logFileID, dataLogsList[ logFileID ].memoryValues, dataLogsList[ logFileID ].memoryValuesCount );
    
    FILE* logFile = dataLogsList[ logFileID ].file;
    
    fclose( logFile );
  }
}

#endif /* DATA_LOGGING_H */
