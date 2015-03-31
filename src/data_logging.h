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

int DataLogging_CreateLog( const char* logName, size_t logValuesNumber )
{
  dataLogsList = (DataLog*) realloc( dataLogsList, sizeof(DataLog) * ( dataLogsNumber + 1 ) );
  
  static char logFileName[ LOG_FILE_NAME_MAX_LEN ];
  
  static time_t logTime;
  
  time( &logTime );
  
  sprintf( logFileName, "%s_data_log-%s.log", logName, "5"/*ctime( &logTime )*/ );
  
  FILE* logFile = dataLogsList[ dataLogsNumber ].file;
  
  if( (logFile = fopen( logFileName, "w+" )) == NULL )
    return -1;
  
  dataLogsList[ dataLogsNumber ].valuesNumber = logValuesNumber;
  dataLogsList[ dataLogsNumber ].memoryValuesCount = 0;
  
  return dataLogsNumber++;
} 

static void SaveLog( int logFileID )
{
  FILE* logFile = dataLogsList[ logFileID ].file;
  
  size_t totalValuesNumber = dataLogsList[ logFileID ].memoryValuesCount;
  size_t lineValuesNumber = dataLogsList[ logFileID ].valuesNumber;
  
  size_t linesNumber = totalValuesNumber / lineValuesNumber;
  
  double* memoryValues = (double*) dataLogsList[ logFileID ].memoryValues;
  
  for( size_t line = 0; line < linesNumber; line++ )
  {
    for( size_t valueIndex = 0; valueIndex < lineValuesNumber; valueIndex++ )
      fprintf( logFile, "a: %.3f\t", memoryValues[ line + valueIndex ] );
    fprintf( logFile, "\n" );
  }
  
  dataLogsList[ logFileID ].memoryValuesCount = 0;
}

void DataLogging_Register( int logFileID, ... )
{
  va_list logValues;
  
  if( logFileID > -1 && logFileID < (int) dataLogsNumber )
  {
    va_start( logValues, logFileID );
    
    double* memoryValues = (double*) dataLogsList[ logFileID ].memoryValues;
    
    size_t logValuesNumber = dataLogsList[ logFileID ].valuesNumber;
    for( size_t valueIndex = 0; valueIndex < logValuesNumber; valueIndex++ )
    {
      double value = va_arg( logValues, double );
      
      memoryValues[ dataLogsList[ logFileID ].memoryValuesCount ] = value;
      dataLogsList[ logFileID ].memoryValuesCount++;
    }
    
    if( dataLogsList[ logFileID ].memoryValuesCount + logValuesNumber > MEMORY_BUFFER_SIZE ) 
      SaveLog( logFileID );
    
    va_end( logValues );
  }
}

void DataLogging_CloseLog( int logFileID )
{
  if( logFileID > -1 && logFileID < (int) dataLogsNumber )
  {
    SaveLog( logFileID );
    
    FILE* logFile = dataLogsList[ logFileID ].file;
    
    fclose( logFile );
  }
}

#endif /* DATA_LOGGING_H */
