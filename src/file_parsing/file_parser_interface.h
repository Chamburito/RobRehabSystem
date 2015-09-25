#ifndef FILE_PARSER_H
#define FILE_PARSER_H

#include <stdbool.h>

#ifdef _CVI_
  #include <utility.h>
  #define SET_PATH( dirPath ) SetDir( dirPath );
#else
  #define SET_PATH( dirPath ) system( "cd " dirPath );
#endif

#define FILE_PARSER_MAX_PATH_LENGTH 256

typedef struct _FileParser
{
  int (*LoadFile)( const char* );
  void (*UnloadFile)( int );
  void (*SetBaseKey)( int, const char* );
  long (*GetIntegerValue)( int, const char* );
  double (*GetRealValue)( int, const char* );
  char* (*GetStringValue)( int, const char* );
  bool (*GetBooleanValue)( int, const char* );
  size_t (*GetListSize)( int, const char* );
  bool (*HasKey)( int, const char* );
}
FileParser;

typedef FileParser* FileParserInterface;

#endif // FILE_PARSER_H
