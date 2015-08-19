#ifndef FILE_PARSER_H
#define FILE_PARSER_H

#include <stdbool.h>

#define FILE_PARSER_MAX_PATH_LENGTH 256

typedef struct _FileParserInterface
{
  int (*OpenFile)( const char* );
  void (*CloseFile)( int );
  void (*SetBaseKey)( int, const char* );
  long (*GetIntegerValue)( int, const char* );
  double (*GetRealValue)( int, const char* );
  char* (*GetStringValue)( int, const char* );
  bool (*GetBooleanValue)( int, const char* );
  size_t (*GetListSize)( int, const char* );
  bool (*HasKey)( int, const char* );
}
FileParserInterface;

#endif // FILE_PARSER_H
