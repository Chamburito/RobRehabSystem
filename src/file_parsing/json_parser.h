#ifndef JSON_PARSER_H
#define JSON_PARSER_H

#include "file_parsing/file_parser_interface.h"

#include "klib/kson.h"
#include "klib/khash.h"

#include "debug/async_debug.h"

static int OpenFile( const char* );
static void CloseFile( int );
static void SetBaseKey( int, const char* );
static long GetIntegerValue( int, const char* );
static double GetRealValue( int, const char* );
static char* GetStringValue( int, const char* );
static bool GetBooleanValue( int, const char* );
static size_t GetListSize( int, const char* );
static bool HasKey( int, const char* );

const FileParserMethods JSONParser = { .OpenFile = OpenFile, .CloseFile = CloseFile, .SetBaseKey = SetBaseKey, .GetIntegerValue = GetIntegerValue,
                                       .GetRealValue = GetRealValue, .GetStringValue = GetStringValue, .GetBooleanValue = GetBooleanValue, .GetListSize = GetListSize, .HasKey = HasKey };

typedef struct _FileData
{
  kson_t* nodeTree;
  kson_node_t* currentNode;
  char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
}
FileData;
                                                
KHASH_MAP_INIT_INT( JSON, FileData )
static khash_t( JSON )* jsonFilesList = NULL;

static int OpenFile( const char* fileName )
{
  char fileNameExt[ strlen( fileName ) + 5 ];
  sprintf( fileNameExt, "%s.json", fileName );
  
  struct FILE* configFile = fopen( fileNameExt, "r" );
  if( configFile != NULL )
  {
    fseek( configFile, 0, SEEK_END );
    size_t fileSize = ftell( configFile );
    char configString[ fileSize ];
    fseek( configFile, 0, SEEK_SET );
    fread( configString, sizeof(char), fileSize, configFile );
    fclose( configFile );
    
    DEBUG_PRINT( "Config file content: %s", configString );
    
    if( jsonFilesList == NULL ) jsonFilesList = kh_init( JSON );
    
    int insertionStatus;
    khint_t fileID = kh_put( JSON, jsonFilesList, (long) configFile, &insertionStatus );
    if( insertionStatus == -1 ) return -1;
    
    kh_value( jsonFilesList, fileID ).nodeTree = kson_parse( (const char*) configString );
    kh_value( jsonFilesList, fileID ).currentNode = kh_value( jsonFilesList, fileID ).nodeTree->root;
    
    DEBUG_PRINT( "Found %u keys in JSON file", kh_value( jsonFilesList, fileID )->root->n );
    
    return (int) fileID;
  }
  
  return -1;
}

static void CloseFile( int fileID )
{
  if( !kh_exist( jsonFilesList, (khint_t) fileID ) ) return;
    
  kson_destroy( kh_value( jsonFilesList, (khint_t) fileID ).nodeTree );
    
  kh_del( JSON, jsonFilesList, (khint_t) fileID );
    
  if( kh_size( jsonFilesList ) == 0 )
  {
    kh_destroy( JSON, jsonFilesList );
    jsonFilesList = NULL;
  }
}

static inline const kson_node_t* GetPathNode( int fileID, const kson_node_t* currentNode, const char* path )
{
  if( !kh_exist( jsonFilesList, (khint_t) fileID ) ) return NULL;
  
  strncpy( kh_value( jsonFilesList, fileID ).searchPath, path, FILE_PARSER_MAX_PATH_LENGTH );
  
  DEBUG_PRINT( "Search path: %s", kh_value( jsonFilesList, fileID ).searchPath );
  
  for( char* key = strtok( kh_value( jsonFilesList, fileID ).searchPath, "." ); key != NULL; key = strtok( NULL, "." ) )
  {
    if( currentNode->type == KSON_TYPE_BRACE )
      currentNode = kson_by_key( currentNode, key );
    else if( currentNode->type == KSON_TYPE_BRACKET )
      currentNode = kson_by_index( currentNode, strtoul( key, NULL, 10 ) );
  }
    
  return currentNode;
}

void SetBaseKey( int fileID, const char* path )
{
  const kson_node_t* baseNode = GetPathNode( fileID, kh_value( jsonFilesList, fileID ).nodeTree->root, path );
  
  if( baseNode != NULL )
    kh_value( jsonFilesList, fileID ).currentNode = baseNode;
}

static char* GetStringValue( int fileID, const char* path )
{
  const kson_node_t* valueNode = GetPathNode( fileID, kh_value( jsonFilesList, fileID ).currentNode, path );
  
  if( valueNode == NULL ) return NULL;
  
  if( valueNode->type != KSON_TYPE_SGL_QUOTE && valueNode->type != KSON_TYPE_DBL_QUOTE )
    return NULL;
  
  DEBUG_PRINT( "Found value: %s", valueNode->v.str );
  
  return valueNode->v.str;
}

static long GetIntegerValue( int fileID, const char* path )
{
  const kson_node_t* valueNode = GetPathNode( fileID, kh_value( jsonFilesList, fileID ).currentNode, path );
  
  if( valueNode == NULL ) return 0L;
  
  if( valueNode->type != KSON_TYPE_NO_QUOTE ) return 0L;
  
  DEBUG_PRINT( "Found value: %ld", strtol( valueNode->v.str, NULL, 0 ) );
  
  return strtol( valueNode->v.str, NULL, 0 );
}

static double GetRealValue( int fileID, const char* path )
{
  const kson_node_t* valueNode = GetPathNode( fileID, kh_value( jsonFilesList, fileID ).currentNode, path );
  
  if( valueNode == NULL ) return strtod( "NaN", NULL );
  
  if( valueNode->type != KSON_TYPE_NO_QUOTE ) return strtod( "NaN", NULL );
  
  DEBUG_PRINT( "Found value: %g", strtod( valueNode->v.str, NULL ) );
  
  return strtod( valueNode->v.str, NULL );
}

static bool GetBooleanValue( int fileID, const char* path )
{
  const kson_node_t* valueNode = GetPathNode( fileID, kh_value( jsonFilesList, fileID ).currentNode, path );
  
  if( valueNode == NULL ) return false;
  
  if( valueNode->type != KSON_TYPE_NO_QUOTE ) return false;
  
  if( strcmp( valueNode->v.str, "true" ) == 0 ) return true;

  return false;
}

static size_t GetListSize( int fileID, const char* path )
{
  const kson_node_t* listNode = GetPathNode( fileID, kh_value( jsonFilesList, fileID ).currentNode, path );
  
  if( listNode == NULL ) return 0;
  
  if( listNode->type != KSON_TYPE_BRACKET ) return 0;
  
  DEBUG_PRINT( "List %s size: %u", path, listNode->n );
  
  return listNode->n;
}

static bool HasKey( int fileID, const char* path )
{
  if( GetPathNode( fileID, kh_value( jsonFilesList, fileID ).currentNode, path ) != NULL ) return true;
  
  return false;
}

#endif // JSON_PARSER_H
