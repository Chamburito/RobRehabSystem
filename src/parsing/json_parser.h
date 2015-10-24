#ifndef JSON_PARSER_H
#define JSON_PARSER_H

#include "file_parsing/file_parser_interface.h"

#include "klib/kson.h"
#include "klib/khash.h"

#include "debug/async_debug.h"

IMPLEMENT_INTERFACE( FileParser, JSONParser )

typedef struct _FileData
{
  kson_t* nodeTree;
  kson_node_t* currentNode;
  char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
}
FileData;
                                                
KHASH_MAP_INIT_INT( JSONInt, FileData )
static khash_t( JSONInt )* jsonFilesList = NULL;

static int LoadFile( const char* filePath )
{
  char* filePathExt = (char*) calloc( strlen( filePath ) + strlen( ".json" ) + 1, sizeof(char) );
  sprintf( filePathExt, "%s.json", filePath );
  
  DEBUG_PRINT( "looking for file: %s", filePathExt );
  
  FILE* configFile = fopen( filePathExt, "r" );
  
  free( filePathExt );
  
  if( configFile == NULL ) return -1;
  
  if( jsonFilesList == NULL ) jsonFilesList = kh_init( JSONInt );
  
  int fileKey = (int) kh_str_hash_func( filePath );
  
  int insertionStatus;
  khint_t newFileIndex = kh_put( JSONInt, jsonFilesList, fileKey, &insertionStatus );
  DEBUG_PRINT( "Trying to insert key: %d (status: %d)", fileKey, insertionStatus );
  if( insertionStatus > 0 )
  {
    fseek( configFile, 0, SEEK_END );
    long int fileSize = ftell( configFile );
    char* configString = (char*) calloc( fileSize + 1, sizeof(char) );
    fseek( configFile, 0, SEEK_SET );
    fread( configString, sizeof(char), fileSize, configFile );
  
    DEBUG_PRINT( "Config file content: %s", configString );
    
    kh_value( jsonFilesList, newFileIndex ).nodeTree = kson_parse( (const char*) configString );

    free( configString );
  }
  
  fclose( configFile );
  
  if( kh_value( jsonFilesList, newFileIndex ).nodeTree == NULL )
  {
    UnloadFile( (int) newFileIndex );
    return -1;
  }
  
  kh_value( jsonFilesList, newFileIndex ).currentNode = kh_value( jsonFilesList, newFileIndex ).nodeTree->root;

  DEBUG_PRINT( "New file inserted: %s (iterator: %u)", filePath, newFileIndex );
  DEBUG_PRINT( "Found %u keys in new JSON file", (size_t) kh_value( jsonFilesList, newFileIndex ).nodeTree->root->n );

  return (int) kh_key( jsonFilesList, newFileIndex );
}

static void UnloadFile( int fileID )
{
  khint_t fileIndex = kh_get( JSONInt, jsonFilesList, (khint_t) fileID );
  if( fileIndex == kh_end( jsonFilesList ) ) return;
    
  kson_destroy( kh_value( jsonFilesList, fileIndex ).nodeTree );
    
  kh_del( JSONInt, jsonFilesList, fileIndex );
    
  if( kh_size( jsonFilesList ) == 0 )
  {
    kh_destroy( JSONInt, jsonFilesList );
    jsonFilesList = NULL;
  }
}

static inline const kson_node_t* GetPathNode( int fileID, const char* path )
{
  khint_t fileIndex = kh_get( JSONInt, jsonFilesList, (khint_t) fileID );
  if( fileIndex == kh_end( jsonFilesList ) ) return NULL;
  
  const kson_node_t* currentNode = kh_value( jsonFilesList, fileIndex ).currentNode;
  char* searchPath = kh_value( jsonFilesList, fileIndex ).searchPath;
  
  strncpy( searchPath, path, FILE_PARSER_MAX_PATH_LENGTH );
  
  DEBUG_PRINT( "Search path: %s", searchPath );
  
  for( char* key = strtok( searchPath, "." ); key != NULL; key = strtok( NULL, "." ) )
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
  khint_t fileIndex = kh_get( JSONInt, jsonFilesList, (khint_t) fileID );
  if( fileIndex == kh_end( jsonFilesList ) ) return;
  
  kh_value( jsonFilesList, fileIndex ).currentNode = GetPathNode( fileID, path );
  DEBUG_PRINT( "setting base key to \"%s\"", path );
}

static char* GetStringValue( int fileID, const char* path )
{
  const kson_node_t* valueNode = GetPathNode( fileID, path );
  
  if( valueNode == NULL ) return NULL;
  
  if( valueNode->type != KSON_TYPE_SGL_QUOTE && valueNode->type != KSON_TYPE_DBL_QUOTE )
    return NULL;
  
  DEBUG_PRINT( "Found value: %s", valueNode->v.str );
  
  return valueNode->v.str;
}

static long GetIntegerValue( int fileID, const char* path )
{
  const kson_node_t* valueNode = GetPathNode( fileID, path );
  
  if( valueNode == NULL ) return 0L;
  
  if( valueNode->type != KSON_TYPE_NO_QUOTE ) return 0L;
  
  DEBUG_PRINT( "Found value: %ld", strtol( valueNode->v.str, NULL, 0 ) );
  
  return strtol( valueNode->v.str, NULL, 0 );
}

static double GetRealValue( int fileID, const char* path )
{
  const kson_node_t* valueNode = GetPathNode( fileID, path );
  
  if( valueNode == NULL ) return strtod( "NaN", NULL );
  
  if( valueNode->type != KSON_TYPE_NO_QUOTE ) return strtod( "NaN", NULL );
  
  DEBUG_PRINT( "Found value: %g", strtod( valueNode->v.str, NULL ) );
  
  return strtod( valueNode->v.str, NULL );
}

static bool GetBooleanValue( int fileID, const char* path )
{
  const kson_node_t* valueNode = GetPathNode( fileID, path );
  
  if( valueNode == NULL ) return false;
  
  if( valueNode->type != KSON_TYPE_NO_QUOTE ) return false;
  
  if( strcmp( valueNode->v.str, "true" ) == 0 ) return true;

  return false;
}

static size_t GetListSize( int fileID, const char* path )
{
  const kson_node_t* listNode = GetPathNode( fileID, path );
  
  if( listNode == NULL ) return 0;
  
  if( listNode->type != KSON_TYPE_BRACKET ) return 0;
  
  DEBUG_PRINT( "List %s size: %u", path, (size_t) listNode->n );
  
  return (size_t) listNode->n;
}

static bool HasKey( int fileID, const char* path )
{
  if( GetPathNode( fileID, path ) != NULL ) return true;
  
  return false;
}

#endif // JSON_PARSER_H
