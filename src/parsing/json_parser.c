#ifndef JSON_PARSER_H
#define JSON_PARSER_H

#include "parsing/parser_interface.h"

#include "klib/kson.h"
#include "klib/khash.h"

#include "debug/async_debug.h"

typedef struct _JSONData
{
  kson_t* nodeTree;
  kson_node_t* currentNode;
  char searchPath[ PARSER_MAX_KEY_PATH_LENGTH ];
}
JSONData;
                                                
KHASH_MAP_INIT_INT( JSONInt, JSONData )
static khash_t( JSONInt )* jsonDataList = NULL;

void UnloadData( int );

int LoadStringData( const char* configString )
{
  if( jsonDataList == NULL ) jsonDataList = kh_init( JSONInt );
  
  int dataKey = (int) kh_size( jsonDataList );
  
  int insertionStatus;
  khint_t newDataIndex = kh_put( JSONInt, jsonDataList, dataKey, &insertionStatus );
  DEBUG_PRINT( "Trying to insert key: %d (status: %d)", dataKey, insertionStatus );
  if( insertionStatus > 0 )
  {
    DEBUG_PRINT( "data content: %s", configString );                                           
    kh_value( jsonDataList, newDataIndex ).nodeTree = kson_parse( (const char*) configString );
    
    if( kh_value( jsonDataList, newDataIndex ).nodeTree == NULL )
    {
      UnloadData( (int) newDataIndex );
      return PARSED_DATA_INVALID_ID;
    }
  
    kh_value( jsonDataList, newDataIndex ).currentNode = kh_value( jsonDataList, newDataIndex ).nodeTree->root;

    DEBUG_PRINT( "Found %u keys in new JSON data", (size_t) kh_value( jsonDataList, newDataIndex ).nodeTree->root->n );
  }
  
  return (int) kh_key( jsonDataList, newDataIndex );
}

int LoadFileData( const char* filePath )
{
  char filePathExt[ PARSER_MAX_FILE_PATH_LENGTH ];
  sprintf( filePathExt, "%s.json", filePath );
  
  DEBUG_PRINT( "looking for file: %s", filePathExt );
  
  FILE* configFile = fopen( filePathExt, "r" );
  if( configFile == NULL ) return PARSED_DATA_INVALID_ID;
  
  fseek( configFile, 0, SEEK_END );
  long int fileSize = ftell( configFile );
  char* configString = (char*) calloc( fileSize + 1, sizeof(char) );
  fseek( configFile, 0, SEEK_SET );
  fread( configString, sizeof(char), fileSize, configFile );
  fclose( configFile );  
  
  int newDataID = LoadStringData( configString );

  free( configString );
  
  if( newDataID != PARSED_DATA_INVALID_ID ) DEBUG_PRINT( "file %s data inserted: %s", filePathExt );

  return newDataID;
}

void UnloadData( int dataID )
{
  khint_t dataIndex = kh_get( JSONInt, jsonDataList, (khint_t) dataID );
  if( dataIndex == kh_end( jsonDataList ) ) return;
    
  kson_destroy( kh_value( jsonDataList, dataIndex ).nodeTree );
    
  kh_del( JSONInt, jsonDataList, dataIndex );
    
  if( kh_size( jsonDataList ) == 0 )
  {
    kh_destroy( JSONInt, jsonDataList );
    jsonDataList = NULL;
  }
}

static inline const kson_node_t* GetPathNode( int dataID, const char* path )
{
  khint_t dataIndex = kh_get( JSONInt, jsonDataList, (khint_t) dataID );
  if( dataIndex == kh_end( jsonDataList ) ) return NULL;
  
  const kson_node_t* currentNode = kh_value( jsonDataList, dataIndex ).currentNode;
  char* searchPath = kh_value( jsonDataList, dataIndex ).searchPath;
  
  strncpy( searchPath, path, PARSER_MAX_KEY_PATH_LENGTH );
  
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

void SetBaseKey( int dataID, const char* path )
{
  khint_t dataIndex = kh_get( JSONInt, jsonDataList, (khint_t) dataID );
  if( dataIndex == kh_end( jsonDataList ) ) return;
  
  kh_value( jsonDataList, dataIndex ).currentNode = GetPathNode( dataID, path );
  DEBUG_PRINT( "setting base key to \"%s\"", path );
}

char* GetStringValue( int dataID, const char* path )
{
  const kson_node_t* valueNode = GetPathNode( dataID, path );
  
  if( valueNode == NULL ) return NULL;
  
  if( valueNode->type != KSON_TYPE_SGL_QUOTE && valueNode->type != KSON_TYPE_DBL_QUOTE )
    return NULL;
  
  if( valueNode->v.str == NULL ) return "";
  
  DEBUG_PRINT( "Found value: %s", valueNode->v.str );
  
  return valueNode->v.str;
}

long GetIntegerValue( int dataID, const char* path )
{
  const kson_node_t* valueNode = GetPathNode( dataID, path );
  
  if( valueNode == NULL ) return 0L;
  
  if( valueNode->type != KSON_TYPE_NO_QUOTE ) return 0L;
  
  DEBUG_PRINT( "Found value: %ld", strtol( valueNode->v.str, NULL, 0 ) );
  
  return strtol( valueNode->v.str, NULL, 0 );
}

double GetRealValue( int dataID, const char* path )
{
  const kson_node_t* valueNode = GetPathNode( dataID, path );
  
  if( valueNode == NULL ) return strtod( "NaN", NULL );
  
  if( valueNode->type != KSON_TYPE_NO_QUOTE ) return strtod( "NaN", NULL );
  
  DEBUG_PRINT( "Found value: %g", strtod( valueNode->v.str, NULL ) );
  
  return strtod( valueNode->v.str, NULL );
}

bool GetBooleanValue( int dataID, const char* path )
{
  const kson_node_t* valueNode = GetPathNode( dataID, path );
  
  if( valueNode == NULL ) return false;
  
  if( valueNode->type != KSON_TYPE_NO_QUOTE ) return false;
  
  if( strcmp( valueNode->v.str, "true" ) == 0 ) return true;

  return false;
}

size_t GetListSize( int dataID, const char* path )
{
  const kson_node_t* listNode = GetPathNode( dataID, path );
  
  if( listNode == NULL ) return 0;
  
  if( listNode->type != KSON_TYPE_BRACKET ) return 0;
  
  DEBUG_PRINT( "List %s size: %u", path, (size_t) listNode->n );
  
  return (size_t) listNode->n;
}

bool HasKey( int dataID, const char* path )
{
  if( GetPathNode( dataID, path ) != NULL ) return true;
  
  return false;
}

#endif // JSON_PARSER_H
