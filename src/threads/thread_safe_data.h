#ifndef THREAD_SAFE_DATA_H
#define THREAD_SAFE_DATA_H

#ifdef WIN32
  #include "threads_windows.h"
#else
  #include "threads_unix.h"
#endif

#include "interfaces.h"

#include "klib/khash.h"

#include <stdbool.h>
#include <stdlib.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD SAFE QUEUE                                       /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _ThreadSafeQueueData
{
  void** cache;
  size_t first, last, maxLength;
  size_t itemSize;
  ThreadLock accessLock;
  Semaphore countLock;
}
ThreadSafeQueueData;

typedef ThreadSafeQueueData* ThreadSafeQueue;

enum TSQueueAccessMode { TSQUEUE_WAIT, TSQUEUE_NOWAIT };

#define THREAD_SAFE_QUEUE_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( ThreadSafeQueue, Namespace, Create, size_t, size_t ) \
        INIT_FUNCTION( void, Namespace, Discard, ThreadSafeQueue ) \
        INIT_FUNCTION( size_t, Namespace, GetItemsCount, ThreadSafeQueue ) \
        INIT_FUNCTION( bool, Namespace, Enqueue, ThreadSafeQueue, void*, enum TSQueueAccessMode ) \
        INIT_FUNCTION( bool, Namespace, Dequeue, ThreadSafeQueue, void*, enum TSQueueAccessMode )

INIT_NAMESPACE_INTERFACE( ThreadSafeQueues, THREAD_SAFE_QUEUE_INTERFACE )


ThreadSafeQueue ThreadSafeQueues_Create( size_t maxLength, size_t itemSize )
{
  ThreadSafeQueue queue = (ThreadSafeQueue) malloc( sizeof(ThreadSafeQueueData) );
  
  queue->maxLength = maxLength;
  queue->itemSize = itemSize;
  
  queue->cache = (void**) calloc( queue->maxLength, sizeof(void*) );
  for( size_t i = 0; i < queue->maxLength; i++ )
    queue->cache[ i ] = (void*) malloc( itemSize );
  
  queue->first = queue->last = 0;
  
  queue->accessLock = ThreadLocks.Create();
  queue->countLock = Semaphores_Create( 0, queue->maxLength );
  
  return queue;
}

void ThreadSafeQueues_Discard( ThreadSafeQueue queue )
{
  if( queue != NULL )
  {
    for( size_t i = 0; i < queue->maxLength; i++ )
      free( queue->cache[ i ] );
    free( queue->cache );
    
    ThreadLocks.Discard( queue->accessLock );
    Semaphores_Discard( queue->countLock );

    free( queue );
    queue = NULL;
  }
}

extern inline size_t ThreadSafeQueues_GetItemsCount( ThreadSafeQueue queue )
{
  return ( queue->last - queue->first );
}

bool ThreadSafeQueues_Enqueue( ThreadSafeQueue queue, void* buffer, enum TSQueueAccessMode mode )
{
  static void* dataIn = NULL;
  
  if( mode == TSQUEUE_WAIT ) Semaphores_Increment( queue->countLock );
  
  ThreadLocks.Aquire( queue->accessLock );
  
  dataIn = queue->cache[ queue->last % queue->maxLength ];
  memcpy( dataIn, buffer, queue->itemSize );
  if( ThreadSafeQueues_GetItemsCount( queue ) == queue->maxLength ) queue->first++;
  queue->last++;
  
  ThreadLocks.Release( queue->accessLock );
  
  return true;
}

bool ThreadSafeQueues_Dequeue( ThreadSafeQueue queue, void* buffer, enum TSQueueAccessMode mode )
{
  static void* dataOut = NULL;

  if( mode == TSQUEUE_NOWAIT && ThreadSafeQueues_GetItemsCount( queue ) == 0 )
    return false;
  
  Semaphores_Decrement( queue->countLock );
  
  ThreadLocks.Aquire( queue->accessLock );
  
  dataOut = queue->cache[ queue->first % queue->maxLength ];
  buffer = memcpy( buffer, dataOut, queue->itemSize );
  queue->first++;
  
  ThreadLocks.Release( queue->accessLock );
  
  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD SAFE LIST                                        /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

static const size_t LIST_LENGTH_INCREMENT = 10;

typedef struct _Item
{
  int key;
  void* data;
}
Item;

typedef struct _ThreadSafeListData
{
  Item* data;
  size_t length, itemsCount, insertCount;
  size_t itemSize;
  ThreadLock accessLock;
}
ThreadSafeListData;

typedef ThreadSafeListData* ThreadSafeList;

#define THREAD_SAFE_LIST_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( ThreadSafeList, Namespace, Create, size_t ) \
        INIT_FUNCTION( void, Namespace, Discard, ThreadSafeList ) \
        INIT_FUNCTION( size_t, Namespace, GetItemsCount, ThreadSafeList ) \
        INIT_FUNCTION( size_t, Namespace, Insert, ThreadSafeList, void* ) \
        INIT_FUNCTION( int, Namespace, GetIndexKey, ThreadSafeList, size_t ) \
        INIT_FUNCTION( bool, Namespace, Remove, ThreadSafeList, int ) \
        INIT_FUNCTION( bool, Namespace, GetItem, ThreadSafeList, int, void* ) \
        INIT_FUNCTION( void*, Namespace, AquireItem, ThreadSafeList, int ) \
        INIT_FUNCTION( void, Namespace, ReleaseItem, ThreadSafeList ) \
        INIT_FUNCTION( bool, Namespace, SetItem, ThreadSafeList, int, void* )

INIT_NAMESPACE_INTERFACE( ThreadSafeLists, THREAD_SAFE_LIST_INTERFACE )


ThreadSafeList ThreadSafeLists_Create( size_t itemSize )
{
  ThreadSafeList list = (ThreadSafeList) malloc( sizeof(ThreadSafeListData) );
  
  list->data = (Item*) malloc( LIST_LENGTH_INCREMENT * sizeof(Item) );
  for( size_t position = 0; position < LIST_LENGTH_INCREMENT; position++ )
    list->data[ position ].data = malloc( itemSize );
  
  list->length = LIST_LENGTH_INCREMENT;
  list->itemsCount = list->insertCount = 0;
  list->itemSize = itemSize;
  
  list->accessLock = ThreadLocks.Create();
  
  return list;
}

void ThreadSafeLists_Discard( ThreadSafeList list )
{
  if( list != NULL )
  {
    for( size_t dataIndex = 0; dataIndex < list->itemsCount; dataIndex++ )
        free( list->data[ dataIndex ].data );
    free( list->data );
    
    ThreadLocks.Discard( list->accessLock );

    free( list );
    list = NULL;
  }
}

size_t ThreadSafeLists_GetItemsCount( ThreadSafeList list )
{
  return list->length;
}

int ListCompare( const void* ref_item_1, const void* ref_item_2 )
{
  return ( ((Item*) ref_item_1)->key - ((Item*) ref_item_2)->key );
}

size_t ThreadSafeLists_Insert( ThreadSafeList list, void* dataIn )
{
  ThreadLocks.Aquire( list->accessLock );
  
  if( list->itemsCount + 1 > list->length )
  {
    list->length += LIST_LENGTH_INCREMENT;
    list->data = (Item*) realloc( list->data, list->length * sizeof(Item) );
  }
  
  list->data[ list->itemsCount ].key = list->insertCount;
  
  list->data[ list->itemsCount ].data = (void*) malloc( list->itemSize );
  memcpy( list->data[ list->itemsCount ].data, dataIn, list->itemSize );
  
  qsort( (void*) list->data, list->length, list->itemSize, ListCompare );
  
  list->insertCount++;
  list->itemsCount++;
  
  ThreadLocks.Release( list->accessLock );
  
  return list->insertCount;
}

int ThreadSafeLists_GetIndexKey( ThreadSafeList list, size_t index )
{
  if( list == NULL ) return -1;
  
  if( index > list->itemsCount ) return -1;
  
  return list->data[ index ].key;
}

bool ThreadSafeLists_Remove( ThreadSafeList list, int key )
{
  ThreadLocks.Aquire( list->accessLock );
  
  if( list->itemsCount == 0 ) return false;
  
  Item comparisonItem = { key, NULL };
  
  Item* foundItem = (Item*) bsearch( (void*) &comparisonItem, (void*) list->data, list->length, list->itemSize, ListCompare );
  if( foundItem == NULL ) return false;
  
  free( foundItem->data );
  foundItem->key = INFINITE;
  
  qsort( (void*) list->data, list->length, list->itemSize, ListCompare );
  
  list->itemsCount--;
  
  if( list->itemsCount < list->length - LIST_LENGTH_INCREMENT )
  {
    list->length -= LIST_LENGTH_INCREMENT;
    list->data = (Item*) realloc( list->data, list->length * sizeof(Item) );
  }
  
  ThreadLocks.Release( list->accessLock );
  
  return true;
}

void* ThreadSafeLists_AquireItem( ThreadSafeList list, int key )
{
  Item comparisonItem = { key, NULL };
  
  ThreadLocks.Aquire( list->accessLock );
  
  Item* foundItem = (Item*) bsearch( (void*) &comparisonItem, (void*) list->data, list->length, list->itemSize, ListCompare );
  if( foundItem == NULL ) return NULL;
  
  return foundItem->data;
}

void ThreadSafeLists_ReleaseItem( ThreadSafeList list )
{
  ThreadLocks.Release( list->accessLock );
}

bool ThreadSafeLists_GetItem( ThreadSafeList list, int key, void* dataOut )
{
  void* foundData = ThreadSafeLists_AquireItem( list, key );
  if( foundData == NULL ) return false;
  
  if( list->itemSize > sizeof(void*) ) foundData = &foundData;
    
  if( dataOut != NULL ) memcpy( dataOut, foundData, list->itemSize );
    
  ThreadSafeLists_ReleaseItem( list );
  
  return true;
}

bool ThreadSafeLists_SetItem( ThreadSafeList list, int key, void* dataIn )
{
  void* foundData = ThreadSafeLists_AquireItem( list, key );
  if( foundData == NULL ) return false;
  
  if( list->itemSize > sizeof(void*) ) foundData = &foundData;
  
  if( dataIn != NULL ) memcpy( foundData, dataIn, list->itemSize );
  
  ThreadSafeLists_ReleaseItem( list );
  
  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      THREAD SAFE MAP                                        /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

enum TSMapKeyType { TSMAP_INT, TSMAP_STR };

KHASH_MAP_INIT_INT64( RefInt, void* )

typedef struct _ThreadSafeMapData
{
  khash_t( RefInt )* hashTable;
  enum TSMapKeyType keyType;
  size_t itemSize;
  ThreadLock accessLock;
}
ThreadSafeMapData;

typedef ThreadSafeMapData* ThreadSafeMap;

#define THREAD_SAFE_MAP_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( ThreadSafeMap, Namespace, Create, enum TSMapKeyType, size_t ) \
        INIT_FUNCTION( void, Namespace, Discard, ThreadSafeMap ) \
        INIT_FUNCTION( size_t, Namespace, GetItemsCount, ThreadSafeMap ) \
        INIT_FUNCTION( unsigned long, Namespace, SetItem, ThreadSafeMap, const void*, void* ) \
        INIT_FUNCTION( bool, Namespace, RemoveItem, ThreadSafeMap, unsigned long ) \
        INIT_FUNCTION( void*, Namespace, AquireItem, ThreadSafeMap, unsigned long ) \
        INIT_FUNCTION( void, Namespace, ReleaseItem, ThreadSafeMap ) \
        INIT_FUNCTION( bool, Namespace, GetItem, ThreadSafeMap, unsigned long, void* ) \
        INIT_FUNCTION( void, Namespace, RunForAll, ThreadSafeMap, void (*)( void* ) )
        
INIT_NAMESPACE_INTERFACE( ThreadSafeMaps, THREAD_SAFE_MAP_INTERFACE )


ThreadSafeMap ThreadSafeMaps_Create( enum TSMapKeyType keyType, size_t itemSize )
{
  ThreadSafeMap map = (ThreadSafeMap) malloc( sizeof(ThreadSafeMapData) );
  
  map->hashTable = kh_init( RefInt );
  map->keyType = keyType;
  map->itemSize = itemSize;
  map->accessLock = ThreadLocks.Create();
  
  return map;
}

void ThreadSafeMaps_Discard( ThreadSafeMap map )
{
  ThreadLocks.Aquire( map->accessLock );
  for( khint_t dataIndex = kh_begin( map->hashTable ); dataIndex != kh_end( map->hashTable ); dataIndex++ )
  {
    if( kh_exist( map->hashTable, dataIndex ) )
      free( kh_value( map->hashTable, dataIndex ) );
  }
  kh_destroy( RefInt, map->hashTable );
  ThreadLocks.Release( map->accessLock );
  
  ThreadLocks.Discard( map->accessLock );
  
  free( map );
}

size_t ThreadSafeMaps_GetItemsCount( ThreadSafeMap map )
{
  return kh_size( map->hashTable );
}

unsigned long ThreadSafeMaps_SetItem( ThreadSafeMap map, const void* key, void* dataIn )
{
  unsigned long hash = 0;
  int insertionStatus = 0;
  
  if( map->keyType == TSMAP_INT ) hash = (unsigned long) key;
  else if( key != NULL ) hash = (unsigned long) kh_str_hash_func( key );
  
  ThreadLocks.Aquire( map->accessLock );
  
  khint_t index = kh_get( RefInt, map->hashTable, hash );
  if( index == kh_end( map->hashTable ) ) 
  {
    index = kh_put( RefInt, map->hashTable, hash, &insertionStatus );
    kh_value( map->hashTable, index ) = malloc( map->itemSize );
  }
    
  if( dataIn != NULL ) memcpy( kh_value( map->hashTable, index ), dataIn, map->itemSize );
  
  ThreadLocks.Release( map->accessLock );
  
  if( insertionStatus == -1 ) return 0;
  
  return (unsigned long) kh_key( map->hashTable, index );
}

bool ThreadSafeMaps_RemoveItem( ThreadSafeMap map, unsigned long hash )
{
  ThreadLocks.Aquire( map->accessLock );
  
  khint_t index = kh_get( RefInt, map->hashTable, (khint64_t) hash );
  
  if( index == kh_end( map->hashTable ) ) return false;
  
  free( kh_value( map->hashTable, index ) );
  kh_del( RefInt, map->hashTable, index );
  
  ThreadLocks.Release( map->accessLock );
  
  return true;
}

void* ThreadSafeMaps_AquireItem( ThreadSafeMap map, unsigned long hash )
{
  ThreadLocks.Aquire( map->accessLock );
  
  khint_t index = kh_get( RefInt, map->hashTable, (khint64_t) hash );
  
  if( index == kh_end( map->hashTable ) ) 
  {
    ThreadLocks.Release( map->accessLock );
    return NULL;
  }
  
  return kh_value( map->hashTable, index );
}

void ThreadSafeMaps_ReleaseItem( ThreadSafeMap map )
{
  ThreadLocks.Release( map->accessLock );
}

bool ThreadSafeMaps_GetItem( ThreadSafeMap map, unsigned long hash, void* dataOut )
{
  void* ref_data = ThreadSafeMaps_AquireItem( map, hash );
  
  if( ref_data == NULL ) return false;
  
  if( dataOut != NULL ) memcpy( dataOut, ref_data, map->itemSize );
  
  ThreadSafeMaps_ReleaseItem( map );
  
  return true;
}

void ThreadSafeMaps_RunForAll( ThreadSafeMap map, void (*objectOperator)(void*) )
{
  if( map == NULL ) return;
  
  ThreadLocks.Aquire( map->accessLock );
  
  for( khint_t dataIndex = kh_begin( map->hashTable ); dataIndex != kh_end( map->hashTable ); dataIndex++ )
  {
    if( kh_exist( map->hashTable, dataIndex ) )
      objectOperator( kh_value( map->hashTable, dataIndex ) );
  }
  
  ThreadLocks.Release( map->accessLock );
}


#endif /* THREAD_SAFE_DATA_H */
