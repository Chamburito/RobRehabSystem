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

#define THREAD_SAFE_QUEUE_FUNCTIONS( namespace, function_init ) \
        function_init( ThreadSafeQueue, namespace, Create, size_t, size_t ) \
        function_init( void, namespace, Discard, ThreadSafeQueue ) \
        function_init( size_t, namespace, GetItemsCount, ThreadSafeQueue ) \
        function_init( bool, namespace, Enqueue, ThreadSafeQueue, void*, enum TSQueueAccessMode ) \
        function_init( bool, namespace, Dequeue, ThreadSafeQueue, void*, enum TSQueueAccessMode )

INIT_NAMESPACE_INTERFACE( ThreadSafeQueues, THREAD_SAFE_QUEUE_FUNCTIONS )


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
  size_t index;
  void* data;
}
Item;

typedef struct _TheadSafeListData
{
  Item* data;
  size_t length, itemsCount, insertCount;
  size_t itemSize;
  ThreadLock accessLock;
}
TheadSafeListData;

typedef TheadSafeListData* TheadSafeList;

#define THREAD_SAFE_LIST_FUNCTIONS( namespace, function_init ) \
        function_init( TheadSafeList, namespace, Create, size_t ) \
        function_init( void, namespace, Discard, TheadSafeList ) \
        function_init( size_t, namespace, GetItemsCount, TheadSafeList ) \
        function_init( size_t, namespace, Insert, TheadSafeList, void* ) \
        function_init( bool, namespace, Remove, TheadSafeList, size_t ) \
        function_init( bool, namespace, GetItem, TheadSafeList, size_t, void* ) \
        function_init( void*, namespace, AquireItem, TheadSafeList, size_t ) \
        function_init( void, namespace, ReleaseItem, TheadSafeList ) \
        function_init( bool, namespace, SetItem, TheadSafeList, size_t, void* )

INIT_NAMESPACE_INTERFACE( TheadSafeLists, THREAD_SAFE_LIST_FUNCTIONS )


TheadSafeList TheadSafeLists_Create( size_t itemSize )
{
  TheadSafeList list = (TheadSafeList) malloc( sizeof(TheadSafeListData) );
  
  list->data = (Item*) malloc( LIST_LENGTH_INCREMENT * sizeof(Item) );
  for( size_t position = 0; position < LIST_LENGTH_INCREMENT; position++ )
    list->data[ position ].data = malloc( itemSize );
  
  list->length = LIST_LENGTH_INCREMENT;
  list->itemsCount = list->insertCount = 0;
  list->itemSize = itemSize;
  
  list->accessLock = ThreadLocks.Create();
  
  return list;
}

void TheadSafeLists_Discard( TheadSafeList list )
{
  if( list != NULL )
  {
    free( list->data );
    
    ThreadLocks.Discard( list->accessLock );

    free( list );
    list = NULL;
  }
}

size_t TheadSafeLists_GetItemsCount( TheadSafeList list )
{
  return list->length;
}

int ListCompare( const void* ref_item_1, const void* ref_item_2 )
{
  return ( ((Item*) ref_item_1)->index - ((Item*) ref_item_2)->index );
}

size_t TheadSafeLists_Insert( TheadSafeList list, void* dataIn )
{
  ThreadLocks.Aquire( list->accessLock );
  
  list->insertCount++;
  
  if( list->itemsCount + 1 > list->length )
  {
    list->length += LIST_LENGTH_INCREMENT;
    list->data = (Item*) realloc( list->data, list->length * sizeof(Item) );
    list->data[ list->itemsCount ].data = (void*) malloc( list->itemSize );
  }
  
  list->data[ list->itemsCount ].index = list->insertCount;
  memcpy( list->data[ list->itemsCount ].data, dataIn, list->itemSize );
  
  qsort( (void*) list->data, list->length, list->itemSize, ListCompare );
  
  list->itemsCount++;
  
  ThreadLocks.Release( list->accessLock );
  
  return list->insertCount;
}

bool TheadSafeLists_Remove( TheadSafeList list, size_t index )
{
  ThreadLocks.Aquire( list->accessLock );
  
  if( list->itemsCount == 0 ) return false;
  
  Item comparisonItem = { index, NULL };
  
  Item* foundItem = (Item*) bsearch( (void*) &comparisonItem, (void*) list->data, list->length, list->itemSize, ListCompare );
  TheadSafeListData
  if( foundItem == NULL ) return false;
  
  free( foundItem->data );
  foundItem->index = INFINITE;
  
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

void* TheadSafeLists_AquireItemRef( TheadSafeList list, size_t index )
{
  Item comparisonItem = { index, NULL };
  
  ThreadLocks.Aquire( list->accessLock );
  
  Item* foundItem = (Item*) bsearch( (void*) &comparisonItem, (void*) list->data, list->length, list->itemSize, ListCompare );
  
  if( foundItem == NULL ) return NULL;
  
  return foundItem->data;
}

void TheadSafeLists_ReleaseItemRef( TheadSafeList list )
{
  ThreadLocks.Release( list->accessLock );
}

bool TheadSafeLists_GetItem( TheadSafeList list, size_t index, void* dataOut )
{
  void* foundData = TheadSafeLists_AquireItemRef( list, index );
  
  if( foundData == NULL ) return false;
  
  if( dataOut != NULL ) memcpy( dataOut, foundData, list->itemSize );
    
  TheadSafeLists_ReleaseItemRef( list );
  
  return true;
}

bool TheadSafeLists_SetItem( TheadSafeList list, size_t index, void* dataIn )
{
  Item comparisonItem = { index, NULL };
  
  ThreadLocks.Aquire( list->accessLock );
  
  Item* foundItem = (Item*) bsearch( (void*) &comparisonItem, (void*) list->data, list->length, list->itemSize, ListCompare );
  
  if( foundItem != NULL && dataIn != NULL ) memcpy( foundItem->data, dataIn, list->itemSize );
  
  ThreadLocks.Release( list->accessLock );
  
  return foundItem->data;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      THREAD SAFE MAP                                        /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

KHASH_MAP_INIT_INT( RefInt, void* )

typedef struct _ThreadSafeMapData
{
  khash_t( RefInt )* hashTable;
  size_t itemSize;
  ThreadLock accessLock;
}
ThreadSafeMapData;

typedef ThreadSafeMapData* ThreadSafeMap;

#define THREAD_SAFE_MAP_FUNCTIONS( namespace, function_init ) \
        function_init( ThreadSafeMap, namespace, Create, size_t ) \
        function_init( void, namespace, Discard, ThreadSafeMap ) \
        function_init( size_t, namespace, GetItemsCount, ThreadSafeMap ) \
        function_init( int, namespace, SetIntKeyItem, ThreadSafeMap, const int, void* ) \
        function_init( int, namespace, SetStrKeyItem, ThreadSafeMap, const char*, void* ) \
        function_init( void*, namespace, AquireItemRef, ThreadSafeMap, int ) \
        function_init( void, namespace, ReleaseItemRef, ThreadSafeMap ) \
        function_init( bool, namespace, GetItem, ThreadSafeMap, int, void* )
        
INIT_NAMESPACE_INTERFACE( ThreadSafeMaps, THREAD_SAFE_MAP_FUNCTIONS )


ThreadSafeMap ThreadSafeMaps_Create( size_t itemSize )
{
  ThreadSafeMap map = (ThreadSafeMap) malloc( sizeof(ThreadSafeMapData) );
  
  map->hashTable = kh_init( RefInt );
  map->itemSize = itemSize;
  map->accessLock = ThreadLocks.Create();
  
  return map;
}

void ThreadSafeMaps_Discard( ThreadSafeMap map )
{
  ThreadLocks.Aquire( map->accessLock );
  if( map->itemSize > sizeof(void*) )
  {
    for( khint_t dataIndex = kh_begin( map->hashTable ); dataIndex != kh_end( map->hashTable ); dataIndex++ )
    {
      if( kh_exist( map->hashTable, dataIndex ) )
        free( kh_value( map->hashTable, dataIndex ) );
    }
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

int ThreadSafeMaps_SetIntKeyItem( ThreadSafeMap map, const int key, void* dataIn )
{
  int insertionStatus = 0;
  
  ThreadLocks.Aquire( map->accessLock );
  
  khint_t index = kh_get( RefInt, map->hashTable, (khint_t) key );
  if( index == kh_end( map->hashTable ) ) index = kh_put( RefInt, map->hashTable, key, &insertionStatus );
  
  if( map->itemSize > sizeof(void*) )
  {
    kh_value( map->hashTable, index ) = malloc( map->itemSize );
    if( dataIn != NULL ) memcpy( kh_value( map->hashTable, index ), dataIn, map->itemSize );
  }
  else
    kh_value( map->hashTable, index ) = dataIn;
  
  ThreadLocks.Release( map->accessLock );
  
  if( insertionStatus == -1 ) return 0;
  
  return (int) kh_key( map->hashTable, index );
}

int ThreadSafeMaps_SetStrKeyItem( ThreadSafeMap map, const char* key, void* dataIn )
{
  if( key == NULL ) return 0;
  
  int hash = kh_str_hash_func( key );
  
  return ThreadSafeMaps_SetIntKeyItem( map, hash, dataIn );
}

void* ThreadSafeMaps_AquireItemRef( ThreadSafeMap map, int key )
{
  ThreadLocks.Aquire( map->accessLock );
  
  khint_t index = kh_get( RefInt, map->hashTable, (khint_t) key );
  
  if( index == kh_end( map->hashTable ) ) return NULL;
  
  return kh_value( map->hashTable, index );
}

void ThreadSafeMaps_ReleaseItemRef( ThreadSafeMap map )
{
  ThreadLocks.Release( map->accessLock );
}

bool ThreadSafeMaps_GetItem( ThreadSafeMap map, int key, void* dataOut )
{
  void* ref_data = ThreadSafeMaps_AquireItemRef( map, key );
  
  if( ref_data == NULL ) return false;
  
  if( dataOut != NULL ) memcpy( dataOut, ref_data, map->itemSize );
  
  ThreadSafeMaps_ReleaseItemRef( map );
  
  return true;
}


#endif /* THREAD_SAFE_DATA_H */
