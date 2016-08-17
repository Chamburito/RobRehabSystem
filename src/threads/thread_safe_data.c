////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016 Leonardo José Consoni                                  //
//                                                                            //
//  This file is part of RobRehabSystem.                                      //
//                                                                            //
//  RobRehabSystem is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobRehabSystem is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


#include "klib/khash.h"

#include <stdbool.h>
#include <stdlib.h>

#include "debug/sync_debug.h"

#include "threads/threading.h"

#include "threads/thread_safe_data.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD SAFE QUEUE                                       /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

struct _ThreadSafeQueueData
{
  void** cache;
  size_t first, last, maxLength;
  size_t itemSize;
  ThreadLock accessLock;
  Semaphore countLock;
};


DEFINE_NAMESPACE_INTERFACE( ThreadSafeQueues, THREAD_SAFE_QUEUE_INTERFACE )


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
  queue->countLock = Semaphores.Create( 0, queue->maxLength );
  
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
    Semaphores.Discard( queue->countLock );

    free( queue );
    queue = NULL;
  }
}

size_t ThreadSafeQueues_GetItemsCount( ThreadSafeQueue queue )
{
  return ( queue->last - queue->first );
}

bool ThreadSafeQueues_Enqueue( ThreadSafeQueue queue, void* buffer, enum TSQueueAccessMode mode )
{
  static void* dataIn = NULL;
  
  if( mode == TSQUEUE_WAIT ) Semaphores.Increment( queue->countLock );
  
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
  
  Semaphores.Decrement( queue->countLock );
  
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

struct _ThreadSafeListData
{
  Item* data;
  size_t length, itemsCount, insertCount;
  size_t itemSize;
  ThreadLock accessLock;
};


DEFINE_NAMESPACE_INTERFACE( ThreadSafeLists, THREAD_SAFE_LIST_INTERFACE )


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

KHASH_MAP_INIT_INT64( RefInt, void* )

struct _ThreadSafeMapData
{
  khash_t( RefInt )* hashTable;
  enum TSMapKeyType keyType;
  size_t itemSize;
  ThreadLock accessLock;
};


DEFINE_NAMESPACE_INTERFACE( ThreadSafeMaps, THREAD_SAFE_MAP_INTERFACE )


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
  
  DEBUG_PRINT( "new item hash: %lx", hash );
  
  //ThreadLocks.Aquire( map->accessLock );
  
  DEBUG_PRINT( "access lock %p aquired", map->accessLock );
  
  khint_t index = kh_get( RefInt, map->hashTable, hash );
  if( index == kh_end( map->hashTable ) ) 
  {
    index = kh_put( RefInt, map->hashTable, hash, &insertionStatus );
    kh_value( map->hashTable, index ) = malloc( map->itemSize );
  }
    
  if( dataIn != NULL ) memcpy( kh_value( map->hashTable, index ), dataIn, map->itemSize );
  
  if( insertionStatus == -1 ) hash = 0;
  else hash = (unsigned long) kh_key( map->hashTable, index );
  
  //ThreadLocks.Release( map->accessLock );
  
  DEBUG_PRINT( "access lock %p released", map->accessLock );
  
  return hash;
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
