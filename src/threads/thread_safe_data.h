#ifndef THREAD_SAFE_DATA_H
#define THREAD_SAFE_DATA_H

#ifdef WIN32
  #include "threads_windows.h"
#else
  #include "threads_unix.h"
#endif

#include "interface.h"

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

enum QueueReadMode { QUEUE_READ_WAIT, QUEUE_READ_NOWAIT };
enum QueueWriteMode { QUEUE_APPEND_WAIT, QUEUE_APPEND_OVERWRITE, QUEUE_WRITE_FLUSH };

#define THREAD_SAFE_QUEUE_FUNCTIONS( namespace, function_init ) \
        function_init( ThreadSafeQueue, namespace, Create, size_t, size_t ) \
        function_init( void, namespace, Discard, ThreadSafeQueue ) \
        function_init( size_t, namespace, GetItemsCount, ThreadSafeQueue ) \
        function_init( bool, namespace, Dequeue, ThreadSafeQueue, void*, enum QueueReadMode ) \
        function_init( void, namespace, Enqueue, ThreadSafeQueue, void*, enum QueueWriteMode )

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

inline size_t ThreadSafeQueues_GetItemsCount( ThreadSafeQueue queue )
{
  return ( queue->last - queue->first );
}

bool ThreadSafeQueues_Dequeue( ThreadSafeQueue queue, void* buffer, enum QueueReadMode mode )
{
  if( mode == QUEUE_READ_NOWAIT && ThreadSafeQueues_GetItemsCount( queue ) == 0 )
    return false;
  
  Semaphores_Decrement( queue->countLock );
  
  ThreadLocks.Aquire( queue->accessLock );
  
  void* dataOut = queue->cache[ queue->first % queue->maxLength ];
  buffer = memcpy( buffer, dataOut, queue->itemSize );
  queue->first++;
  
  ThreadLocks.Release( queue->accessLock );
  
  return true;
}

void ThreadSafeQueues_Enqueue( ThreadSafeQueue queue, void* buffer, enum QueueWriteMode mode )
{
  if( mode == QUEUE_WRITE_FLUSH )
  {
    Semaphores_SetCount( queue->countLock, 0 );
    queue->first = queue->last = 0;
  }
  
  if( mode != QUEUE_APPEND_OVERWRITE || Semaphores_GetCount( queue->countLock ) < queue->maxLength )
    Semaphores_Increment( queue->countLock );
  
  ThreadLocks.Aquire( queue->accessLock );
  
  void* dataIn = queue->cache[ queue->last % queue->maxLength ];
  memcpy( dataIn, buffer, queue->itemSize );
  if( ThreadSafeQueues_GetItemsCount( queue ) == queue->maxLength ) queue->first++;
  queue->last++;
  
  ThreadLocks.Release( queue->accessLock );
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

typedef struct _DataList
{
  Item* data;
  size_t length, itemsCount, insertCount;
  size_t itemSize;
  ThreadLock accessLock;
}
DataList;

DataList* DataList_Init( size_t itemSize )
{
  DataList* list = (DataList*) malloc( sizeof(DataList) );
  
  list->data = (Item*) malloc( LIST_LENGTH_INCREMENT * sizeof(Item) );
  for( size_t position = 0; position < LIST_LENGTH_INCREMENT; position++ )
    list->data[ position ].data = malloc( itemSize );
  
  list->length = LIST_LENGTH_INCREMENT;
  list->itemsCount = list->insertCount = 0;
  list->itemSize = itemSize;
  
  list->accessLock = ThreadLocks.Create();
  
  return list;
}

void DataList_End( DataList* list )
{
  if( list != NULL )
  {
    //for( size_t position = 0; position < list->length; position++ )
    //  free( list->data[ position ] );
    free( list->data );
    
    ThreadLocks.Discard( list->accessLock );

    free( list );
    list = NULL;
  }
}

int ListCompare( const void* ref_item_1, const void* ref_item_2 )
{
  return ( ((Item*) ref_item_1)->index - ((Item*) ref_item_2)->index );
}

size_t DataList_Insert( DataList* list, void* dataIn )
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

void DataList_Remove( DataList* list, size_t index )
{
  ThreadLocks.Aquire( list->accessLock );
  
  if( list->insertCount > 0 )
  {  
    Item comparisonItem = { index, NULL };
    
    Item* foundItem = (Item*) bsearch( (void*) &comparisonItem, (void*) list->data, list->length, list->itemSize, ListCompare );
    
    if( foundItem != NULL )
    {
      free( foundItem->data );
      foundItem->index = INFINITE;
      
      qsort( (void*) list->data, list->length, list->itemSize, ListCompare );
      
      list->itemsCount--;
      
      if( list->itemsCount < list->length - LIST_LENGTH_INCREMENT )
      {
        list->length -= LIST_LENGTH_INCREMENT;
        list->data = (Item*) realloc( list->data, list->length * sizeof(Item) );
      }
    }
  }
  
  ThreadLocks.Release( list->accessLock );
}

void* DataList_GetValue( DataList* list, size_t index )
{
  //ThreadLocks.Aquire( list->accessLock );
  
  Item comparisonItem = { index, NULL };
  
  void* foundItem = bsearch( (void*) &comparisonItem, (void*) list->data, list->length, list->itemSize, ListCompare );
  
  //ThreadLocks.Release( list->accessLock );
  
  return foundItem;
}


#endif /* THREAD_SAFE_DATA_H */
