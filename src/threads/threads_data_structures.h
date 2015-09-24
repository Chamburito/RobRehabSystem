#ifndef THREADS_DATA_STRUCT_H
#define THREADS_DATA_STRUCT_H

#ifdef WIN32
  #include "threads_windows.h"
#else
  #include "threads_unix.h"
#endif

#include "klib/khash.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD SAFE QUEUE                                       /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _DataQueue
{
  void** cache;
  size_t first, last, maxLength;
  size_t itemSize;
  ThreadLock accessLock;
  Semaphore* countLock;
}
DataQueue;

DataQueue* DataQueue_Init( size_t maxLength, size_t itemSize )
{
  DataQueue* queue = (DataQueue*) malloc( sizeof(DataQueue) );
  
  queue->maxLength = maxLength;
  queue->itemSize = itemSize;
  
  queue->cache = (void**) calloc( queue->maxLength, sizeof(void*) );
  for( size_t i = 0; i < queue->maxLength; i++ )
    queue->cache[ i ] = (void*) malloc( itemSize );
  
  queue->first = queue->last = 0;
  
  queue->accessLock = ThreadLocks_Create();
  queue->countLock = Semaphores_Create( 0, queue->maxLength );
  
  return queue;
}

void DataQueue_End( DataQueue* queue )
{
  if( queue != NULL )
  {
    for( size_t i = 0; i < queue->maxLength; i++ )
      free( queue->cache[ i ] );
    free( queue->cache );
    
    ThreadLocks_Discard( queue->accessLock );
    Semaphores_Discard( queue->countLock );

    free( queue );
    queue = NULL;
  }
}

extern inline size_t DataQueue_GetItemsCount( DataQueue* queue )
{
  return ( queue->last - queue->first );
}

enum QueueReadMode { QUEUE_READ_WAIT, QUEUE_READ_NOWAIT };
int DataQueue_Pop( DataQueue* queue, void* buffer, enum QueueReadMode mode )
{
  static void* dataOut = NULL;

  if( mode == QUEUE_READ_NOWAIT && DataQueue_GetItemsCount( queue ) == 0 )
    return -1;
  
  Semaphores_Decrement( queue->countLock );
  
  ThreadLocks_Aquire( queue->accessLock );
  
  dataOut = queue->cache[ queue->first % queue->maxLength ];
  buffer = memcpy( buffer, dataOut, queue->itemSize );
  queue->first++;
  
  ThreadLocks_Release( queue->accessLock );
  
  return 0;
}

enum QueueWriteMode { QUEUE_APPEND_WAIT, QUEUE_APPEND_OVERWRITE, QUEUE_APPEND_FLUSH };
int DataQueue_Push( DataQueue* queue, void* buffer, enum QueueWriteMode mode )
{
  static void* dataIn = NULL;
  
  if( mode == QUEUE_APPEND_WAIT ) Semaphores_Increment( queue->countLock );
  
  ThreadLocks_Aquire( queue->accessLock );
  
  dataIn = queue->cache[ queue->last % queue->maxLength ];
  memcpy( dataIn, buffer, queue->itemSize );
  if( DataQueue_GetItemsCount( queue ) == queue->maxLength ) queue->first++;
  queue->last++;
  
  ThreadLocks_Release( queue->accessLock );
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
  for( size_t position; position < LIST_LENGTH_INCREMENT; position++ )
    list->data[ position ].data = malloc( itemSize );
  
  list->length = LIST_LENGTH_INCREMENT;
  list->itemsCount = list->insertCount = 0;
  list->itemSize = itemSize;
  
  list->accessLock = ThreadLocks_Create();
}

void DataList_End( DataList* list )
{
  if( list != NULL )
  {
    //for( size_t position = 0; position < list->length; position++ )
    //  free( list->data[ position ] );
    free( list->data );
    
    ThreadLocks_Discard( list->accessLock );

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
  ThreadLocks_Aquire( list->accessLock );
  
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
  
  ThreadLocks_Release( list->accessLock );
  
  return list->insertCount;
}

void DataList_Remove( DataList* list, size_t index )
{
  ThreadLocks_Aquire( list->accessLock );
  
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
  
  ThreadLocks_Release( list->accessLock );
}

void* DataList_GetValue( DataList* list, size_t index )
{
  //ThreadLocks_Aquire( list->accessLock );
  
  Item comparisonItem = { index, NULL };
  
  void* foundItem = bsearch( (void*) &comparisonItem, (void*) list->data, list->length, list->itemSize, ListCompare );
  
  //ThreadLocks_Release( list->accessLock );
  
  return foundItem;
}


#endif /* THREADS_DATA_STRUCT_H */
