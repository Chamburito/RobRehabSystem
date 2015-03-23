#ifndef THREADS_DATA_QUEUE_H
#define THREADS_DATA_QUEUE_H

#ifdef WIN32
  #include "threads_windows.h"
#else
  #include "threads_unix.h"
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD SAFE QUEUE                                       /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _DataQueue
{
  void** cache;
  size_t first, last, maxLength;
  size_t itemSize;
  ThreadLock accessLock;
  Semaphore countLock;
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
  
  queue->accessLock = ThreadLock_Create();
  queue->countLock = Semaphore_Create( 0, queue->maxLength );
  
  return queue;
}

void DataQueue_End( DataQueue* queue )
{
  if( queue != NULL )
  {
    for( size_t i = 0; i < queue->maxLength; i++ )
      free( queue->cache[ i ] );
    free( queue->cache );
    
    ThreadLock_Discard( queue->accessLock );
    Semaphore_Discard( queue->countLock );

    free( queue );
    queue = NULL;
  }
}

extern inline size_t DataQueue_ItemCount( DataQueue* queue )
{
  return ( queue->last - queue->first );
}

void* DataQueue_Read( DataQueue* queue, void* buffer )
{
  static void* dataOut = NULL;

  Semaphore_Decrement( queue->countLock );
  
  ThreadLock_Aquire( queue->accessLock );
  
  dataOut = queue->cache[ queue->first % queue->maxLength ]; // Always keep access index between 0 and MAX_DATA
  buffer = memcpy( buffer, dataOut, queue->itemSize );
  queue->first++;
  
  ThreadLock_Release( queue->accessLock );
  
  return buffer;
}

enum { WAIT = 0, REPLACE = 1 };

void DataQueue_Write( DataQueue* queue, void* buffer, uint8_t replace )
{
  static void* dataIn = NULL;
  
  if( replace == 0 ) Semaphore_Increment( queue->countLock ); // Replace old data behaviour
  
  ThreadLock_Aquire( queue->accessLock );
  
  dataIn = queue->cache[ queue->last % queue->maxLength ]; // Always keep access index between 0 and MAX_DATA
  memcpy( dataIn, buffer, queue->itemSize );
  if( DataQueue_ItemCount( queue ) == queue->maxLength ) queue->first++;
  queue->last++;
  
  ThreadLock_Release( queue->accessLock );
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
    list->data[ position ].data = (void*) malloc( itemSize );
  
  list->length = LIST_LENGTH_INCREMENT;
  list->itemsCount = list->insertCount = 0;
  list->itemSize = itemSize;
  
  list->accessLock = ThreadLock_Create();
}

void DataList_End( DataList* list )
{
  if( list != NULL )
  {
    for( size_t position = 0; position < list->length; position++ )
      free( list->data[ position ] );
    free( list->data );
    
    ThreadLock_Discard( list->accessLock );

    free( list );
    list = NULL;
  }
}

int ListCompare( void* ref_item_1, void* ref_item_2 );

size_t DataList_Insert( DataList* list, void* dataIn )
{
  ThreadLock_Aquire( list->accessLock );
  
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
  
  ThreadLock_Release( list->accessLock );
  
  return list->insertCount;
}

void DataList_GetValue( DataList* list, size_t index )
{
  
}


#endif /* THREADS_DATA_QUEUE_H */