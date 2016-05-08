#ifndef THREAD_SAFE_DATA_H
#define THREAD_SAFE_DATA_H

#include "namespaces.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD SAFE QUEUE                                       /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _ThreadSafeQueueData ThreadSafeQueueData;
typedef ThreadSafeQueueData* ThreadSafeQueue;

enum TSQueueAccessMode { TSQUEUE_WAIT, TSQUEUE_NOWAIT };

#define THREAD_SAFE_QUEUE_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( ThreadSafeQueue, Namespace, Create, size_t, size_t ) \
        INIT_FUNCTION( void, Namespace, Discard, ThreadSafeQueue ) \
        INIT_FUNCTION( size_t, Namespace, GetItemsCount, ThreadSafeQueue ) \
        INIT_FUNCTION( bool, Namespace, Enqueue, ThreadSafeQueue, void*, enum TSQueueAccessMode ) \
        INIT_FUNCTION( bool, Namespace, Dequeue, ThreadSafeQueue, void*, enum TSQueueAccessMode )

DECLARE_NAMESPACE_INTERFACE( ThreadSafeQueues, THREAD_SAFE_QUEUE_INTERFACE )


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD SAFE LIST                                        /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _ThreadSafeListData ThreadSafeListData;
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

DECLARE_NAMESPACE_INTERFACE( ThreadSafeLists, THREAD_SAFE_LIST_INTERFACE )


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      THREAD SAFE MAP                                        /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

enum TSMapKeyType { TSMAP_INT, TSMAP_STR };

typedef struct _ThreadSafeMapData ThreadSafeMapData;
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
        
DECLARE_NAMESPACE_INTERFACE( ThreadSafeMaps, THREAD_SAFE_MAP_INTERFACE )


#endif /* THREAD_SAFE_DATA_H */
