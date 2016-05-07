/// @file shared_memory.h
/// @brief Memory sharing functions.

#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include "interfaces.h"

// Access permissions
#define SHM_READ 0xF0
#define SHM_WRITE 0x0F

#define SHARED_OBJECT_PATH_MAX_LENGTH 256

/// Functions declaration macro   
#define SHARED_MEMORY_INTERFACE( Namespace, INIT_FUNCTION )                        \
        INIT_FUNCTION( void*, Namespace, CreateObject, const char*, size_t, uint8_t )  \
        INIT_FUNCTION( void, Namespace, DestroyObject, void* )

DECLARE_NAMESPACE_INTERFACE( SharedObjects, SHARED_MEMORY_INTERFACE )

/// @struct SharedObjects
/// @brief Function pointer struct as our shared memory interface 

/// @fn CreateObject                                                                                  
/// @brief Creates shared memory area and returns its pointer                                                
/// @param mappingName name of the shared memory area (mapped file)                                   
/// @param objectSize size in bytes of the shared memory area                                         
/// @param flags bitfield containing access permissions ( read-only, write-only or read-write )       
/// @return generic (void*) pointer to the created memory area (returns (void*) -1 when fails)  

/// @fn DestroyObject
/// Discards shared memory area and remove its pointer from the hash table                              
/// @param sharedObject pointer to the shared memory area                                               

#endif // SHARED_MEMORY_H
