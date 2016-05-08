#ifndef SHM_CONTROL_H
#define SHM_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#ifdef __unix__
  #include <unistd.h>
#endif

#include "namespaces.h"


enum SHMControlTypes { SHM_CONTROL_OUT, SHM_CONTROL_IN, SHM_CONTROL_TYPES_NUMBER };

#define SHM_CONTROL_MASK_SIZE 32
#define SHM_CONTROL_MAX_DATA_SIZE ( 8 * SHM_CONTROL_MASK_SIZE )
#define SHARED_VARIABLE_NAME_MAX_LENGTH 128

//const bool SHM_CONTROL_PEEK = false;
//const bool SHM_CONTROL_REMOVE = true;

#define SHM_CONTROL_BIT_INDEX( index ) ( 1 << (index) )
#define SHM_CONTROL_SET_BIT( field, index ) ( (field) |= SHM_CONTROL_BIT_INDEX( index ) )
#define SHM_CONTROL_CLR_BIT( field, index ) ( (field) &= (~SHM_CONTROL_BIT_INDEX( index )) )
#define SHM_CONTROL_IS_BIT_SET( field, index ) ( (field) & SHM_CONTROL_BIT_INDEX( index ) )

//#define SHM_CONTROL_SET_BYTE( field, index, byte ) ( (field) |= ( ( (byte) << index * 8 ) ) )
//#define SHM_CONTROL_GET_BYTE( field, index ) ( ( (field) >> index * 8 ) & 0xFF )


/////////////////////////////////////////////////////////////////////////////
/////                             INTERFACE                             /////
/////////////////////////////////////////////////////////////////////////////

typedef struct _SHMControlData SHMControlData;
typedef SHMControlData* SHMController;

#define SHM_CONTROL_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( SHMController, Namespace, InitData, const char*, enum SHMControlTypes ) \
        INIT_FUNCTION( void, Namespace, EndData, SHMController ) \
        INIT_FUNCTION( bool, Namespace, GetData, SHMController, void*, size_t, size_t ) \
        INIT_FUNCTION( bool, Namespace, SetData, SHMController, void*, size_t, size_t ) \
        INIT_FUNCTION( uint8_t, Namespace, GetMaskByte, SHMController, size_t ) \
        INIT_FUNCTION( uint8_t, Namespace, SetMaskByte, SHMController, size_t, uint8_t )

DECLARE_NAMESPACE_INTERFACE( SHMControl, SHM_CONTROL_INTERFACE )


#endif // SHM_CONTROL_H
