#ifndef SHM_DOF_CONTROL_H
#define SHM_DOF_CONTROL_H

#include "shm_control.h"

#define INT32_MASK_1 0x00000000FFFFFFFF
#define INT32_MASK_2 0xFFFFFFFF00000000

#define GET_INT32_1( key ) (key & INT32_MASK_1)
#define SET_INT32_1( key, value ) key = ( (key & INT32_MASK_2) | (value & INT32_MASK_1) )

#define GET_INT32_2( key ) ( (key & INT32_MASK_2) >> 32 )
#define SET_INT32_2( key, value ) key = ( (key & INT32_MASK_1) | ( (value & INT32_MASK_1) << 32 ) )

typedef struct _SHMDoFControllerData
{
  void* ref_localDoF;
  SHMController sharedData;
}
SHMDoFControllerData;

typedef SHMDoFControllerData* SHMDoFController;

#endif // SHM_DOF_CONTROL_H
