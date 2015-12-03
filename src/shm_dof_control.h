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
  int64_t localControllerKey;
  SHMController sharedData;
}
SHMDoFControllerData;

typedef SHMDoFControllerData* SHMDoFController;

/*enum { SHM_COMMAND_DISABLE = 1, SHM_STATE_DISABLED = SHM_COMMAND_DISABLE, SHM_COMMAND_ENABLE, SHM_STATE_ENABLED = SHM_COMMAND_ENABLE, 
       SHM_COMMAND_RESET, SHM_STATE_ERROR = SHM_COMMAND_RESET, SHM_COMMAND_OFFSET, SHM_COMMAND_CALIBRATE, SHM_AXIS_BYTE_END };*/

#endif // SHM_DOF_CONTROL_H
