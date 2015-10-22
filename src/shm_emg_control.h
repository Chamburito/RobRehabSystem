#ifndef SHM_EMG_CONTROL_H
#define SHM_EMG_CONTROL_H

#include "shm_axis_control.h"

//const char* SHM_EMG_BASE_KEY = "emg_joints";

enum { SHM_EMG_TORQUE = SHM_AXIS_FLOATS_MAX_NUMBER, SHM_EMG_STIFFNESS, SHM_EMG_FLOATS_MAX_NUMBER };

enum { SHM_EMG_CALIBRATION = SHM_AXIS_BYTE_END, SHM_EMG_SAMPLING, SHM_EMG_MEASUREMENT, SHM_EMG_BYTE_END };

#endif // SHM_EMG_CONTROL_H
