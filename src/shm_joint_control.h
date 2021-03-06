#ifndef SHM_JOINT_CONTROL_H
#define SHM_JOINT_CONTROL_H

enum { SHM_JOINT_POSITION, SHM_JOINT_FORCE, SHM_JOINT_STIFFNESS,
       SHM_JOINT_EMG_1, SHM_JOINT_EMG_2, SHM_JOINT_EMG_3, SHM_JOINT_EMG_4, SHM_JOINT_EMG_5, SHM_JOINT_FLOATS_NUMBER };

#define JOINT_DATA_BLOCK_SIZE SHM_JOINT_FLOATS_NUMBER * sizeof(float)

#endif // SHM_JOINT_CONTROL_H
