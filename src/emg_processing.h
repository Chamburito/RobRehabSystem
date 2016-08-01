#ifndef EMG_PROCESSING_H
#define EMG_PROCESSING_H

#include "namespaces.h"

enum EMGProcessingPhase { EMG_PROCESSING_MEASUREMENT, EMG_PROCESSING_CALIBRATION, EMG_PROCESSING_OFFSET, EMG_PROCESSING_SAMPLING, EMG_PROCESSING_PHASES_NUMBER };

enum EMGMuscleGain { MUSCLE_GAIN_ACTIVATION, MUSCLE_GAIN_LENGTH, MUSCLE_GAIN_ARM, MUSCLE_GAIN_PENATION, MUSCLE_GAIN_FORCE, MUSCLE_GAINS_NUMBER };

#define EMG_JOINT_INVALID_ID 0


#define EMG_PROCESSING_FUNCTIONS( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( int, Namespace, InitJoint, const char* ) \
        INIT_FUNCTION( void, Namespace, EndJoint, int ) \
        INIT_FUNCTION( double, Namespace, GetJointMuscleSignal, int, size_t ) \
        INIT_FUNCTION( double, Namespace, GetJointTorque, int, double, double ) \
        INIT_FUNCTION( double, Namespace, GetJointStiffness, int, double ) \
        INIT_FUNCTION( double, Namespace, SetJointGain, int, double ) \
        INIT_FUNCTION( void, Namespace, SetProcessingPhase, int, enum EMGProcessingPhase ) \
        INIT_FUNCTION( size_t, Namespace, GetJointMusclesCount, int ) \
        INIT_FUNCTION( double, Namespace, SetJointMuscleGain, int, size_t, enum EMGMuscleGain, double ) \
        INIT_FUNCTION( double, Namespace, GetJointMuscleTorque, int, size_t, double, double )

DECLARE_NAMESPACE_INTERFACE( EMGProcessing, EMG_PROCESSING_FUNCTIONS )


#endif /* EMG_PROCESSING_H */
