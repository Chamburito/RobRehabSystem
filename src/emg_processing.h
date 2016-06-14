#ifndef EMG_PROCESSING_H
#define EMG_PROCESSING_H

#include "namespaces.h"

#include "signal_processing.h"

//enum { MUSCLE_ACTIVE_FORCE, MUSCLE_PASSIVE_FORCE, MUSCLE_MOMENT_ARM, MUSCLE_NORM_LENGTH, MUSCLE_PENATION_ANGLE, MUSCLE_CURVES_NUMBER };
enum EMGMuscleGains { MUSCLE_GAIN_ACTIVATION, MUSCLE_GAIN_LENGTH, MUSCLE_GAIN_ARM, MUSCLE_GAIN_PENATION, MUSCLE_GAIN_FORCE, MUSCLE_GAINS_NUMBER };

typedef struct _EMGMuscleData EMGMuscleData;
typedef EMGMuscleData* EMGMuscle;

typedef struct _EMGJointData EMGJointData;
typedef EMGJointData* EMGJoint;

#define EMG_JOINT_INVALID_ID 0


#define EMG_PROCESSING_FUNCTIONS( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( int, Namespace, InitJoint, const char* ) \
        INIT_FUNCTION( void, Namespace, EndJoint, int ) \
        INIT_FUNCTION( double, Namespace, GetJointMuscleSignal, int, size_t ) \
        INIT_FUNCTION( double, Namespace, GetJointTorque, int, double ) \
        INIT_FUNCTION( double, Namespace, GetJointStiffness, int, double ) \
        INIT_FUNCTION( double, Namespace, SetJointGain, int, double ) \
        INIT_FUNCTION( void, Namespace, SetProcessingPhase, int, enum SignalProcessingPhase ) \
        INIT_FUNCTION( size_t, Namespace, GetJointMusclesCount, int ) \
        INIT_FUNCTION( double, Namespace, SetJointMuscleGain, int, size_t, enum EMGMuscleGains, double ) \
        INIT_FUNCTION( double, Namespace, GetJointMuscleTorque, int, size_t, double, double )

DECLARE_NAMESPACE_INTERFACE( EMGProcessing, EMG_PROCESSING_FUNCTIONS )


#endif /* EMG_PROCESSING_H */
