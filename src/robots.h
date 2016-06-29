#ifndef ROBOTS_H
#define ROBOTS_H

#include "namespaces.h"

#include "actuators.h"
#include "control_definitions.h"

/////////////////////////////////////////////////////////////////////////////////
/////                               INTERFACE                               /////
/////////////////////////////////////////////////////////////////////////////////

#define ROBOT_INVALID_ID -1

typedef struct _JointData JointData;
typedef JointData* Joint;

typedef struct _AxisData AxisData;
typedef AxisData* Axis;

typedef struct _RobotData RobotData;
typedef RobotData* Robot;

#define ROBOT_INTERFACE( namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( int, namespace, Init, const char* ) \
        INIT_FUNCTION( void, namespace, End, int ) \
        INIT_FUNCTION( bool, namespace, Enable, int ) \
        INIT_FUNCTION( bool, namespace, Disable, int ) \
        INIT_FUNCTION( bool, namespace, SetControlState, int, enum ControlState ) \
        INIT_FUNCTION( Joint, namespace, GetJoint, int, size_t ) \
        INIT_FUNCTION( Axis, namespace, GetAxis, int, size_t ) \
        INIT_FUNCTION( char*, namespace, GetJointName, int, size_t ) \
        INIT_FUNCTION( char*, namespace, GetAxisName, int, size_t ) \
        INIT_FUNCTION( double, namespace, GetJointMeasure, Joint, enum ControlVariable ) \
        INIT_FUNCTION( double, namespace, GetAxisMeasure, Axis, enum ControlVariable ) \
        INIT_FUNCTION( double, namespace, SetJointSetpoint, Joint, enum ControlVariable, double ) \
        INIT_FUNCTION( double, namespace, SetAxisSetpoint, Axis, enum ControlVariable, double ) \
        INIT_FUNCTION( size_t, namespace, GetJointsNumber, int ) \
        INIT_FUNCTION( size_t, namespace, GetAxesNumber, int )

DECLARE_NAMESPACE_INTERFACE( Robots, ROBOT_INTERFACE )


#endif /* ROBOTS_H */ 
