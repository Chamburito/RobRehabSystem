#ifndef ROBOTS_H
#define ROBOTS_H

#include "interfaces.h"

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
        INIT_FUNCTION( void, namespace, Disable, int ) \
        INIT_FUNCTION( Joint, namespace, GetJoint, int, size_t ) \
        INIT_FUNCTION( Axis, namespace, GetAxis, int, size_t ) \
        INIT_FUNCTION( char*, namespace, GetJointName, int, size_t ) \
        INIT_FUNCTION( char*, namespace, GetAxisName, int, size_t ) \
        INIT_FUNCTION( double*, namespace, GetAxisMeasuresList, Axis ) \
        INIT_FUNCTION( double*, namespace, GetJointMeasuresList, Joint ) \
        INIT_FUNCTION( double, namespace, SetJointSetpoint, Joint, enum ControlVariables, double ) \
        INIT_FUNCTION( double, namespace, SetAxisSetpoint, Axis, enum ControlVariables, double ) \
        INIT_FUNCTION( Actuator, namespace, GetJointActuator, Joint ) \
        INIT_FUNCTION( size_t, namespace, GetJointsNumber, int ) \
        INIT_FUNCTION( size_t, namespace, GetAxesNumber, int )

DECLARE_NAMESPACE_INTERFACE( Robots, ROBOT_INTERFACE )


#endif /* ROBOTS_H */ 
