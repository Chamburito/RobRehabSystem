#ifndef ROBOT_CONTROL_INTERFACE_H
#define ROBOT_CONTROL_INTERFACE_H

#include "modules.h"
#include "control_definitions.h"

#define ROBOT_CONTROL_INTERFACE( Interface, INIT_FUNCTION ) \
        INIT_FUNCTION( Controller, Interface, InitController, const char*, const char* ) \
        INIT_FUNCTION( void, Interface, EndController, Controller ) \
        INIT_FUNCTION( size_t, Interface, GetJointsNumber, Controller ) \
        INIT_FUNCTION( char**, Interface, GetJointNamesList, Controller ) \
        INIT_FUNCTION( size_t, Interface, GetAxesNumber, Controller ) \
        INIT_FUNCTION( char**, Interface, GetAxisNamesList, Controller ) \
        INIT_FUNCTION( void, Interface, SetControlState, Controller, enum ControlState ) \
        INIT_FUNCTION( void, Interface, RunControlStep, Controller, double**, double**, double**, double** )

#endif  // ROBOT_CONTROL_INTERFACE_H
