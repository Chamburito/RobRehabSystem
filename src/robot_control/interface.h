#ifndef ROBOT_CONTROL_INTERFACE_H
#define ROBOT_CONTROL_INTERFACE_H

#include "interfaces.h"
#include "control_definitions.h"

#define ROBOT_CONTROL_FUNCTIONS( interface, function_init ) \
        function_init( Controller, interface, InitController, const char* ) \
        function_init( void, interface, EndController, Controller ) \ 
        function_init( size_t, interface, GetDoFsNumber, Controller ) \
        function_init( const char**, interface, GetJointNamesList, Controller ) \
        function_init( const char**, interface, GetAxisNamesList, Controller ) \
        function_init( void, interface, RunStep, Controller, double**, double**, double**, double** )

DEFINE_INTERFACE( RobotControl, ROBOT_CONTROL_FUNCTIONS )

#endif  // ROBOT_CONTROL_INTERFACE_H
