cmake_minimum_required( VERSION 2.8 )
project( RobRehabSystem )

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR} )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/plugins )

set( CMAKE_C_STANDARD 99 )
set( CMAKE_C_STANDARD_REQUIRED ON )
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

add_executable( RobRehabControl src/robrehab_system.c )
add_library( JSON MODULE src/data_io/json_parser.c src/klib/kson.c )
add_library( Dummy MODULE src/signal_io/dummy_io.c )
add_library( NICANEPOS MODULE src/signal_io/ni_can_epos/signal_io.c )
add_library( ForcePIControl MODULE src/actuator_control/force_pi_control.c )
add_library( SimpleJoint MODULE src/robot_control/simple_joint.c )
# add_library( CppPlugin MODULE src/robot_control/cpp_plugin.cpp )
add_library( OpenSimModel MODULE src/robot_control/open_sim_model.cpp )

# add_executable( RobRehabServer src/robrehab_system.c )

include_directories( ${CMAKE_SOURCE_DIR}/src )
link_directories( ${CMAKE_SOURCE_DIR}/libs )

target_compile_definitions( RobRehabControl PUBLIC -DROBREHAB_CONTROL -DDEBUG )
target_link_libraries( RobRehabControl -lrt -lm -ldl -lblas -llapack -lpthread )

set_target_properties( JSON PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/data_io/ )
set_target_properties( JSON PROPERTIES PREFIX "" )
target_include_directories( JSON PUBLIC src/data_io/ )

set_target_properties( Dummy PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/signal_io/ )
set_target_properties( Dummy PROPERTIES PREFIX "" )
target_include_directories( Dummy PUBLIC src/signal_io/ )

set_target_properties( NICANEPOS PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/signal_io/ )
set_target_properties( NICANEPOS PROPERTIES PREFIX "" )
target_include_directories( NICANEPOS PUBLIC src/signal_io/ )

set_target_properties( ForcePIControl PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/actuator_control/ )
set_target_properties( ForcePIControl PROPERTIES PREFIX "" )
target_include_directories( ForcePIControl PUBLIC src/actuator_control/ )

set_target_properties( SimpleJoint PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/robot_control/ )
set_target_properties( SimpleJoint PROPERTIES PREFIX "" )
target_include_directories( SimpleJoint PUBLIC src/robot_control/ )

# set_target_properties( CppPlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/robot_control/ )
# set_target_properties( CppPlugin PROPERTIES PREFIX "" )
# target_include_directories( CppPlugin PUBLIC src/robot_control/ )

set_target_properties( OpenSimModel PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/robot_control/ )
set_target_properties( OpenSimModel PROPERTIES PREFIX "" )
target_include_directories( OpenSimModel PUBLIC src/robot_control/ src/robot_control/SimTK/simbody )
target_link_libraries( OpenSimModel libSimTKcommon_d.so libSimTKmath_d.so libSimTKsimbody_d.so libosimActuators_d.so libosimAnalyses_d.so libosimCommon_d.so libosimLepton_d.so libosimSimulation_d.so libosimTools_d.so )

# target_compile_definitions( RobRehabControl PUBLIC -DROBREHAB_SERVER -DDEBUG )
# target_link_libraries( RobRehabControl -lpthread )

# install( TARGETS RobRehabGui RobRehabServer RUNTIME DESTINATION . )
