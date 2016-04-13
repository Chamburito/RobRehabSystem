cmake_minimum_required( VERSION 2.8 )
project( RobRehabSystem )

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR} )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/plugins )

set( CMAKE_C_STANDARD_REQUIRED 99 )
set( CMAKE_CXX_STANDARD_REQUIRED 11 )

#find_package( PythonLibs REQUIRED )
#find_package( Threads REQUIRED )

add_executable( RobRehabControl src/robrehab_system.c )
add_library( JSON MODULE src/data_io/json_parser.c src/klib/kson.c )
add_library( NICANEPOS MODULE src/signal_io/dummy_io.c )
add_library( SimpleJoint MODULE src/robot_control/simple_joint.c )
add_library( ForcePIControl MODULE src/actuator_control/force_pi_control.c )

# add_executable( RobRehabServer src/robrehab_system.c )

include_directories( src )

target_compile_definitions( RobRehabControl PUBLIC -DROBREHAB_CONTROL -DDEBUG )
target_link_libraries( RobRehabControl -lrt -lm -ldl -lblas -llapack -lpthread )

set_target_properties( JSON PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/data_io/ )
set_target_properties( JSON PROPERTIES PREFIX "" )
target_include_directories( JSON PUBLIC src/data_io/ )
target_compile_options( JSON PUBLIC -shared -fPIC )

set_target_properties( NICANEPOS PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/signal_io/ )
set_target_properties( NICANEPOS PROPERTIES PREFIX "" )
target_include_directories( NICANEPOS PUBLIC src/signal_io/ )
target_compile_options( NICANEPOS PUBLIC -shared -fPIC )

set_target_properties( SimpleJoint PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/robot_control/ )
set_target_properties( SimpleJoint PROPERTIES PREFIX "" )
target_include_directories( SimpleJoint PUBLIC src/robot_control/ )
target_compile_options( SimpleJoint PUBLIC -shared -fPIC -lm )

set_target_properties( ForcePIControl PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/actuator_control/ )
set_target_properties( ForcePIControl PROPERTIES PREFIX "" )
target_include_directories( ForcePIControl PUBLIC src/actuator_control/ )
target_compile_options( ForcePIControl PUBLIC -shared -fPIC -lm )

# target_compile_definitions( RobRehabControl PUBLIC -DROBREHAB_SERVER -DDEBUG )
# target_link_libraries( RobRehabControl -lpthread )

# install( TARGETS RobRehabGui RobRehabServer RUNTIME DESTINATION . )
