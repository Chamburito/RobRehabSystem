cmake_minimum_required( VERSION 2.8 )
project( RobRehabSystem )

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR} )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/plugins )

set( CMAKE_C_STANDARD 99 )
set( CMAKE_C_STANDARD_REQUIRED ON )
#set( CMAKE_CXX_STANDARD 11 )
#set( CMAKE_CXX_STANDARD_REQUIRED ON )

find_package( Threads REQUIRED )
#find_package( BLAS REQUIRED )
#find_package( LAPACK REQUIRED )

set( PLATFORM_SOURCES )
if( UNIX )
  list( APPEND PLATFORM_SOURCES src/shared_memory/shm_unix.c src/threads/threads_unix.c src/time/timing_unix.c )
elseif( WIN32 )
  list( APPEND PLATFORM_SOURCES src/shared_memory/shm_windows.c src/threads/threads_windows.c src/time/timing_windows.c )
endif()

include_directories( ${CMAKE_SOURCE_DIR}/src )
#link_directories( ${CMAKE_SOURCE_DIR}/libs )


# (REAL-TIME) CONTROL APPLICATION
#add_executable( RobRehabControl src/robrehab_system.c src/robrehab_control.c src/shm_control.c src/matrices_blas.c src/kalman_filters.c src/robots.c src/actuators.c src/config_parser.c src/debug/data_logging.c src/sensors.c src/signal_processing.c src/motors.c src/curve_interpolation.c ${PLATFORM_SOURCES} )
#target_compile_definitions( RobRehabControl PUBLIC -DROBREHAB_CONTROL -DDEBUG )
#target_link_libraries( RobRehabControl -lm ${CMAKE_DL_LIBS} ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} )
#if( UNIX AND NOT APPLE )
#  target_link_libraries( RobRehabControl -lrt )
#endif()

# (AXIS/JOINT) SERVER
add_executable( RobRehabServer src/robrehab_system.c src/robrehab_network.c src/ip_network/ip_network.c src/ip_network/async_ip_network.c src/threads/thread_safe_data.c src/shm_control.c ${PLATFORM_SOURCES} )
target_include_directories( RobRehabServer PUBLIC src/ip_network/ )
target_compile_definitions( RobRehabServer PUBLIC -DROBREHAB_SERVER -D_DEFAULT_SOURCE=__STRICT_ANSI__ -DDEBUG -DIP_NETWORK_LEGACY )
target_link_libraries( RobRehabServer ${CMAKE_THREAD_LIBS_INIT} )

# PLUGINS/MODULES

add_library( JSON MODULE src/data_io/json_parser.c src/klib/kson.c )
set_target_properties( JSON PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/data_io/ )
set_target_properties( JSON PROPERTIES PREFIX "" )
target_include_directories( JSON PUBLIC src/data_io/ )

# add_library( Dummy MODULE src/signal_io/dummy_io.c )
# set_target_properties( Dummy PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/signal_io/ )
# set_target_properties( Dummy PROPERTIES PREFIX "" )
# target_include_directories( Dummy PUBLIC src/signal_io/ )

#add_library( NICANEPOS MODULE src/signal_io/ni_can_epos/signal_io.c )
#set_target_properties( NICANEPOS PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/signal_io/ )
#set_target_properties( NICANEPOS PROPERTIES PREFIX "" )
#target_include_directories( NICANEPOS PUBLIC src/signal_io/ )

#add_library( ForcePIControl MODULE src/actuator_control/force_pi_control.c )
#set_target_properties( ForcePIControl PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/actuator_control/ )
#set_target_properties( ForcePIControl PROPERTIES PREFIX "" )
#target_include_directories( ForcePIControl PUBLIC src/actuator_control/ )

#add_library( SimpleJoint MODULE src/robot_control/simple_joint.c )
#set_target_properties( SimpleJoint PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/robot_control/ )
#set_target_properties( SimpleJoint PROPERTIES PREFIX "" )
#target_include_directories( SimpleJoint PUBLIC src/robot_control/ )

# add_library( CppPlugin MODULE src/robot_control/cpp_plugin.cpp )
# set_target_properties( CppPlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/robot_control/ )
# set_target_properties( CppPlugin PROPERTIES PREFIX "" )
# target_include_directories( CppPlugin PUBLIC src/robot_control/ )

# add_library( OpenSimModel MODULE src/robot_control/open_sim_model.cpp )
# set_target_properties( OpenSimModel PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/robot_control/ )
# set_target_properties( OpenSimModel PROPERTIES PREFIX "" )
# target_include_directories( OpenSimModel PUBLIC src/robot_control/ src/robot_control/SimTK/simbody )
# target_link_libraries( OpenSimModel libSimTKcommon_d.so libSimTKmath_d.so libSimTKsimbody_d.so libosimActuators_d.so libosimAnalyses_d.so libosimCommon_d.so libosimLepton_d.so libosimSimulation_d.so libosimTools_d.so )

# install( TARGETS RobRehabGui RobRehabServer RUNTIME DESTINATION . )
