cmake_minimum_required( VERSION 2.6 )
project( RobRehabGui )

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR} )

set( CMAKE_CXX_STANDARD_REQUIRED 11 )

find_package( PythonLibs REQUIRED )
find_package( Threads REQUIRED )

add_executable( RobRehabGui src/gui_main.cpp )
add_executable( RobRehabServer src/server_main.c )

include_directories( ./src ${PYTHON_INCLUDE_DIRS} )

target_link_libraries( RobRehabGui ${PYTHON_LIBRARIES} -lpthread )
target_link_libraries( RobRehabServer ${PYTHON_LIBRARIES} -lpthread )

install( TARGETS RobRehabGui RobRehabServer RUNTIME DESTINATION . )