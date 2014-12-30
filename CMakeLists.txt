cmake_minimum_required(VERSION 2.6)
project(robrehabsystem)

add_executable(robrehabsystem main.cpp)

install(TARGETS robrehabsystem RUNTIME DESTINATION bin)
