# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rerob/Projetos/RobRehabSystem

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rerob/Projetos/RobRehabSystem/build

# Include any dependencies generated for this target.
include CMakeFiles/ForcePIControl.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ForcePIControl.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ForcePIControl.dir/flags.make

CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.o: CMakeFiles/ForcePIControl.dir/flags.make
CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.o: ../src/actuator_control/force_pi_control.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rerob/Projetos/RobRehabSystem/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.o   -c /home/rerob/Projetos/RobRehabSystem/src/actuator_control/force_pi_control.c

CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/rerob/Projetos/RobRehabSystem/src/actuator_control/force_pi_control.c > CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.i

CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/rerob/Projetos/RobRehabSystem/src/actuator_control/force_pi_control.c -o CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.s

CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.o.requires:

.PHONY : CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.o.requires

CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.o.provides: CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.o.requires
	$(MAKE) -f CMakeFiles/ForcePIControl.dir/build.make CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.o.provides.build
.PHONY : CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.o.provides

CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.o.provides.build: CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.o


# Object files for target ForcePIControl
ForcePIControl_OBJECTS = \
"CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.o"

# External object files for target ForcePIControl
ForcePIControl_EXTERNAL_OBJECTS =

../plugins/actuator_control/ForcePIControl.so: CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.o
../plugins/actuator_control/ForcePIControl.so: CMakeFiles/ForcePIControl.dir/build.make
../plugins/actuator_control/ForcePIControl.so: CMakeFiles/ForcePIControl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rerob/Projetos/RobRehabSystem/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C shared module ../plugins/actuator_control/ForcePIControl.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ForcePIControl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ForcePIControl.dir/build: ../plugins/actuator_control/ForcePIControl.so

.PHONY : CMakeFiles/ForcePIControl.dir/build

CMakeFiles/ForcePIControl.dir/requires: CMakeFiles/ForcePIControl.dir/src/actuator_control/force_pi_control.c.o.requires

.PHONY : CMakeFiles/ForcePIControl.dir/requires

CMakeFiles/ForcePIControl.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ForcePIControl.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ForcePIControl.dir/clean

CMakeFiles/ForcePIControl.dir/depend:
	cd /home/rerob/Projetos/RobRehabSystem/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rerob/Projetos/RobRehabSystem /home/rerob/Projetos/RobRehabSystem /home/rerob/Projetos/RobRehabSystem/build /home/rerob/Projetos/RobRehabSystem/build /home/rerob/Projetos/RobRehabSystem/build/CMakeFiles/ForcePIControl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ForcePIControl.dir/depend

