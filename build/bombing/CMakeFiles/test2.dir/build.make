# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/amov/catkin_ws/src/bombing

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/amov/catkin_ws/build/bombing

# Include any dependencies generated for this target.
include CMakeFiles/test2.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/test2.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/test2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test2.dir/flags.make

CMakeFiles/test2.dir/src/test2.cpp.o: CMakeFiles/test2.dir/flags.make
CMakeFiles/test2.dir/src/test2.cpp.o: /home/amov/catkin_ws/src/bombing/src/test2.cpp
CMakeFiles/test2.dir/src/test2.cpp.o: CMakeFiles/test2.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amov/catkin_ws/build/bombing/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test2.dir/src/test2.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test2.dir/src/test2.cpp.o -MF CMakeFiles/test2.dir/src/test2.cpp.o.d -o CMakeFiles/test2.dir/src/test2.cpp.o -c /home/amov/catkin_ws/src/bombing/src/test2.cpp

CMakeFiles/test2.dir/src/test2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test2.dir/src/test2.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amov/catkin_ws/src/bombing/src/test2.cpp > CMakeFiles/test2.dir/src/test2.cpp.i

CMakeFiles/test2.dir/src/test2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test2.dir/src/test2.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amov/catkin_ws/src/bombing/src/test2.cpp -o CMakeFiles/test2.dir/src/test2.cpp.s

# Object files for target test2
test2_OBJECTS = \
"CMakeFiles/test2.dir/src/test2.cpp.o"

# External object files for target test2
test2_EXTERNAL_OBJECTS =

/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: CMakeFiles/test2.dir/src/test2.cpp.o
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: CMakeFiles/test2.dir/build.make
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /opt/ros/melodic/lib/libroscpp.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /opt/ros/melodic/lib/librosconsole.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /opt/ros/melodic/lib/libserial.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /opt/ros/melodic/lib/librostime.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /opt/ros/melodic/lib/libcpp_common.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2: CMakeFiles/test2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/amov/catkin_ws/build/bombing/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test2.dir/build: /home/amov/catkin_ws/devel/.private/bombing/lib/bombing/test2
.PHONY : CMakeFiles/test2.dir/build

CMakeFiles/test2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test2.dir/clean

CMakeFiles/test2.dir/depend:
	cd /home/amov/catkin_ws/build/bombing && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/amov/catkin_ws/src/bombing /home/amov/catkin_ws/src/bombing /home/amov/catkin_ws/build/bombing /home/amov/catkin_ws/build/bombing /home/amov/catkin_ws/build/bombing/CMakeFiles/test2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test2.dir/depend

