# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/lorenzo/my_workspace/src/differential_gt

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lorenzo/my_workspace/src/differential_gt/build

# Include any dependencies generated for this target.
include CMakeFiles/ncgt.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ncgt.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ncgt.dir/flags.make

CMakeFiles/ncgt.dir/src/differential_gt/ncgt.cpp.o: CMakeFiles/ncgt.dir/flags.make
CMakeFiles/ncgt.dir/src/differential_gt/ncgt.cpp.o: ../src/differential_gt/ncgt.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lorenzo/my_workspace/src/differential_gt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ncgt.dir/src/differential_gt/ncgt.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ncgt.dir/src/differential_gt/ncgt.cpp.o -c /home/lorenzo/my_workspace/src/differential_gt/src/differential_gt/ncgt.cpp

CMakeFiles/ncgt.dir/src/differential_gt/ncgt.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ncgt.dir/src/differential_gt/ncgt.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lorenzo/my_workspace/src/differential_gt/src/differential_gt/ncgt.cpp > CMakeFiles/ncgt.dir/src/differential_gt/ncgt.cpp.i

CMakeFiles/ncgt.dir/src/differential_gt/ncgt.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ncgt.dir/src/differential_gt/ncgt.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lorenzo/my_workspace/src/differential_gt/src/differential_gt/ncgt.cpp -o CMakeFiles/ncgt.dir/src/differential_gt/ncgt.cpp.s

# Object files for target ncgt
ncgt_OBJECTS = \
"CMakeFiles/ncgt.dir/src/differential_gt/ncgt.cpp.o"

# External object files for target ncgt
ncgt_EXTERNAL_OBJECTS =

devel/lib/libncgt.so: CMakeFiles/ncgt.dir/src/differential_gt/ncgt.cpp.o
devel/lib/libncgt.so: CMakeFiles/ncgt.dir/build.make
devel/lib/libncgt.so: /opt/ros/noetic/lib/libroscpp.so
devel/lib/libncgt.so: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/libncgt.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
devel/lib/libncgt.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
devel/lib/libncgt.so: /opt/ros/noetic/lib/librosconsole.so
devel/lib/libncgt.so: /opt/ros/noetic/lib/librosconsole_log4cxx.so
devel/lib/libncgt.so: /opt/ros/noetic/lib/librosconsole_backend_interface.so
devel/lib/libncgt.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/libncgt.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
devel/lib/libncgt.so: /opt/ros/noetic/lib/libroscpp_serialization.so
devel/lib/libncgt.so: /opt/ros/noetic/lib/libxmlrpcpp.so
devel/lib/libncgt.so: /opt/ros/noetic/lib/librostime.so
devel/lib/libncgt.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
devel/lib/libncgt.so: /opt/ros/noetic/lib/libcpp_common.so
devel/lib/libncgt.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
devel/lib/libncgt.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
devel/lib/libncgt.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/libncgt.so: CMakeFiles/ncgt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lorenzo/my_workspace/src/differential_gt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library devel/lib/libncgt.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ncgt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ncgt.dir/build: devel/lib/libncgt.so

.PHONY : CMakeFiles/ncgt.dir/build

CMakeFiles/ncgt.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ncgt.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ncgt.dir/clean

CMakeFiles/ncgt.dir/depend:
	cd /home/lorenzo/my_workspace/src/differential_gt/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lorenzo/my_workspace/src/differential_gt /home/lorenzo/my_workspace/src/differential_gt /home/lorenzo/my_workspace/src/differential_gt/build /home/lorenzo/my_workspace/src/differential_gt/build /home/lorenzo/my_workspace/src/differential_gt/build/CMakeFiles/ncgt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ncgt.dir/depend

