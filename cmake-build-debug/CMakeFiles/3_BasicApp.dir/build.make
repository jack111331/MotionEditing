# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /var/lib/snapd/snap/clion/138/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /var/lib/snapd/snap/clion/138/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/edge/Code/ThreeD_Game/Project_1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/edge/Code/ThreeD_Game/Project_1/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/3_BasicApp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/3_BasicApp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/3_BasicApp.dir/flags.make

CMakeFiles/3_BasicApp.dir/BasicApp.cpp.o: CMakeFiles/3_BasicApp.dir/flags.make
CMakeFiles/3_BasicApp.dir/BasicApp.cpp.o: ../BasicApp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/edge/Code/ThreeD_Game/Project_1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/3_BasicApp.dir/BasicApp.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/3_BasicApp.dir/BasicApp.cpp.o -c /home/edge/Code/ThreeD_Game/Project_1/BasicApp.cpp

CMakeFiles/3_BasicApp.dir/BasicApp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/3_BasicApp.dir/BasicApp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/edge/Code/ThreeD_Game/Project_1/BasicApp.cpp > CMakeFiles/3_BasicApp.dir/BasicApp.cpp.i

CMakeFiles/3_BasicApp.dir/BasicApp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/3_BasicApp.dir/BasicApp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/edge/Code/ThreeD_Game/Project_1/BasicApp.cpp -o CMakeFiles/3_BasicApp.dir/BasicApp.cpp.s

# Object files for target 3_BasicApp
3_BasicApp_OBJECTS = \
"CMakeFiles/3_BasicApp.dir/BasicApp.cpp.o"

# External object files for target 3_BasicApp
3_BasicApp_EXTERNAL_OBJECTS =

3_BasicApp: CMakeFiles/3_BasicApp.dir/BasicApp.cpp.o
3_BasicApp: CMakeFiles/3_BasicApp.dir/build.make
3_BasicApp: /usr/local/lib/libOgreBites.so.1.12.11
3_BasicApp: /usr/local/lib/libOgreMeshLodGenerator.so.1.12.11
3_BasicApp: /usr/local/lib/libOgreOverlay.so.1.12.11
3_BasicApp: /usr/local/lib/libOgreProperty.so.1.12.11
3_BasicApp: /usr/local/lib/libOgreRTShaderSystem.so.1.12.11
3_BasicApp: /usr/local/lib/libOgreTerrain.so.1.12.11
3_BasicApp: /usr/local/lib/libOgreVolume.so.1.12.11
3_BasicApp: /usr/local/lib/libOgrePaging.so.1.12.11
3_BasicApp: /usr/local/lib/libOgreMain.so.1.12.11
3_BasicApp: CMakeFiles/3_BasicApp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/edge/Code/ThreeD_Game/Project_1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable 3_BasicApp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/3_BasicApp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/3_BasicApp.dir/build: 3_BasicApp

.PHONY : CMakeFiles/3_BasicApp.dir/build

CMakeFiles/3_BasicApp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/3_BasicApp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/3_BasicApp.dir/clean

CMakeFiles/3_BasicApp.dir/depend:
	cd /home/edge/Code/ThreeD_Game/Project_1/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/edge/Code/ThreeD_Game/Project_1 /home/edge/Code/ThreeD_Game/Project_1 /home/edge/Code/ThreeD_Game/Project_1/cmake-build-debug /home/edge/Code/ThreeD_Game/Project_1/cmake-build-debug /home/edge/Code/ThreeD_Game/Project_1/cmake-build-debug/CMakeFiles/3_BasicApp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/3_BasicApp.dir/depend

