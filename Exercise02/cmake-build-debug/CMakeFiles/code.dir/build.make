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
CMAKE_COMMAND = /snap/clion/137/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/137/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/daniel/TU_Berlin/PCV/Exercise02

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/daniel/TU_Berlin/PCV/Exercise02/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/code.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/code.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/code.dir/flags.make

CMakeFiles/code.dir/Pcv2.cpp.o: CMakeFiles/code.dir/flags.make
CMakeFiles/code.dir/Pcv2.cpp.o: ../Pcv2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/daniel/TU_Berlin/PCV/Exercise02/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/code.dir/Pcv2.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/code.dir/Pcv2.cpp.o -c /home/daniel/TU_Berlin/PCV/Exercise02/Pcv2.cpp

CMakeFiles/code.dir/Pcv2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/code.dir/Pcv2.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/daniel/TU_Berlin/PCV/Exercise02/Pcv2.cpp > CMakeFiles/code.dir/Pcv2.cpp.i

CMakeFiles/code.dir/Pcv2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/code.dir/Pcv2.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/daniel/TU_Berlin/PCV/Exercise02/Pcv2.cpp -o CMakeFiles/code.dir/Pcv2.cpp.s

CMakeFiles/code.dir/Helper.cpp.o: CMakeFiles/code.dir/flags.make
CMakeFiles/code.dir/Helper.cpp.o: ../Helper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/daniel/TU_Berlin/PCV/Exercise02/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/code.dir/Helper.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/code.dir/Helper.cpp.o -c /home/daniel/TU_Berlin/PCV/Exercise02/Helper.cpp

CMakeFiles/code.dir/Helper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/code.dir/Helper.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/daniel/TU_Berlin/PCV/Exercise02/Helper.cpp > CMakeFiles/code.dir/Helper.cpp.i

CMakeFiles/code.dir/Helper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/code.dir/Helper.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/daniel/TU_Berlin/PCV/Exercise02/Helper.cpp -o CMakeFiles/code.dir/Helper.cpp.s

# Object files for target code
code_OBJECTS = \
"CMakeFiles/code.dir/Pcv2.cpp.o" \
"CMakeFiles/code.dir/Helper.cpp.o"

# External object files for target code
code_EXTERNAL_OBJECTS =

libcode.a: CMakeFiles/code.dir/Pcv2.cpp.o
libcode.a: CMakeFiles/code.dir/Helper.cpp.o
libcode.a: CMakeFiles/code.dir/build.make
libcode.a: CMakeFiles/code.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/daniel/TU_Berlin/PCV/Exercise02/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libcode.a"
	$(CMAKE_COMMAND) -P CMakeFiles/code.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/code.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/code.dir/build: libcode.a

.PHONY : CMakeFiles/code.dir/build

CMakeFiles/code.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/code.dir/cmake_clean.cmake
.PHONY : CMakeFiles/code.dir/clean

CMakeFiles/code.dir/depend:
	cd /home/daniel/TU_Berlin/PCV/Exercise02/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/daniel/TU_Berlin/PCV/Exercise02 /home/daniel/TU_Berlin/PCV/Exercise02 /home/daniel/TU_Berlin/PCV/Exercise02/cmake-build-debug /home/daniel/TU_Berlin/PCV/Exercise02/cmake-build-debug /home/daniel/TU_Berlin/PCV/Exercise02/cmake-build-debug/CMakeFiles/code.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/code.dir/depend

