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
CMAKE_COMMAND = /snap/clion/138/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/138/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/daniel/TU_Berlin/PCV/Exercise04

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/daniel/TU_Berlin/PCV/Exercise04/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/main_manual.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main_manual.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main_manual.dir/flags.make

CMakeFiles/main_manual.dir/main_manual.cpp.o: CMakeFiles/main_manual.dir/flags.make
CMakeFiles/main_manual.dir/main_manual.cpp.o: ../main_manual.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/daniel/TU_Berlin/PCV/Exercise04/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main_manual.dir/main_manual.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main_manual.dir/main_manual.cpp.o -c /home/daniel/TU_Berlin/PCV/Exercise04/main_manual.cpp

CMakeFiles/main_manual.dir/main_manual.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main_manual.dir/main_manual.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/daniel/TU_Berlin/PCV/Exercise04/main_manual.cpp > CMakeFiles/main_manual.dir/main_manual.cpp.i

CMakeFiles/main_manual.dir/main_manual.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main_manual.dir/main_manual.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/daniel/TU_Berlin/PCV/Exercise04/main_manual.cpp -o CMakeFiles/main_manual.dir/main_manual.cpp.s

# Object files for target main_manual
main_manual_OBJECTS = \
"CMakeFiles/main_manual.dir/main_manual.cpp.o"

# External object files for target main_manual
main_manual_EXTERNAL_OBJECTS =

main_manual: CMakeFiles/main_manual.dir/main_manual.cpp.o
main_manual: CMakeFiles/main_manual.dir/build.make
main_manual: libcode.a
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
main_manual: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
main_manual: CMakeFiles/main_manual.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/daniel/TU_Berlin/PCV/Exercise04/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable main_manual"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main_manual.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main_manual.dir/build: main_manual

.PHONY : CMakeFiles/main_manual.dir/build

CMakeFiles/main_manual.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main_manual.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main_manual.dir/clean

CMakeFiles/main_manual.dir/depend:
	cd /home/daniel/TU_Berlin/PCV/Exercise04/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/daniel/TU_Berlin/PCV/Exercise04 /home/daniel/TU_Berlin/PCV/Exercise04 /home/daniel/TU_Berlin/PCV/Exercise04/cmake-build-debug /home/daniel/TU_Berlin/PCV/Exercise04/cmake-build-debug /home/daniel/TU_Berlin/PCV/Exercise04/cmake-build-debug/CMakeFiles/main_manual.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main_manual.dir/depend

