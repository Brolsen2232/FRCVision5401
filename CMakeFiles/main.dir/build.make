# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

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
CMAKE_SOURCE_DIR = /home/mint/Desktop/RoboticsVision

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mint/Desktop/RoboticsVision

# Include any dependencies generated for this target.
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/main.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/main.cpp.o: main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mint/Desktop/RoboticsVision/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/main.cpp.o -c /home/mint/Desktop/RoboticsVision/main.cpp

CMakeFiles/main.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mint/Desktop/RoboticsVision/main.cpp > CMakeFiles/main.dir/main.cpp.i

CMakeFiles/main.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mint/Desktop/RoboticsVision/main.cpp -o CMakeFiles/main.dir/main.cpp.s

# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/main.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

main: CMakeFiles/main.dir/main.cpp.o
main: CMakeFiles/main.dir/build.make
main: /usr/local/lib/librealsense2.so
main: /usr/local/lib/libopencv_dnn.so.4.2.0
main: /usr/local/lib/libopencv_gapi.so.4.2.0
main: /usr/local/lib/libopencv_highgui.so.4.2.0
main: /usr/local/lib/libopencv_ml.so.4.2.0
main: /usr/local/lib/libopencv_objdetect.so.4.2.0
main: /usr/local/lib/libopencv_photo.so.4.2.0
main: /usr/local/lib/libopencv_stitching.so.4.2.0
main: /usr/local/lib/libopencv_video.so.4.2.0
main: /usr/local/lib/libopencv_videoio.so.4.2.0
main: /usr/local/lib/libdlib.a
main: /usr/lib/x86_64-linux-gnu/libboost_system.so
main: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
main: /usr/lib/x86_64-linux-gnu/libboost_thread.so
main: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
main: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
main: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
main: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
main: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
main: /usr/lib/x86_64-linux-gnu/libboost_regex.so
main: /usr/local/lib/libpcl_common.so
main: /usr/local/lib/libpcl_octree.so
main: /usr/lib/libOpenNI.so
main: /usr/lib/libOpenNI2.so
main: /usr/local/lib/libpcl_io.so
main: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
main: /usr/local/lib/libpcl_kdtree.so
main: /usr/local/lib/libpcl_search.so
main: /usr/local/lib/libpcl_visualization.so
main: /usr/local/lib/libpcl_sample_consensus.so
main: /usr/local/lib/libpcl_filters.so
main: /usr/local/lib/libpcl_features.so
main: /usr/local/lib/libpcl_ml.so
main: /usr/local/lib/libpcl_segmentation.so
main: /usr/lib/x86_64-linux-gnu/libboost_system.so
main: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
main: /usr/lib/x86_64-linux-gnu/libboost_thread.so
main: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
main: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
main: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
main: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
main: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
main: /usr/lib/x86_64-linux-gnu/libboost_regex.so
main: /usr/lib/libOpenNI.so
main: /usr/lib/libOpenNI2.so
main: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
main: /usr/local/lib/libvtkDomainsChemistryOpenGL2-7.1.so.1
main: /usr/local/lib/libvtkFiltersFlowPaths-7.1.so.1
main: /usr/local/lib/libvtkFiltersGeneric-7.1.so.1
main: /usr/local/lib/libvtkFiltersHyperTree-7.1.so.1
main: /usr/local/lib/libvtkFiltersParallelImaging-7.1.so.1
main: /usr/local/lib/libvtkFiltersPoints-7.1.so.1
main: /usr/local/lib/libvtkFiltersProgrammable-7.1.so.1
main: /usr/local/lib/libvtkFiltersSMP-7.1.so.1
main: /usr/local/lib/libvtkFiltersSelection-7.1.so.1
main: /usr/local/lib/libvtkFiltersTexture-7.1.so.1
main: /usr/local/lib/libvtkFiltersVerdict-7.1.so.1
main: /usr/local/lib/libvtkverdict-7.1.so.1
main: /usr/local/lib/libvtkGeovisCore-7.1.so.1
main: /usr/local/lib/libvtkproj4-7.1.so.1
main: /usr/local/lib/libvtkIOAMR-7.1.so.1
main: /usr/local/lib/libvtkIOEnSight-7.1.so.1
main: /usr/local/lib/libvtkIOExodus-7.1.so.1
main: /usr/local/lib/libvtkIOExport-7.1.so.1
main: /usr/local/lib/libvtkRenderingGL2PSOpenGL2-7.1.so.1
main: /usr/local/lib/libvtkgl2ps-7.1.so.1
main: /usr/local/lib/libvtkIOImport-7.1.so.1
main: /usr/local/lib/libvtkIOInfovis-7.1.so.1
main: /usr/local/lib/libvtklibxml2-7.1.so.1
main: /usr/local/lib/libvtkIOLSDyna-7.1.so.1
main: /usr/local/lib/libvtkIOMINC-7.1.so.1
main: /usr/local/lib/libvtkIOMovie-7.1.so.1
main: /usr/local/lib/libvtkoggtheora-7.1.so.1
main: /usr/local/lib/libvtkIOPLY-7.1.so.1
main: /usr/local/lib/libvtkIOParallel-7.1.so.1
main: /usr/local/lib/libvtkjsoncpp-7.1.so.1
main: /usr/local/lib/libvtkIOParallelXML-7.1.so.1
main: /usr/local/lib/libvtkIOSQL-7.1.so.1
main: /usr/local/lib/libvtksqlite-7.1.so.1
main: /usr/local/lib/libvtkIOTecplotTable-7.1.so.1
main: /usr/local/lib/libvtkIOVideo-7.1.so.1
main: /usr/local/lib/libvtkImagingMorphological-7.1.so.1
main: /usr/local/lib/libvtkImagingStatistics-7.1.so.1
main: /usr/local/lib/libvtkImagingStencil-7.1.so.1
main: /usr/local/lib/libvtkInteractionImage-7.1.so.1
main: /usr/local/lib/libvtkRenderingContextOpenGL2-7.1.so.1
main: /usr/local/lib/libvtkRenderingImage-7.1.so.1
main: /usr/local/lib/libvtkRenderingLOD-7.1.so.1
main: /usr/local/lib/libvtkRenderingVolumeOpenGL2-7.1.so.1
main: /usr/local/lib/libvtkViewsContext2D-7.1.so.1
main: /usr/local/lib/libvtkViewsInfovis-7.1.so.1
main: /usr/local/lib/libpcl_common.so
main: /usr/local/lib/libpcl_octree.so
main: /usr/local/lib/libpcl_io.so
main: /usr/local/lib/libpcl_kdtree.so
main: /usr/local/lib/libpcl_search.so
main: /usr/local/lib/libpcl_visualization.so
main: /usr/local/lib/libpcl_sample_consensus.so
main: /usr/local/lib/libpcl_filters.so
main: /usr/local/lib/libpcl_features.so
main: /usr/local/lib/libpcl_ml.so
main: /usr/local/lib/libpcl_segmentation.so
main: /usr/local/lib/libopencv_imgcodecs.so.4.2.0
main: /usr/local/lib/libopencv_calib3d.so.4.2.0
main: /usr/local/lib/libopencv_features2d.so.4.2.0
main: /usr/local/lib/libopencv_flann.so.4.2.0
main: /usr/local/lib/libopencv_imgproc.so.4.2.0
main: /usr/local/lib/libopencv_core.so.4.2.0
main: /usr/lib/x86_64-linux-gnu/libgif.so
main: /usr/lib/x86_64-linux-gnu/libpng.so
main: /usr/lib/x86_64-linux-gnu/libz.so
main: /usr/lib/x86_64-linux-gnu/libjpeg.so
main: /usr/lib/x86_64-linux-gnu/libsqlite3.so
main: /usr/local/lib/libvtkDomainsChemistry-7.1.so.1
main: /usr/local/lib/libvtkFiltersAMR-7.1.so.1
main: /usr/local/lib/libvtkFiltersParallel-7.1.so.1
main: /usr/local/lib/libvtkexoIIc-7.1.so.1
main: /usr/local/lib/libvtkIOGeometry-7.1.so.1
main: /usr/local/lib/libvtkIONetCDF-7.1.so.1
main: /usr/local/lib/libvtkNetCDF_cxx-7.1.so.1
main: /usr/local/lib/libvtkNetCDF-7.1.so.1
main: /usr/local/lib/libvtkhdf5_hl-7.1.so.1
main: /usr/local/lib/libvtkhdf5-7.1.so.1
main: /usr/local/lib/libvtkParallelCore-7.1.so.1
main: /usr/local/lib/libvtkIOLegacy-7.1.so.1
main: /usr/local/lib/libvtkRenderingOpenGL2-7.1.so.1
main: /usr/lib/x86_64-linux-gnu/libSM.so
main: /usr/lib/x86_64-linux-gnu/libICE.so
main: /usr/lib/x86_64-linux-gnu/libX11.so
main: /usr/lib/x86_64-linux-gnu/libXext.so
main: /usr/lib/x86_64-linux-gnu/libXt.so
main: /usr/local/lib/libvtkglew-7.1.so.1
main: /usr/local/lib/libvtkImagingMath-7.1.so.1
main: /usr/local/lib/libvtkChartsCore-7.1.so.1
main: /usr/local/lib/libvtkRenderingContext2D-7.1.so.1
main: /usr/local/lib/libvtkFiltersImaging-7.1.so.1
main: /usr/local/lib/libvtkInfovisLayout-7.1.so.1
main: /usr/local/lib/libvtkInfovisCore-7.1.so.1
main: /usr/local/lib/libvtkViewsCore-7.1.so.1
main: /usr/local/lib/libvtkInteractionWidgets-7.1.so.1
main: /usr/local/lib/libvtkFiltersHybrid-7.1.so.1
main: /usr/local/lib/libvtkImagingGeneral-7.1.so.1
main: /usr/local/lib/libvtkImagingSources-7.1.so.1
main: /usr/local/lib/libvtkFiltersModeling-7.1.so.1
main: /usr/local/lib/libvtkImagingHybrid-7.1.so.1
main: /usr/local/lib/libvtkIOImage-7.1.so.1
main: /usr/local/lib/libvtkDICOMParser-7.1.so.1
main: /usr/local/lib/libvtkmetaio-7.1.so.1
main: /usr/local/lib/libvtkpng-7.1.so.1
main: /usr/local/lib/libvtktiff-7.1.so.1
main: /usr/local/lib/libvtkjpeg-7.1.so.1
main: /usr/lib/x86_64-linux-gnu/libm.so
main: /usr/local/lib/libvtkInteractionStyle-7.1.so.1
main: /usr/local/lib/libvtkFiltersExtraction-7.1.so.1
main: /usr/local/lib/libvtkFiltersStatistics-7.1.so.1
main: /usr/local/lib/libvtkImagingFourier-7.1.so.1
main: /usr/local/lib/libvtkalglib-7.1.so.1
main: /usr/local/lib/libvtkRenderingAnnotation-7.1.so.1
main: /usr/local/lib/libvtkImagingColor-7.1.so.1
main: /usr/local/lib/libvtkRenderingVolume-7.1.so.1
main: /usr/local/lib/libvtkImagingCore-7.1.so.1
main: /usr/local/lib/libvtkIOXML-7.1.so.1
main: /usr/local/lib/libvtkIOXMLParser-7.1.so.1
main: /usr/local/lib/libvtkIOCore-7.1.so.1
main: /usr/local/lib/libvtkexpat-7.1.so.1
main: /usr/local/lib/libvtkRenderingLabel-7.1.so.1
main: /usr/local/lib/libvtkRenderingFreeType-7.1.so.1
main: /usr/local/lib/libvtkRenderingCore-7.1.so.1
main: /usr/local/lib/libvtkCommonColor-7.1.so.1
main: /usr/local/lib/libvtkFiltersGeometry-7.1.so.1
main: /usr/local/lib/libvtkFiltersSources-7.1.so.1
main: /usr/local/lib/libvtkFiltersGeneral-7.1.so.1
main: /usr/local/lib/libvtkCommonComputationalGeometry-7.1.so.1
main: /usr/local/lib/libvtkFiltersCore-7.1.so.1
main: /usr/local/lib/libvtkCommonExecutionModel-7.1.so.1
main: /usr/local/lib/libvtkCommonDataModel-7.1.so.1
main: /usr/local/lib/libvtkCommonTransforms-7.1.so.1
main: /usr/local/lib/libvtkCommonMisc-7.1.so.1
main: /usr/local/lib/libvtkCommonMath-7.1.so.1
main: /usr/local/lib/libvtkCommonSystem-7.1.so.1
main: /usr/local/lib/libvtkCommonCore-7.1.so.1
main: /usr/local/lib/libvtksys-7.1.so.1
main: /usr/local/lib/libvtkfreetype-7.1.so.1
main: /usr/local/lib/libvtkzlib-7.1.so.1
main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mint/Desktop/RoboticsVision/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: main

.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd /home/mint/Desktop/RoboticsVision && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mint/Desktop/RoboticsVision /home/mint/Desktop/RoboticsVision /home/mint/Desktop/RoboticsVision /home/mint/Desktop/RoboticsVision /home/mint/Desktop/RoboticsVision/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend

