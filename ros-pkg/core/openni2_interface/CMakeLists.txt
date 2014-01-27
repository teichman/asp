cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(openni2_interface
  src/lib/openni2_interface.cpp
  src/lib/openni_helpers.cpp
  )
rosbuild_add_compile_flags(openni2_interface)  # -DJARVIS_DEBUG
rosbuild_link_boost(openni2_interface filesystem program_options)

rosbuild_add_executable(openni_test src/prog/openni_test.cpp)

rosbuild_add_executable(openni2_interface_test src/prog/openni2_interface_test.cpp)
target_link_libraries(openni2_interface_test openni2_interface)

rosbuild_add_executable(jittery_image_experiment src/prog/jittery_image_experiment.cpp)
target_link_libraries(jittery_image_experiment openni2_interface)

rosbuild_add_executable(visualize src/prog/visualize.cpp)
target_link_libraries(visualize openni2_interface)