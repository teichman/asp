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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME}
  src/pipeline.cpp
  src/params.cpp
  src/pod.cpp
  src/example_pods.cpp
  src/twiddler.cpp
  src/common.cpp
  )
#target_link_libraries(${PROJECT_NAME} yaml-cpp)
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} filesystem system)

rosbuild_add_gtest(test_pipeline src/test_pipeline.cpp)
rosbuild_add_gtest_build_flags(test_pipeline)
target_link_libraries(test_pipeline pipeline3)

#rosbuild_add_gtest(test_twiddler src/test_twiddler.cpp)
#rosbuild_add_compile_flags(test_twiddler -std=c++0x)
#target_link_libraries(test_twiddler pipeline3)

rosbuild_add_executable(yaml_experiment src/yaml_experiment.cpp)
target_link_libraries(yaml_experiment ${PROJECT_NAME})

rosbuild_add_executable(yaml_clone src/yaml_clone.cpp)
target_link_libraries(yaml_clone ${PROJECT_NAME})

rosbuild_add_executable(write_graphviz src/prog/write_graphviz.cpp)
target_link_libraries(write_graphviz ${PROJECT_NAME})
rosbuild_link_boost(write_graphviz filesystem program_options)
