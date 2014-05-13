cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(dst
  src/lib/realtime_interface.cpp
  src/lib/potentials_cache.cpp
  src/lib/kinect_sequence.cpp
  src/lib/seed_npg.cpp
  src/lib/color_histogram_npg.cpp
  src/lib/seed_image_distance_npg.cpp
  src/lib/canny_epg.cpp
  src/lib/color_delta_epg.cpp
  src/lib/depth_epg.cpp
  src/lib/depth_epg_velodyne.cpp
  src/lib/node_potential_aggregator.cpp
  src/lib/edge_potential_aggregator.cpp
  src/lib/node_potential_generator.cpp
  src/lib/edge_potential_generator.cpp
  src/lib/depth_projector.cpp
  src/lib/optical_flow_node.cpp
  src/lib/label_flow_npg.cpp
  src/lib/kdtree_node.cpp
  src/lib/scene_alignment_node.cpp
  src/lib/scene_alignment_npg.cpp
  src/lib/surface_normal_node.cpp
  src/lib/surface_normal_epg.cpp
  src/lib/flow_clustering_epg.cpp
  src/lib/edge_potential_product_epg.cpp
  src/lib/bilateral_npg.cpp
  src/lib/helper_functions.cpp
  src/lib/organized_surface_normal_node.cpp
  src/lib/segmentation_visualizer.cpp
  src/lib/patch_classifier.cpp
  src/lib/depth_npg.cpp
  src/lib/icp_npg.cpp
  src/lib/prior_npg.cpp
  src/lib/foreground_kdtree_node.cpp
  src/lib/distance_npg.cpp
  src/lib/template_matcher_npg.cpp
  src/lib/image_processing_nodes.cpp
  src/lib/cie-xyz-image-node.cpp
#  src/lib/sobel-image-node.cpp
  src/lib/boundary_mask_node.cpp
  src/lib/sequence_segmentation_view_controller.cpp
  src/lib/segmentation_pipeline.cpp
  src/lib/struct_svm.cpp
  src/lib/evaluator.cpp
  )
rosbuild_add_compile_flags(dst -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)
rosbuild_link_boost(dst system)

rosbuild_add_executable(image_cut src/program/image_cut.cpp)
rosbuild_add_executable(kinect_cut src/program/kinect_cut.cpp)
target_link_libraries(kinect_cut dst)


rosbuild_add_executable(train src/program/train.cpp)
target_link_libraries(train dst)
rosbuild_add_compile_flags(train -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)


rosbuild_add_executable(generate_houghtrack_input src/program/generate_houghtrack_input.cpp)
target_link_libraries(generate_houghtrack_input dst)
rosbuild_add_compile_flags(generate_houghtrack_input -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(evaluate_houghtrack src/program/evaluate_houghtrack.cpp)
target_link_libraries(evaluate_houghtrack dst)
rosbuild_add_compile_flags(evaluate_houghtrack -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(sequence_stats src/program/sequence_stats.cpp)
target_link_libraries(sequence_stats dst)
rosbuild_add_compile_flags(sequence_stats -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(variance src/program/variance.cpp)
target_link_libraries(variance dst)
rosbuild_add_compile_flags(variance -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(segmentation_visualizer src/program/segmentation_visualizer.cpp)
target_link_libraries(segmentation_visualizer dst)
rosbuild_add_compile_flags(segmentation_visualizer -Wno-deprecated -Wno-unused-but-set-variable)

rosbuild_add_executable(kinect_cut_interactive src/program/kinect_cut_interactive.cpp)
target_link_libraries(kinect_cut_interactive dst)
rosbuild_add_compile_flags(kinect_cut_interactive -Wno-deprecated -Wno-unused-but-set-variable)

rosbuild_add_executable(openni_viewer_simple src/program/openni_viewer_simple.cpp)
target_link_libraries(openni_viewer_simple dst)
rosbuild_add_compile_flags(openni_viewer_simple -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(realtime_interface src/program/realtime_interface.cpp)
target_link_libraries(realtime_interface dst)
rosbuild_link_boost(realtime_interface filesystem program_options)
rosbuild_add_compile_flags(realtime_interface -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(translate_sequence src/program/translate_sequence.cpp)
target_link_libraries(translate_sequence dst)
rosbuild_add_compile_flags(translate_sequence -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(evaluate src/program/evaluate.cpp)
target_link_libraries(evaluate dst)
rosbuild_add_compile_flags(evaluate -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(generate_random_weights src/program/generate_random_weights.cpp)
target_link_libraries(generate_random_weights)
rosbuild_link_boost(generate_random_weights filesystem program_options)
rosbuild_add_compile_flags(generate_random_weights -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_gtest(test_maxflow src/test/test_maxflow.cpp)
target_link_libraries(test_maxflow dst)
rosbuild_add_gtest_build_flags(test_maxflow)

