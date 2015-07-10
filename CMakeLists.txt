cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(correspondence_grouping)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (correspondence_grouping correspondence_grouping.cpp)
target_link_libraries (correspondence_grouping ${PCL_LIBRARIES})

add_executable (bb_hull bb_hull.cpp)
target_link_libraries (bb_hull ${PCL_LIBRARIES})

add_executable (bbox bbox.cpp)
target_link_libraries (bbox ${PCL_LIBRARIES})

add_executable (mesh_sampling mesh_sampling.cpp)
target_link_libraries (mesh_sampling ${PCL_LIBRARIES})

add_executable (visualize_2_clouds_with_normals visualize_2_clouds_with_normals.cpp)
target_link_libraries (visualize_2_clouds_with_normals ${PCL_LIBRARIES})

add_executable (add_normals add_normals.cpp)
target_link_libraries (add_normals ${PCL_LIBRARIES})
