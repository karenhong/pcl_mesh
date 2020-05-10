cmake_minimum_required(VERSION 3.9)
set(CMAKE_CXX_STANDARD 11)
project(gore_pipeline)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (gore_pipeline gore_pipeline_start.cpp)
target_link_libraries (gore_pipeline ${PCL_LIBRARIES})