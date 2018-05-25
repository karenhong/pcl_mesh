cmake_minimum_required(VERSION 3.9)
#project(pcd_test_project)

#set(CMAKE_CXX_STANDARD 11)

#add_executable(pcd_test_project pcd_write.cpp)
#cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)
project(pcd_test_project)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_test_project pcd_write.cpp)
target_link_libraries (pcd_test_project ${PCL_LIBRARIES})