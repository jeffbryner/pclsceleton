cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pclsceleton)



#include_directories ("/usr/local/include/oscpack")
include_directories("./oscpack","/usr/lib64")

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pclsceleton pclsceleton.cpp)
target_link_libraries (pclsceleton ${PCL_LIBRARIES} oscpack)

