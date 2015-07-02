
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_CXX_FLAGS "-std=c++0x")
project(region_growing)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ./include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (region_growing ./src/region_growing.cpp ./src/mylibforpcl.cpp)
target_link_libraries (region_growing ${PCL_LIBRARIES} )
