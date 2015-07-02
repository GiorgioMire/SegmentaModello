
cmake_minimum_required(VERSION 2.8)

project(ComputeFeature)


#Set variables 
set(CD ${CMAKE_CURRENT_SOURCE_DIR})
set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/)
set(LIBDIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(LOCAL_INCLUDE  /usr/local/include/)
message("Variabili dei percorsi impostate")


find_package(PCL 1.8 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS} ${CD}/include ${LOCAL_INCLUDE}/eigen3/)
link_directories(${PCL_LIBRARY_DIRS} ${LIBDIR})
add_definitions(${PCL_DEFINITIONS})
message("Libreria PCL trovata")


SET(CMAKE_CXX_FLAGS "-std=c++0x")

add_executable (ComputeFeature_exe
 ${SRC}/main.cpp
${SRC}/CalcolaFPFHsuPatch.cpp
 )



target_link_libraries (ComputeFeature_exe ${PCL_LIBRARIES}  VisualizzatorePatch)
