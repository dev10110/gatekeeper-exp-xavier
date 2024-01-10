cmake_minimum_required(VERSION 3.5 FATAL_ERROR)


project(process_maps)


find_package(PCL 1.2 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# create executable
add_executable (process_maps process_maps.cpp)
target_link_libraries (process_maps ${PCL_LIBRARIES})
