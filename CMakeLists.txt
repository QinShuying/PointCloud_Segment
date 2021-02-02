cmake_minimum_required(VERSION 3.17)
project(PointCloud_Segment)

set(CMAKE_CXX_STANDARD 14)



find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(seg_basic seg_basic.cpp)
target_link_libraries(seg_basic ${PCL_LIBRARIES})

add_executable(seg_region seg_region.cpp)
target_link_libraries(seg_region ${PCL_LIBRARIES})

add_executable(seg_Euclidean seg_Euclidean.cpp)
target_link_libraries(seg_Euclidean ${PCL_LIBRARIES})

add_executable(test test.cpp)
target_link_libraries(test ${PCL_LIBRARIES})