cmake_minimum_required(VERSION 3.17)
project(PointCloud_Segment)

set(CMAKE_CXX_STANDARD 14)



find_package(PCL REQUIRED)

add_definitions(${PCL_DEFINITIONS})


include_directories(
        include
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        SHARED
        src/datapretreat.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARY_DIRS}
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
        )

add_executable(seg_basic code/seg_basic.cpp)
target_link_libraries(seg_basic ${PROJECT_NAME})

add_executable(seg_region code/seg_region.cpp)
target_link_libraries(seg_region ${PROJECT_NAME})

add_executable(seg_Euclidean code/seg_Euclidean.cpp)
target_link_libraries(seg_Euclidean ${PROJECT_NAME})

add_executable(test code/test.cpp)
target_link_libraries(test ${PROJECT_NAME})