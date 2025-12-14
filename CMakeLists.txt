cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(VSLAM)

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package (Eigen3 REQUIRED NO_MODULE)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR})

# ignore python pangolin
set(PANGOLIN_LIBS_FILTERED "")
foreach(lib ${Pangolin_LIBRARIES})
    if(NOT lib MATCHES "python")
        list(APPEND PANGOLIN_LIBS_FILTERED ${lib})
    endif()
endforeach()

add_library(geom SHARED viz/vizlib.cpp)
target_link_libraries(geom ${PANGOLIN_LIBS_FILTERED})

add_executable(vslam slam_main.cpp)
target_link_libraries(vslam geom)