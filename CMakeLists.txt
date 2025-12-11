cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(VSLAM)

find_package(Pangolin REQUIRED)

include_directories("/usr/include/eigen3")
include_directories(${Pangolin_INCLUDE_DIRS})

# ignore python pangolin
set(PANGOLIN_LIBS_FILTERED "")
foreach(lib ${Pangolin_LIBRARIES})
    if(NOT lib MATCHES "python")
        list(APPEND PANGOLIN_LIBS_FILTERED ${lib})
    endif()
endforeach()

add_library(geom SHARED vizlib.cpp)
target_link_libraries(geom ${PANGOLIN_LIBS_FILTERED})

add_executable(vslam slam_main.cpp)
target_link_libraries(vslam geom)
