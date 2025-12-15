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

add_library(viz SHARED viz/vizlib.cpp)
target_link_libraries(viz ${PANGOLIN_LIBS_FILTERED})

add_library(utils SHARED utils/utillib.cpp)
target_link_libraries(utils Eigen3::Eigen)

add_executable(vslam slam_main.cpp)
target_link_libraries(vslam viz utils)