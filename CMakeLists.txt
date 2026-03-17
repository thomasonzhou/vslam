cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(VSLAM)

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package (Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(glog REQUIRED)
find_package(g2o REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS}) 
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

add_library(utils SHARED utils/utillib.cpp utils/eval.cpp)
target_link_libraries(utils viz Eigen3::Eigen)

add_library(frontend frontend/orb.cpp)
target_link_libraries(frontend ${OpenCV_LIBS})

add_library(backend SHARED
    backend/epipolar.cpp
    backend/icp_nonlinear.cpp
    backend/pnp.cpp
    backend/pnp_g2o.cpp
    backend/solvers.cpp
)
target_link_libraries(backend
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        g2o::core
        ${OpenCV_LIBS}
)

add_executable(vslam slam_main.cpp)
target_link_libraries(vslam viz utils frontend backend)
