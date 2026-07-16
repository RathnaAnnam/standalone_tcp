cmake_minimum_required(VERSION 3.10)

project(StandaloneEKF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(ekf
    #src/main.cpp
    src/filter_base.cpp
    src/filter_utilities.cpp
    src/ekf.cpp
    #src/ekf_manager.cpp
    src/tcp_filter.cpp
   
)

target_link_libraries(ekf
    Eigen3::Eigen
)


