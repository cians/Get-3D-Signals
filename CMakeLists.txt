cmake_minimum_required(VERSION 2.8)
project(calibLocation)
set( CMAKE_CXX_FLAGS "-std=c++11 -O3")
set( CMAKE_CXX_COMPILER "g++")
set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

#add findEigen3.cmake 
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )
include_directories( OPENCV_INCLUDE_DIRS)
add_executable( calibLocation Calib.cc)
target_link_libraries( calibLocation 
    ${OpenCV_LIBS})