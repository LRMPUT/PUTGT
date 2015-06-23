cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Recomp)

set(CMAKE_BUILD_TYPE Release) 
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") # -Wall 

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3)

include_directories(${EIGEN3_INCLUDE_DIR})

message(${EIGEN3_INCLUDE_DIR})

add_executable (RecompPUTGT main.cpp)
target_link_libraries (RecompPUTGT ${OpenCV_LIBS})
