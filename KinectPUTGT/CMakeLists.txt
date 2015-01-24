cmake_minimum_required(VERSION 2.8)
project( Kinectv1GT )

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
endif()

find_package( OpenCV REQUIRED )


add_executable( Kinectv1GT main.cpp )
target_link_libraries( Kinectv1GT ${OpenCV_LIBS}) 
