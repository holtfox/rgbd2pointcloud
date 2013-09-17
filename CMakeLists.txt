cmake_minimum_required(VERSION 2.8)
project(rgdb2point)

find_package(CURL REQUIRED)
# find_package(OpenNI2 REQUIRED)

find_path(OPENNI2_INCLUDE_DIR OpenNI.h
			 HINTS $ENV{OPENNI2_INCLUDE})
find_library(OPENNI2_LIBRARIES NAMES OpenNI2
             HINTS $ENV{OPENNI2_REDIST})

set(SRCS
         rgbd2point.cpp
)

include_directories(${CURL_INCLUDE_DIR} ${OPENNI2_INCLUDE_DIR})

add_definitions(-Wall)

add_executable(rgbd2point ${SRCS})

target_link_libraries(rgbd2point ${CURL_LIBRARIES} ${OPENNI2_LIBRARIES})