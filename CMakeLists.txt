cmake_minimum_required(VERSION 3.5)
project(LIPS_CV4NI2)
add_executable(LIPS_CV4NI2 src/main.cpp)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS "-O0 -g -Dlinux -std=gnu++11")

SET(OpenCV_DIR /home/inaho-00/tmp/opencv4/build/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
#pkg_check_modules(OpenCV opencv4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(LIPS_CV4NI2 ${OpenCV_LIBRARIES})

include_directories(
    /usr/local/include/OpenNI2
)

target_link_libraries(
    LIPS_CV4NI2 
    -L/home/inaho-00/tmp/LIPS-Linux-x64-OpenNI2.2/Tools
    -lOpenNI2
)
