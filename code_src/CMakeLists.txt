cmake_minimum_required(VERSION 2.8)

project(Tabb_AhmadYousef_RWHEC_Jun2018_Project)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra -std=c++11")

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS} /usr/local /usr/local/eigen3 /usr/local/include/opencv4)
link_directories( )


add_executable(Tabb_AhmadYousef_RWHEC_Jun2018_Project
CostFunctions.hpp
CostFunctions.cpp
Calibration2.hpp
Calibration2.cpp
StringFunctions.hpp
DirectoryFunctions.hpp
DirectoryFunctions.cpp
Tabb_AhmadYousef_RWHEC_Jun2018_main.hpp
Tabb_AhmadYousef_RWHEC_Jun2018_main.cpp
)

target_link_libraries(Tabb_AhmadYousef_RWHEC_Jun2018_Project ${CERES_LIBRARIES} opencv_core opencv_highgui opencv_imgproc opencv_calib3d opencv_imgcodecs)

