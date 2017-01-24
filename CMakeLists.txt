cmake_minimum_required(VERSION 2.8)

project(Tabb_AhmadYousef_RWHEC_Jan2017_Project)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra")

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS} /usr/local)
link_directories( /usr/local/newmat )


add_executable(Tabb_AhmadYousef_RWHEC_Jan2017_Project
ComparisonMethods.hpp
ComparisonMethods.cpp
CostFunctions.hpp
CostFunctions.cpp
Calibration2.hpp
Calibration2.cpp
StringFunctions.hpp
DirectoryFunctions.hpp
DirectoryFunctions.cpp
Tabb_AhmadYousef_RWHEC_Jan2017_main.hpp
Tabb_AhmadYousef_RWHEC_Jan2017_main.cc
)

target_link_libraries(Tabb_AhmadYousef_RWHEC_Jan2017_Project ${CERES_LIBRARIES} newmat opencv_core opencv_highgui opencv_imgproc opencv_calib3d opencv_imgcodecs)

