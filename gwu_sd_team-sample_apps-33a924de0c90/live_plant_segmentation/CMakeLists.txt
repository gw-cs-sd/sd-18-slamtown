cmake_minimum_required(VERSION 2.6 FATAL_ERROR)


# Project name
project(LIVE_PLANT_SEGMENTATION)


# Required packages
find_package(PCL 1.7 REQUIRED)
#find_package (OpenCV REQUIRED)


# Resources
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#include_directories(${OpenCV_INCLUDE_DIRS})
#link_directories(${OpenCV_LIBRARY_DIRS})
#add_definitions(${OpenCV_DEFINITIONS})


# Create executable
add_executable (live_plant_segmentation live_plant_segmentation.cpp)
#target_link_libraries (live_capture ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} )
target_link_libraries (live_plant_segmentation ${PCL_LIBRARIES} )


# Add app to the system bins
#install (TARGETS live_capture DESTINATION /usr/local/bin)
