cmake_minimum_required (VERSION 3.0)

project (SensorFusion_RGBDT)

# set flags
if (WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W2")
else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -W2 -Wextra")
endif()

# Add to module path, so we can find our cmake modules
list( APPEND CMAKE_MODULE_PATH C:\\opencv\\build\\FindOpenCV.cmake )

# find 3rd party libraries
set(CMAKE_HELPERS_DIR ${CMAKE_SOURCE_DIR}/cmake)
include(${CMAKE_HELPERS_DIR}/third-party.cmake)
third_party()

 set (OpenCV_DIR C:\opencv\build)# set output structure for binaries and libraries
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/install/lib ${CMAKE_SOURCE_DIR}/install/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/install/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/install/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/install/bin)

# Compile libraries
add_subdirectory(lib)

# Compile apps
add_subdirectory(apps)

# Compile examples
add_subdirectory(examples)