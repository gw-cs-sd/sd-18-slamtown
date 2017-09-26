cmake_minimum_required(VERSION 2.6 FATAL_ERROR) 
project(LIVE_SEG)

find_package(PCL 1.7 REQUIRED)

# Find QT4
find_package(Qt4 REQUIRED) 
if (QT4_FOUND)
  include(${QT_USE_FILE})
endif (QT4_FOUND)
include_directories(${PCL_INCLUDE_DIRS}) 
include_directories(${PROJECT_SOURCE_DIR})
 message("VTK found (include: ${VTK_INCLUDE_DIRS}, lib: ${VTK_LIBRARY_DIRS})")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" )

if(PCL_SHARED_LIBS)
message ("Warning: PCL_SHARED_LIBS is ON!")
else()
message ("Warning: PCL_SHARED_LIBS is OFF!")
endif()

if(VTK_BUILD_SHARED_LIBS)
message ("Warning: VTK_BUILD_SHARED_LIBS is ON!")
else()
message ("Warning: VTK_BUILD_SHARED_LIBS is OFF!")
endif()

# Find VTK
find_package(VTK)
if(VTK_FOUND)

    set(VTK_FOUND TRUE)
    find_package(QVTK)
    message(STATUS "VTK found (include: ${VTK_INCLUDE_DIRS}, lib: ${VTK_LIBRARY_DIRS})")
    link_directories(${VTK_LIBRARY_DIRS})
    set(HAVE_VTK ON)
else()
message ("Warning: VTK_FOUNDis OFF!")
endif()
# Find VTK
if(NOT VTK_FOUND)
    set(DEFAULT FALSE)
    set(REASON "VTK was not found.")
else(NOT VTK_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
    include (${VTK_USE_FILE})
endif(NOT VTK_FOUND)

#find_package(Boost REQUIRED)

#Here we add our source files 
SET (myprj_src 
        src/prototyper_window.cpp 
        src/main.cpp
     ) 
#Define QT resource file.this file may define icons,images. 
#SET(myprj_rccs 
#    resources/myresources.qrc 
#   ) 

#QT headers needed to be processed with QT MOH compiler 
SET(myprj_hdrs 
        include/prototyper_window.h OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
        #include/prototyper_qt.h
    )
message("headers: ${myprj_hdrs}")
#QT UI files needed to be processed 
SET(myprj_uis 
        src/prototyper_window.ui)

#Generate CPP files from moh headers 
QT4_WRAP_CPP(myprj_headers_moc ${myprj_hdrs}) 

#Generate resource header files 
#QT4_ADD_RESOURCES(myprj_rccs_srcs ${myprj_rccs})

#Generate headers using QT UI files 
QT4_WRAP_UI(myprj_ui_hdrs ${myprj_uis}) 

link_directories(${PCL_LIBRARY_DIRS}) 
add_definitions(${PCL_DEFINITIONS}) 

INCLUDE(${QT_USE_FILE})

#add_executable(editor main.cpp editorwindow.cpp) 
#target_link_libraries(editor ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${QT_LIBRARIES}) 
ADD_EXECUTABLE(live_segmentation ${myprj_src}
	${myprj_headers_moc} 
        ${myprj_ui_hdrs} )
#	${myprj_rccs_srcs} )
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
TARGET_LINK_LIBRARIES(live_segmentation QVTK ${QT_LIBRARIES} ${PCL_LIBRARIES})
