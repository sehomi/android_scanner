# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.
project("scanner")

cmake_minimum_required(VERSION 3.10.2)

set(OpenCV_STATIC ON)
set(OpenCV_DIR ${CMAKE_CURRENT_LIST_DIR}/../OpenCV-android-sdk/sdk/native/jni)
find_package(OpenCV REQUIRED)

set(BOOST_ROOT ${CMAKE_CURRENT_LIST_DIR}/../boost-1_72_0)
set(Boost_INCLUDE_DIR ${BOOST_ROOT}/include)
set(Boost_LIBRARY_DIR ${BOOST_ROOT}/arm64-v8a/lib)
set(Boost_Version 1_72_0)

find_package( Boost COMPONENTS geometry )

if( Boost_FOUND )
    
else()
        message(" \n\n\n######## boost not found ########\n\n\n\n")
endif()

include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../eigen-master)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../nasagrid)
include_directories(${Boost_INCLUDE_DIR})
#include_directories(D:/android_scanner/GeographicLib/include)

add_library( # Sets the name of the library.
        scanner-lib

        # Sets the library as a shared library.
        SHARED

        # Provides a relative path to your source file(s).
        ${CMAKE_CURRENT_LIST_DIR}/src/scanner.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/detector.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/sweeper.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Logger.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/UTM.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/motionDetector.cpp)

add_library( # Sets the name of the library.
        nasagrid

        # Sets the library as a shared library.
        SHARED

        # Provides a relative path to your source file(s).
        ${CMAKE_CURRENT_LIST_DIR}/../nasagrid/gridsquare.cc
        ${CMAKE_CURRENT_LIST_DIR}/../nasagrid/nasagridsquare.cc
        ${CMAKE_CURRENT_LIST_DIR}/../nasagrid/utils.cc)

find_library(jnigraphics-lib jnigraphics)
find_library(log-lib log)

link_directories( ${Boost_LIBRARY_DIR} )

target_link_libraries(
        nasagrid

        ${log-lib}
        )

target_link_libraries(
        scanner-lib

        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${Boost_LIBRARIES}
        nasagrid
        )



