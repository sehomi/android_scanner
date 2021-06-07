# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.10.2)

project("scanner")

include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

add_library( # Sets the name of the library.
        scanner-lib

        # Sets the library as a shared library.
        SHARED

        # Provides a relative path to your source file(s).
        ${CMAKE_CURRENT_LIST_DIR}/src/scanner.cpp )



