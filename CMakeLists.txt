cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

if (NOT MSVC)
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g -Wall -Wextra")
endif (NOT MSVC)
set(CMAKE_BINARY_DIR "${CMAKE_SOURCE_DIR}/bin")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_subdirectory(Parser)

add_executable (helloWorld main.cpp) 
