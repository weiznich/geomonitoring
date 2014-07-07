cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

if (NOT MSVC)
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g -Wall -Wextra -mtune=native -march=native -fdiagnostics-color=always")
endif (NOT MSVC)
set(CMAKE_BINARY_DIR "${CMAKE_SOURCE_DIR}/bin")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

FIND_PACKAGE(PCL REQUIRED)
FIND_PACKAGE(Boost COMPONENTS random REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(Parser)

add_subdirectory(ICP)

