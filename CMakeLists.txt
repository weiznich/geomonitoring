set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g -Wall -Wextra")
set(CMAKE_BINARY_DIR "${CMAKE_SOURCE_DIR}/bin")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_executable (helloWorld main.cpp) 
