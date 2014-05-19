FIND_PACKAGE(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


ADD_EXECUTABLE( testIPC test_ipc.cpp  )

TARGET_LINK_LIBRARIES( testIPC parser ${PCL_LIBRARIES})
