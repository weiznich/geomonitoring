FIND_PACKAGE(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


ADD_EXECUTABLE( ICP run_icp.cpp  )

TARGET_LINK_LIBRARIES( ICP parser ${PCL_LIBRARIES})
