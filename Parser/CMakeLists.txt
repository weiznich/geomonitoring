FIND_PACKAGE(Boost COMPONENTS system REQUIRED )
FIND_PACKAGE(PCL COMPONENTS common visualization REQUIRED)

#INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} ${PCL_VISUALIZATION_INCLUDE_DIR} ${PCL_COMMON_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

project(PARSER)

set(
PARSER_SRCS 
	PlyParser.cpp
)
set(
PARSER_HDRS 
	PlyParser.h
)
include_directories(${Boost_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(parser STATIC  ${PARSER_SRCS} ${PARSER_HDRS})

ADD_EXECUTABLE( minimalParser testViewer.cpp  )

TARGET_LINK_LIBRARIES( minimalParser parser ${Boost_SYSTEM_LIBRARIES} ${PCL_LIBRARIES})
