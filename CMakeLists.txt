# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)

# Declare the name of the CMake Project
project(harrt_ros)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             geometry_msgs
             message_generation
            )

# Find package homotopy
find_path(HOMOTOPY_INCLUDE_DIR tpp/homotopy/worldmap.h
    HINTS ${HOMOTOPY_INCLUDE_DIR} ${HOMOTOPY_INCLUDE_DIRS})
#message("HOMOTOPY_INCLUDE_DIR:")
#message(${HOMOTOPY_INCLUDE_DIR})

find_library(HOMOTOPY_LIBRARY tpp-homotopy
    HINTS ${HOMOTOPY_LIB_DIR} ${HARRT_LIBRARY_DIRS})
#message("HOMOTOPY_LIBRARY:")
#message(${HOMOTOPY_LIBRARY})

# Find package harrt
find_path(HARRT_INCLUDE_DIR tpp/harrt/birrtstar.h
    HINTS ${HARRT_INCLUDE_DIR} ${HARRT_INCLUDE_DIRS})
#message("HARRT_INCLUDE_DIR:")
#message(${HARRT_INCLUDE_DIR})

find_library(HARRT_LIBRARY tpp-harrts tpp-homotopy
    HINTS ${HARRT_LIB_DIR} ${HARRT_LIBRARY_DIRS})
#message("HARRT_LIBRARY:")
#message(${HARRT_LIBRARY})

# search for the libxml2 package, exit if not found
find_package(LibXml2)
if( NOT LIBXML2_FOUND )
  message( FATAL_ERROR "libxml2 not found\n" )
  return()
endif()
include_directories(${LIBXML2_INCLUDE_DIR})

# search for CGAL, exit if not found
find_package(CGAL QUIET COMPONENTS Core)
if( NOT CGAL_FOUND )
  message( FATAL_ERROR "CGAL not found\n" ) 
  return()
endif()
#message( "CGAL FOUND ")
#message( "CGAL_USE_FILE: " ${CGAL_USE_FILE} )
#message( "CGAL_INCLUDE_DIR: " ${CGAL_INCLUDE_DIR} )
include(${CGAL_USE_FILE})
include_directories(${CGAL_INCLUDE_DIR})

# search for Open CV, exit if not found
find_package( OpenCV REQUIRED )
if( NOT OpenCV_FOUND ) 
  message( FATAL_ERROR "OpenCV not found\n")
  return()
endif()
#message( "OpenCV FOUND ")
#message( "OpenCV_USE_FILE: " ${OpenCV_USE_FILE} )
#message( "OpenCV_INCLUDE_DIR: " ${OpenCV_INCLUDE_DIRS} )
#message( "OpenCV_LIB:" ${OpenCV_LIBRARIES} )
include_directories(${OpenCV_INCLUDE_DIRS})

add_message_files (
  FILES
  int16_image.msg
  harrt_init.msg
  single_objective_path.msg
)

add_service_files(
  FILES
  harrt_initialize.srv
  harrt_continue.srv
)

generate_messages (
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# Declare this project as a catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hello_world
   CATKIN_DEPENDS message_runtime
)

# Add the headers from roscpp
include_directories(include 
                    ${catkin_INCLUDE_DIRS}
                    ${HOMOTOPY_INCLUDE_DIR}
                    ${HARRT_INCLUDE_DIR}
                    )

add_executable(harrt_node src/harrt_service.cpp
                          src/harrt_service_node.cpp
              )

add_dependencies(harrt_node harrt_ros_generate_messages_cpp)

target_link_libraries(harrt_node ${catkin_LIBRARIES}
                                 ${HOMOTOPY_LIBRARY}
                                 ${HARRT_LIBRARY}
                                 ${CGAL_LIBRARY}
                                 ${OpenCV_LIBRARIES}
                     )

install(TARGETS harrt_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
