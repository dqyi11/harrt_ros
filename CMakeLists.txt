# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)

# Declare the name of the CMake Project
project(morrf_ros)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             geometry_msgs
             message_generation
            )

# Find package morrf
find_path(MORRF_INCLUDE_DIR morrf/morrf.h
    HINTS ${MORRF_INCLUDEDIR} ${MORRF_INCLUDE_DIRS})
#message("MORRF_INCLUDE_DIR:")
#message(${MORRF_INCLUDE_DIR})

find_library(MORRF_LIBRARY morrf
    HINTS ${MORRF_LIBDIR} ${MORRF_LIBRARY_DIRS})
#message("MORRF_LIBRARY:")
#message(${MORRF_LIBRARY})

add_message_files (
  FILES
  int16_image.msg
  morrf_init.msg
  multi_objective_path.msg
)

add_service_files(
  FILES
  morrf_initialize.srv
  morrf_continue.srv
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
include_directories(include ${catkin_INCLUDE_DIRS}
                            ${MORRF_INCLUDE_DIR}
                            )

add_executable(morrf_node src/morrf_service.cpp
                          src/morrf_service_node.cpp
              )

#add_executable(morrf_continue_node src/morrf_continue_node.cpp)

target_link_libraries(morrf_node ${catkin_LIBRARIES}
                                 ${MORRF_LIBRARY}
                     )

#target_link_libraries(morrf_continue_node ${catkin_LIBRARIES}
 #                                         ${MORRF_LIBRARY}
  #                   )

install(TARGETS morrf_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
