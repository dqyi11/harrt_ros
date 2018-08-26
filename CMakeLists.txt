# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)

# Declare the name of the CMake Project
project(harrt_ros)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             geometry_msgs
             nav_msgs
             message_generation
            )

add_message_files (
  FILES
  int16_costmap.msg
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
  nav_msgs
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
                    )

add_executable(harrt_node src/harrt_service.cpp
                          src/harrt_service_node.cpp
              )

add_dependencies(harrt_node harrt_ros_generate_messages_cpp)

target_link_libraries(harrt_node ${catkin_LIBRARIES}
                     )

install(TARGETS harrt_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
