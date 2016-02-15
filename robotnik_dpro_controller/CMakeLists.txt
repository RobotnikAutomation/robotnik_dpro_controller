cmake_minimum_required(VERSION 2.8.3)
project(robotnik_dpro_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  self_test
)

catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(robotnik_dpro_controller
#   src/${PROJECT_NAME}/robotnik_dpro_controller.cpp
# )

# For debugging via Qt Creator
# SET(CMAKE_BUILD_TYPE Debug)
link_directories(~/catkin_ws/src/robotnik_dpro_controller)

## Declare a cpp executable
add_executable(robotnik_dpro_controller_node src/robotnik_dpro_controller.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(robotnik_dpro_controller_node robotnik_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robotnik_dpro_controller_node
   ${catkin_LIBRARIES}
   -ldxl
)

