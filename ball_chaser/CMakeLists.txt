cmake_minimum_required(VERSION 2.8.3)
project(ball_chaser)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

################################################
## Declare ROS messages, services, and actions #
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DriveToTarget.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_generation roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(drive_bot_node src/drive_bot.cpp)
add_executable(process_image_node src/process_image.cpp)

## Specify libraries to link the executable target against
target_link_libraries(drive_bot_node
  ${catkin_LIBRARIES}
)

target_link_libraries(process_image_node
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies for message generation
add_dependencies(drive_bot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(process_image_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
