cmake_minimum_required(VERSION 3.0.2)
project(visual_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  arm_follow
  geometry_msgs
  roscpp
  rospy
  roslib
  std_msgs
  tf2
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS arm_follow geometry_msgs roscpp rospy roslib std_msgs tf2
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ executable
add_executable(visual_robot_node src/visual_robot_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(visual_robot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(visual_robot_node
  ${catkin_LIBRARIES}
)
