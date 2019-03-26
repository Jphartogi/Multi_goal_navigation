cmake_minimum_required(VERSION 2.8.3)
project(multi_goal_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  nav_msgs
  std_msgs
  tf
  roscpp
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES multi_goal_driver
  CATKIN_DEPENDS actionlib geometry_msgs nav_msgs std_msgs tf roscpp
#  DEPENDS system_lib
)


add_executable(multi_goal_driver_node src/Multi_goal.cpp)
target_link_libraries(multi_goal_driver_node ${catkin_LIBRARIES})



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)




