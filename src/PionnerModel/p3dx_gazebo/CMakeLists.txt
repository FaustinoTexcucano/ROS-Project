cmake_minimum_required(VERSION 2.8.3)
project(p3dx_gazebo)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  p3dx_control
  p3dx_description

  laser_geometry
  pcl_ros
)

find_package( PCL REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES p3dx_gazebo
#  CATKIN_DEPENDS gazebo_ros p3dx_control p3dx_description
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(p3dx_gazebo
#   src/${PROJECT_NAME}/p3dx_gazebo.cpp
# )

## Declare a cpp executable
# add_executable(p3dx_gazebo_node src/p3dx_gazebo_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(p3dx_gazebo_node p3dx_gazebo_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(p3dx_gazebo_node
#   ${catkin_LIBRARIES}
# )

#add_executable(RobotDriver src/RobotDriver.cpp)
#target_link_libraries(RobotDriver ${catkin_LIBRARIES})
#add_dependencies(RobotDriver p3dx_gazebo_generate_messages_cpp)

#find_package(Eigen3 REQUIRED) include_directories(${EIGEN3_INCLUDE_DIR})
#include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})


add_library(graph src/graph.cpp)
add_library(node src/node.cpp)
add_library(robotmotion src/robotmotion.cpp)
add_library(plotter src/plotter.cpp)
add_executable(RobotDriver src/RobotDriver.cpp)
target_link_libraries(RobotDriver ${catkin_LIBRARIES})
target_link_libraries(RobotDriver graph)
target_link_libraries(RobotDriver node)
target_link_libraries(RobotDriver plotter)
target_link_libraries(RobotDriver robotmotion)

