cmake_minimum_required(VERSION 3.10)
project(base_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    gazebo_msgs
    roslint
)

###################################
## catkin specific configuration ##
###################################
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    gazebo_msgs
  LIBRARIES base_controller 
  DEPENDS 
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  src/base_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
