cmake_minimum_required(VERSION 3.10)
project(husky_teleop_controller)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslaunch
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)



