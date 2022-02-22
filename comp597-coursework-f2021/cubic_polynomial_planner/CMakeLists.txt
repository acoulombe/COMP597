cmake_minimum_required(VERSION 3.10)
project(cubic_polynomial_planner)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslaunch
    geometry_msgs
    gazebo_msgs
    genmsg
)

add_service_files(FILES target_pose.srv)

generate_messages(DEPENDENCIES )

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs gazebo_msgs genmsg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare cpp executables
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencpp
)



roslaunch_add_file_check(launch)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)



