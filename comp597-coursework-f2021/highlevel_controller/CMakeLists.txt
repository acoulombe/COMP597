cmake_minimum_required(VERSION 3.10)
project(highlevel_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#add_definitions(-Wall -Werror) 					# make sure this is not used

find_package(PkgConfig) 							# find pkg_config
pkg_search_module(Eigen3 REQUIRED eigen3)			# use pkg_config to find eigen3
find_package(pinocchio REQUIRED)					# find pinocchio
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30") 	# something for pinocchio


## Find catkin macros and libraries
find_package(catkin 
	REQUIRED
	COMPONENTS
		roscpp
		geometry_msgs
		sensor_msgs
		genmsg
		roslib
)

add_service_files(FILES 
	target_pose.srv
	pick_and_place.srv
)


generate_messages(DEPENDENCIES )

catkin_package(
	INCLUDE_DIRS
    	include  
    	${Eigen_INCLUDE_DIRS}
    	${PINOCCHIO_INCLUDE_DIRS}
  	LIBRARIES
  		pinocchio
		${PROJECT_NAME}_core		
  	CATKIN_DEPENDS
    	roscpp
    	geometry_msgs    
		sensor_msgs	 	  
		genmsg
		roslib
  	DEPENDS
  	    	
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}		
	${Eigen3_INCLUDE_DIRS}		
	${PINOCCHIO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_core
	src/robot.cpp
	src/potential_field_planner.cpp
	src/inverse_dynamics_controller.cpp
	src/inverse_kinematics_controller.cpp
)
target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_11)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_node.cpp
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_core
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${PINOCCHIO_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${PINOCCHIO_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencpp
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  	DIRECTORY include/${PROJECT_NAME}/
 	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  	FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
