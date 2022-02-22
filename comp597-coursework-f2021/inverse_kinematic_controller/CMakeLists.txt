cmake_minimum_required(VERSION 3.10)
project(inverse_kinematic_controller)

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

add_service_files(FILES target_pose.srv)

generate_messages(DEPENDENCIES )

catkin_package(
	INCLUDE_DIRS
    #	include  
    	${Eigen_INCLUDE_DIRS}
    	${PINOCCHIO_INCLUDE_DIRS}
  	LIBRARIES
  		pinocchio    	  		
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
	#include
	${catkin_INCLUDE_DIRS}		
	${Eigen3_INCLUDE_DIRS}		
	${PINOCCHIO_INCLUDE_DIRS}
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PINOCCHIO_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencpp
)

#############
## Install ##
#############

# Mark cpp header files for installation
install(
  	DIRECTORY include/${PROJECT_NAME}/
 	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  	FILES_MATCHING PATTERN "*.hpp"
)