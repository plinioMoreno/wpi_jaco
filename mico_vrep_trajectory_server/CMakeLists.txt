cmake_minimum_required(VERSION 2.8.3)
project(mico_vrep_action_srv)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib control_msgs ecl_geometry roscpp sensor_msgs std_msgs std_srvs)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(mico_vrep_trajectory_node src/mico_vrep_trajectory_node.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(mico_vrep_trajectory_node
  ${catkin_LIBRARIES}
)
## Add cmake target dependencies of the executable/library
#add_dependencies(mico_vrep_trajectory_node )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS mico_vrep_trajectory_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Copy config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
