cmake_minimum_required(VERSION 2.8.3)
project(eddiebot_joy)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs sensor_msgs joy)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs sensor_msgs joy
  DEPENDS
)

###########
## Build ##
###########

add_executable(eddiebot_joy src/eddie_joy.cpp)
target_link_libraries(eddiebot_joy ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation


## Mark executables and/or libraries for installation
install(TARGETS eddiebot_joy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
