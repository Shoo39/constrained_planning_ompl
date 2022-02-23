cmake_minimum_required(VERSION 2.8.3)
project(kin_constr_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  moveit_core
  moveit_msgs
  roscpp
  moveit_visual_tools
)

find_package(ompl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS moveit_core moveit_msgs
  # DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/kin_constraint.cpp
  src/state_validity_checker.cpp
  src/threadsafe_state_storage.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/
  DESTINATION
    ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

##########
## Test ##
##########


