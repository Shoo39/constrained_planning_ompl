cmake_minimum_required(VERSION 2.8.3)
project(kin_constr_moveit_plugin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  pluginlib
  roscpp
  kin_constr_planner
  moveit_planners_ompl
  moveit_ros_planning
)

find_package(ompl REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS moveit_ros_planning
#  CATKIN_DEPENDS moveit_core moveit_msgs pluginlib roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS} # TODO see line 16
)

add_library(
  ${PROJECT_NAME}
  src/kin_constr_planner_manager.cpp
  src/kin_constr_planning_context.cpp
  src/kin_constr_ompl_interface.cpp
  src/kin_constr_goal_sampler.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES} # TODO see line 16
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

install(
  FILES
    kin_constr_interface_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)