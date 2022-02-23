cmake_minimum_required(VERSION 2.8.3)
project(kin_constr_examples)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  eigen_conversions
  pluginlib
  roscpp
  rosbag
  moveit_planners_ompl
  kin_constr_planner
  kin_constr_moveit_plugin
)
find_package(Boost REQUIRED serialization ${EXTRA_BOOST_COMPONENTS})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kin_constr_examples_utils
#  CATKIN_DEPENDS moveit_core moveit_msgs moveit_ros_planning moveit_ros_planning_interface moveit_visual_tools pluginlib roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_utils
  src/utils.cpp
)
target_link_libraries(${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
)



add_executable(test_pipeline_panda src/test_pipeline_panda.cpp)

target_link_libraries(test_pipeline_panda
  ${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
  Boost::serialization
)
