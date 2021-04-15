cmake_minimum_required(VERSION 2.8.3)
project(mushr_coordination)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

add_message_files(
  FILES
  GoalPoseArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs
    tf2
    visualization_msgs
    message_runtime
)

include_directories(include 
  ${catkin_INCLUDE_DIRS}
  ${Boost_LIBRARIES}
  libMultiRobotPlanning/include
)

add_executable(mushr_coordination src/nodes/mushr_coor_node.cpp include/mushr_coordination/mushr_coordination.hpp)
target_link_libraries(mushr_coordination ${catkin_LIBRARIES})
add_dependencies(mushr_coordination ${PROJECT_NAME}_generate_messages_cpp)

catkin_install_python(PROGRAMS 
  src/init_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY libMultiRobotPlanning/include/libMultiRobotPlanning/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY libMultiRobotPlanning/example/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

#add_library(libMultiRobotPlanning)
install(TARGETS mushr_coordination
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#