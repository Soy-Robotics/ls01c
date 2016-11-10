cmake_minimum_required(VERSION 2.8.3)
project(ls01c)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
  DEPENDS
)

catkin_add_env_hooks(25.ls01c
  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

include_directories(
  ${catkin_INCLUDE_DIRS}/src
)

add_executable(LS01C_node src/talker.cpp src/uart_driver.cpp)
target_link_libraries(LS01C_node ${catkin_LIBRARIES})


## Mark executables and/or libraries for installation
install(TARGETS LS01C_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
