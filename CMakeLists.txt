cmake_minimum_required(VERSION 3.1)
project(dynamic_robot_state_publisher)

find_package(catkin REQUIRED
  COMPONENTS dynamic_reconfigure kdl_parser robot_state_publisher roscpp sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/DynamicRobotState.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamic_reconfigure kdl_parser robot_state_publisher roscpp sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/dynamic_robot_state_publisher_node.cpp src/joint_state_listener.cpp src/robot_state_publisher.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_dynamic_publisher test/test_dynamic_publisher.test test/test_dynamic_publisher.cpp)
  target_link_libraries(test_dynamic_publisher ${catkin_LIBRARIES})
  add_dependencies(test_dynamic_publisher ${PROJECT_NAME})
endif()