cmake_minimum_required(VERSION 2.8.3)
project(dbw_pacifica_joystick_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  dbw_pacifica_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_joystick_demo
  src/joystick_demo.cpp
  src/JoystickDemo.cpp
)

target_link_libraries(${PROJECT_NAME}_joystick_demo
  ${catkin_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}_joystick_demo PROPERTIES OUTPUT_NAME joystick_demo PREFIX "")

install(TARGETS ${PROJECT_NAME}_joystick_demo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
