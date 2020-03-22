cmake_minimum_required(VERSION 2.8.3)
project(raptor_dbw_joystick_speed_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  raptor_dbw_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_joystick_speed_demo
  src/joystick_demo.cpp
  src/JoystickDemo.cpp
)

add_dependencies(${PROJECT_NAME}_joystick_speed_demo 
  raptor_dbw_msgs_gencpp
)

target_link_libraries(${PROJECT_NAME}_joystick_speed_demo
  ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_joystick_speed_demo PROPERTIES OUTPUT_NAME joystick_speed_demo PREFIX "")

install(TARGETS ${PROJECT_NAME}_joystick_speed_demo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
