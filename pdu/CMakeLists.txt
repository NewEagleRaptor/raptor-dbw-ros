cmake_minimum_required(VERSION 2.8.3)
project(pdu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  can_msgs
  pdu_msgs
  can_dbc_parser
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_pdu_node
  src/node.cpp
  src/pdu.cpp
)

add_dependencies(${PROJECT_NAME}_pdu_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_pdu_node
  ${catkin_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}_pdu_node PROPERTIES OUTPUT_NAME pdu_node PREFIX "")

install(TARGETS ${PROJECT_NAME}_pdu_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
