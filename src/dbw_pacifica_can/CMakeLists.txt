cmake_minimum_required(VERSION 2.8.3)
project(dbw_pacifica_can)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  nodelet
  std_msgs
  geometry_msgs
  sensor_msgs
  can_msgs
  dbw_pacifica_msgs
  dbc
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/nodelet.cpp
  src/DbwNode.cpp
)
add_dependencies(${PROJECT_NAME} dbw_pacifica_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_dbw_node
  src/node.cpp
)
add_dependencies(${PROJECT_NAME}_dbw_node dbw_pacifica_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}_dbw_node
  ${PROJECT_NAME}
)
set_target_properties(${PROJECT_NAME}_dbw_node PROPERTIES OUTPUT_NAME dbw_node PREFIX "")

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_dbw_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
)
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
