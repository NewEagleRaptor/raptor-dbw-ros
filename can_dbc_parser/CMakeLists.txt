cmake_minimum_required(VERSION 2.8.3)
project(can_dbc_parser)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(can_dbc_parser
  src/DbcMessage.cpp
  src/DbcSignal.cpp
  src/Dbc.cpp
  src/LineParser.cpp
  src/DbcBuilder.cpp
)

target_link_libraries(can_dbc_parser
  ${catkin_LIBRARIES}
)

add_dependencies(can_dbc_parser
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
