cmake_minimum_required(VERSION 2.8.3)
project(pdu_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(DIRECTORY msg FILES
  FuseReport.msg
  FuseStatus.msg
  RelayCommand.msg
  RelayReport.msg
  RelayState.msg
  RelayStatus.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
)

install(DIRECTORY bmr
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.bmr"
)
