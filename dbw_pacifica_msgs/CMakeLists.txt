cmake_minimum_required(VERSION 2.8.3)
project(dbw_pacifica_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(DIRECTORY msg FILES
  BrakeCmd.msg
  BrakeReport.msg
  Gear.msg
  GearCmd.msg
  GearReject.msg
  GearReport.msg
  MiscReport.msg
  ParkingBrake.msg
  SteeringCmd.msg
  SteeringReport.msg
  AcceleratorPedalCmd.msg
  AcceleratorPedalReport.msg
  TirePressureReport.msg
  TurnSignal.msg
  MiscCmd.msg
  TwistCmd.msg
  WheelPositionReport.msg
  WheelSpeedReport.msg
  Wiper.msg
  SonarArcNum.msg
  SurroundReport.msg
  WatchdogStatus.msg
  DriverInputReport.msg
  HighBeam.msg
  ControlMode.msg
  SteeringCommandType.msg
  DoorRequest.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)

install(DIRECTORY bmr
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.bmr"
)
