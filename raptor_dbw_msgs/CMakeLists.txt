cmake_minimum_required(VERSION 2.8.3)
project(raptor_dbw_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(DIRECTORY msg FILES
  AcceleratorPedalCmd.msg
  AcceleratorPedalReport.msg
  ActuatorControlMode.msg
  Brake2Report.msg
  BrakeCmd.msg
  BrakeReport.msg
  DoorRequest.msg
  DriverInputReport.msg 
  Gear.msg
  GearCmd.msg  
  GearReport.msg
  HighBeam.msg
  Ignition.msg
  LowVoltageSystemReport.msg
  MiscCmd.msg
  MiscReport.msg  
  ParkingBrake.msg
  SonarArcNum.msg  
  Steering2Report.msg
  SteeringCmd.msg
  SteeringReport.msg
  SurroundReport.msg  
  TirePressureReport.msg
  TurnSignal.msg  
  TwistCmd.msg
  WatchdogStatus.msg
  WheelPositionReport.msg
  WheelSpeedReport.msg
  WheelSpeedType.msg
  WiperFront.msg  
  WiperRear.msg
  GlobalEnableCmd.msg
  HmiGlobalEnableReport.msg
  OtherActuatorsReport.msg
  LowBeam.msg
  FaultActionsReport.msg
  DoorState.msg
  HighBeamState.msg
  HornState.msg
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
