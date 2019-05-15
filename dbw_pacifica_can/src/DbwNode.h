/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 New Eagle
 *  Copyright (c) 2015-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef _DBW_NODE_H_
#define _DBW_NODE_H_

#include <ros/ros.h>

// ROS messages
#include <can_msgs/Frame.h>
#include <dbw_pacifica_msgs/BrakeCmd.h>
#include <dbw_pacifica_msgs/BrakeReport.h>
#include <dbw_pacifica_msgs/AcceleratorPedalCmd.h>
#include <dbw_pacifica_msgs/AcceleratorPedalReport.h>
#include <dbw_pacifica_msgs/SteeringCmd.h>
#include <dbw_pacifica_msgs/SteeringReport.h>
#include <dbw_pacifica_msgs/GearCmd.h>
#include <dbw_pacifica_msgs/GearReport.h>
#include <dbw_pacifica_msgs/MiscCmd.h>
#include <dbw_pacifica_msgs/MiscReport.h>
#include <dbw_pacifica_msgs/WheelPositionReport.h>
#include <dbw_pacifica_msgs/WheelSpeedReport.h>
#include <dbw_pacifica_msgs/TirePressureReport.h>
#include <dbw_pacifica_msgs/SurroundReport.h>
#include <dbw_pacifica_msgs/DriverInputReport.h>
#include <dbw_pacifica_msgs/LowVoltageSystemReport.h>
#include <dbw_pacifica_msgs/ActuatorControlMode.h>
#include <dbw_pacifica_msgs/Brake2Report.h>
#include <dbw_pacifica_msgs/Steering2Report.h>
#include <dbw_pacifica_msgs/GlobalEnableCmd.h>
#include <dbw_pacifica_msgs/FaultActionsReport.h>
#include <dbw_pacifica_msgs/HmiGlobalEnableReport.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

//#include <dbc/DbcUtilities.h>
#include <dbc/DbcMessage.h>
#include <dbc/DbcSignal.h>
#include <dbc/Dbc.h>
#include <dbc/DbcBuilder.h>

#include <pdu_msgs/RelayCommand.h>
#include <pdu_msgs/RelayState.h>

namespace dbw_pacifica_can
{

class DbwNode
{
public:
  DbwNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
  ~DbwNode();

private:
  void timerCallback(const ros::TimerEvent& event);
  void recvEnable(const std_msgs::Empty::ConstPtr& msg);
  void recvDisable(const std_msgs::Empty::ConstPtr& msg);
  void recvCAN(const can_msgs::Frame::ConstPtr& msg);
  void recvCanImu(const std::vector<can_msgs::Frame::ConstPtr> &msgs);
  void recvCanGps(const std::vector<can_msgs::Frame::ConstPtr> &msgs);
  void recvBrakeCmd(const dbw_pacifica_msgs::BrakeCmd::ConstPtr& msg);
  void recvAcceleratorPedalCmd(const dbw_pacifica_msgs::AcceleratorPedalCmd::ConstPtr& msg);
  void recvSteeringCmd(const dbw_pacifica_msgs::SteeringCmd::ConstPtr& msg);
  void recvGearCmd(const dbw_pacifica_msgs::GearCmd::ConstPtr& msg);
  void recvMiscCmd(const dbw_pacifica_msgs::MiscCmd::ConstPtr& msg);
  void recvGlobalEnableCmd(const dbw_pacifica_msgs::GlobalEnableCmd::ConstPtr& msg);

  ros::Timer timer_;
  bool prev_enable_;
  bool enable_;
  bool override_brake_;
  bool override_accelerator_pedal_;
  bool override_steering_;
  bool override_gear_;
  bool fault_brakes_;
  bool fault_accelerator_pedal_;
  bool fault_steering_;
  bool fault_steering_cal_;
  bool fault_watchdog_;
  bool fault_watchdog_using_brakes_;
  bool fault_watchdog_warned_;
  bool timeout_brakes_;
  bool timeout_accelerator_pedal_;
  bool timeout_steering_;
  bool enabled_brakes_;
  bool enabled_accelerator_pedal_;
  bool enabled_steering_;
  bool gear_warned_;
  inline bool fault() { return fault_brakes_ || fault_accelerator_pedal_ || fault_steering_ || fault_steering_cal_ || fault_watchdog_; }
  inline bool override() { return override_brake_ || override_accelerator_pedal_ || override_steering_ || override_gear_; }
  inline bool clear() { return enable_ && override(); }
  inline bool enabled() { return enable_ && !fault() && !override(); }
  bool publishDbwEnabled();
  void enableSystem();
  void disableSystem();
  void buttonCancel();
  void overrideBrake(bool override);
  void overrideAcceleratorPedal(bool override);
  void overrideSteering(bool override);
  void overrideGear(bool override);
  void timeoutBrake(bool timeout, bool enabled);
  void timeoutAcceleratorPedal(bool timeout, bool enabled);
  void timeoutSteering(bool timeout, bool enabled);
  void faultBrakes(bool fault);
  void faultAcceleratorPedal(bool fault);
  void faultSteering(bool fault);
  void faultSteeringCal(bool fault);
  void faultWatchdog(bool fault, uint8_t src, bool braking);
  void faultWatchdog(bool fault, uint8_t src = 0);

  enum {
    JOINT_FL = 0, // Front left wheel
    JOINT_FR, // Front right wheel
    JOINT_RL, // Rear left wheel
    JOINT_RR, // Rear right wheel
    JOINT_SL, // Steering left
    JOINT_SR, // Steering right
    JOINT_COUNT, // Number of joints
  };
  sensor_msgs::JointState joint_state_;
  void publishJointStates(const ros::Time &stamp, const dbw_pacifica_msgs::WheelSpeedReport *wheels, const dbw_pacifica_msgs::SteeringReport *steering);
  // Licensing
  std::string vin_;

  // Frame ID
  std::string frame_id_;

  // Buttons (enable/disable)
  bool buttons_;

  // Ackermann steering
  double acker_wheelbase_;
  double acker_track_;
  double steering_ratio_;

  // Subscribed topics
  ros::Subscriber sub_enable_;
  ros::Subscriber sub_disable_;
  ros::Subscriber sub_can_;
  ros::Subscriber sub_brake_;
  ros::Subscriber sub_accelerator_pedal_;
  ros::Subscriber sub_steering_;
  ros::Subscriber sub_gear_;
  ros::Subscriber sub_misc_;
  ros::Subscriber sub_global_enable_;

  // Published topics
  ros::Publisher pub_can_;
  ros::Publisher pub_brake_;
  ros::Publisher pub_accel_pedal_;
  ros::Publisher pub_steering_;
  ros::Publisher pub_gear_;
  ros::Publisher pub_misc_;
  ros::Publisher pub_wheel_speeds_;
  ros::Publisher pub_wheel_positions_;
  ros::Publisher pub_tire_pressure_;
  ros::Publisher pub_surround_;
  ros::Publisher pub_imu_;
  ros::Publisher pub_joint_states_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_vin_;
  ros::Publisher pub_sys_enable_;
  ros::Publisher pub_driver_input_;
  ros::Publisher pub_low_voltage_system_;

  ros::Publisher pub_brake_2_report_;
  ros::Publisher pub_steering_2_report_;
  ros::Publisher pub_fault_actions_report_;
  ros::Publisher pub_hmi_global_enable_report_;

  NewEagle::Dbc dbwDbc_;
  std::string dbcFile_;

  // Test stuff
  ros::Publisher pdu1_relay_pub_;
  uint32_t count_;
};

} // dbw_pacifica_can

#endif // _DBW_NODE_H_

