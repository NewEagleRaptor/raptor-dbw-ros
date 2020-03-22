/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018-2019 New Eagle 
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

#ifndef JOYSTICKDEMO_H_
#define JOYSTICKDEMO_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>

#include <raptor_dbw_msgs/AcceleratorPedalCmd.h>
#include <raptor_dbw_msgs/BrakeCmd.h>
#include <raptor_dbw_msgs/SteeringCmd.h>
#include <raptor_dbw_msgs/GearCmd.h>
#include <raptor_dbw_msgs/MiscCmd.h>
#include <raptor_dbw_msgs/GlobalEnableCmd.h>

namespace joystick_demo
{

typedef struct {
  ros::Time stamp;
  float brake_joy;
  float accelerator_pedal_joy;
  float steering_joy;
  bool steering_mult;
  int gear_cmd;
  int turn_signal_cmd;
  bool joy_accelerator_pedal_valid;
  bool joy_brake_valid;
  float accel_decel_limits;
} JoystickDataStruct;

class JoystickDemo {
public:
  JoystickDemo(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
private:
  void recvJoy(const sensor_msgs::Joy::ConstPtr& msg);
  void cmdCallback(const ros::TimerEvent& event);

  // Topics
  ros::Subscriber sub_joy_;
  ros::Publisher pub_accelerator_pedal_;
  ros::Publisher pub_brake_;
  ros::Publisher pub_steering_;
  ros::Publisher pub_gear_;
  ros::Publisher pub_misc_;
  ros::Publisher pub_enable_;
  ros::Publisher pub_disable_;
  ros::Publisher pub_global_enable_;
  // Parameters
  bool ignore_; // Ignore driver overrides
  bool enable_; // Use enable and disable buttons
  bool count_; // Increment counter to enable watchdog
  double svel_; // Steering command speed

  // Variables
  ros::Timer timer_;
  JoystickDataStruct data_;
  sensor_msgs::Joy joy_;
  uint8_t counter_;

  enum {
    BTN_PARK = 3,
    BTN_REVERSE = 1,
    BTN_NEUTRAL = 2,
    BTN_DRIVE = 0,
    BTN_ENABLE = 5,
    BTN_DISABLE = 4,
    BTN_STEER_MULT_1 = 6,
    BTN_STEER_MULT_2 = 7,
    BTN_COUNT = 11,
    AXIS_ACCELERATOR_PEDAL = 5,
    AXIS_BRAKE = 2,
    AXIS_STEER_1 = 0,
    AXIS_STEER_2 = 3,
    AXIS_TURN_SIG = 6,
    AXIS_COUNT = 8,
  };
};

}

#endif /* JOYSTICKDEMO_H_ */
