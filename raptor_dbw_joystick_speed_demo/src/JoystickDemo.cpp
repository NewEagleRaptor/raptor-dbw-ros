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

#include "JoystickDemo.h"

namespace joystick_demo
{

JoystickDemo::JoystickDemo(ros::NodeHandle &node, ros::NodeHandle &priv_nh) : counter_(0)
{
  joy_.axes.resize(AXIS_COUNT, 0);
  joy_.buttons.resize(BTN_COUNT, 0);

  ignore_ = false;
  enable_ = true;
  count_ = false;
  svel_ = 0.0;
  priv_nh.getParam("ignore", ignore_);
  priv_nh.getParam("enable", enable_);
  priv_nh.getParam("count", count_);
  priv_nh.getParam("svel", svel_);

  sub_joy_ = node.subscribe("/joy", 1, &JoystickDemo::recvJoy, this);

  data_.brake_joy = 0.0;
  data_.gear_cmd = raptor_dbw_msgs::Gear::NONE;
  data_.steering_joy = 0.0;
  data_.steering_mult = false;
  data_.accelerator_pedal_joy = 0.0;
  data_.turn_signal_cmd = raptor_dbw_msgs::TurnSignal::NONE;
  data_.joy_accelerator_pedal_valid = false;
  data_.joy_brake_valid = false;
  data_.accel_decel_limits = 3;

  pub_accelerator_pedal_ = node.advertise<raptor_dbw_msgs::AcceleratorPedalCmd>("accelerator_pedal_cmd", 1);
  pub_brake_ = node.advertise<raptor_dbw_msgs::BrakeCmd>("brake_cmd", 1);
  pub_misc_ = node.advertise<raptor_dbw_msgs::MiscCmd>("misc_cmd", 1);
  pub_steering_ = node.advertise<raptor_dbw_msgs::SteeringCmd>("steering_cmd", 1);
  pub_global_enable_ = node.advertise<raptor_dbw_msgs::GlobalEnableCmd>("global_enable_cmd", 1);
  pub_gear_ = node.advertise<raptor_dbw_msgs::GearCmd>("gear_cmd", 1);
  if (enable_) {
    pub_enable_ = node.advertise<std_msgs::Empty>("enable", 1);
    pub_disable_ = node.advertise<std_msgs::Empty>("disable", 1);
  }

  timer_ = node.createTimer(ros::Duration(0.02), &JoystickDemo::cmdCallback, this);
}

void JoystickDemo::cmdCallback(const ros::TimerEvent& event)
{
  // Detect joy timeouts and reset
  if (event.current_real - data_.stamp > ros::Duration(0.1)) {
    data_.joy_accelerator_pedal_valid = false;
    data_.joy_brake_valid = false;
    return;
  }

  // Optional watchdog counter
  if (count_) {
    counter_++;
    if (counter_ > 15)
    {
      counter_ = 0;
    }
  }

  // Accelerator Pedal
  raptor_dbw_msgs::AcceleratorPedalCmd accelerator_pedal_msg;
  accelerator_pedal_msg.enable = true;
  accelerator_pedal_msg.ignore = ignore_;
  accelerator_pedal_msg.rolling_counter = counter_;
  //accelerator_pedal_msg.pedal_cmd = data_.accelerator_pedal_joy * 100;
  accelerator_pedal_msg.speed_cmd = data_.accelerator_pedal_joy; // * 10 * 0.44704;
  accelerator_pedal_msg.road_slope = 0;
  accelerator_pedal_msg.accel_limit = data_.accel_decel_limits;
  accelerator_pedal_msg.control_type.value = raptor_dbw_msgs::ActuatorControlMode::closed_loop_vehicle;
  pub_accelerator_pedal_.publish(accelerator_pedal_msg);

  // Brake
  raptor_dbw_msgs::BrakeCmd brake_msg;
  brake_msg.enable = true;
  brake_msg.rolling_counter = counter_;
  brake_msg.pedal_cmd = data_.brake_joy * 100;
  brake_msg.control_type.value = raptor_dbw_msgs::ActuatorControlMode::closed_loop_vehicle;
  brake_msg.decel_limit = data_.accel_decel_limits;
  pub_brake_.publish(brake_msg);

  // Steering
  raptor_dbw_msgs::SteeringCmd steering_msg;
  steering_msg.enable = true;
  steering_msg.ignore = ignore_;
  steering_msg.rolling_counter = counter_;
  steering_msg.angle_cmd = data_.steering_joy;
  steering_msg.angle_velocity = svel_;
  steering_msg.control_type.value = raptor_dbw_msgs::ActuatorControlMode::closed_loop_actuator;
  if (!data_.steering_mult) {
    steering_msg.angle_cmd *= 0.5;
  }
  pub_steering_.publish(steering_msg);

  // Gear
  raptor_dbw_msgs::GearCmd gear_msg;
  gear_msg.cmd.gear = data_.gear_cmd;
  gear_msg.enable = true;
  gear_msg.rolling_counter = counter_;
  pub_gear_.publish(gear_msg);

  // Turn signal
  raptor_dbw_msgs::MiscCmd misc_msg;
  misc_msg.cmd.value = data_.turn_signal_cmd;
  misc_msg.rolling_counter = counter_;
  pub_misc_.publish(misc_msg);

  raptor_dbw_msgs::GlobalEnableCmd globalEnable_msg;
  globalEnable_msg.global_enable = true;
  globalEnable_msg.enable_joystick_limits = true;
  globalEnable_msg.rolling_counter = counter_;
  pub_global_enable_.publish(globalEnable_msg);
}

void JoystickDemo::recvJoy(const sensor_msgs::Joy::ConstPtr& msg)
{
  // Check for expected sizes
  if (msg->axes.size() != (size_t)AXIS_COUNT) {
    ROS_ERROR("Expected %zu joy axis count, received %zu", (size_t)AXIS_COUNT, msg->axes.size());
    return;
  }
  if (msg->buttons.size() != (size_t)BTN_COUNT) {
    ROS_ERROR("Expected %zu joy button count, received %zu", (size_t)BTN_COUNT, msg->buttons.size());
    return;
  }

  // Handle joystick startup
  if (msg->axes[AXIS_ACCELERATOR_PEDAL] != 0.0) {
    data_.joy_accelerator_pedal_valid = true;
  }
  if (msg->axes[AXIS_BRAKE] != 0.0) {
    data_.joy_brake_valid = true;
  }

  data_.accel_decel_limits = 3;

  // Accelerator pedal
  if (data_.joy_accelerator_pedal_valid) {
    //data_.accelerator_pedal_joy = 0.5 - 0.5 * msg->axes[AXIS_ACCELERATOR_PEDAL];
    if (msg->axes[AXIS_ACCELERATOR_PEDAL] < 0.0) {
      data_.accel_decel_limits = 0;
    }
  }

  // Brake
  if (data_.joy_brake_valid) {
    //data_.brake_joy = 0; //0.5 - 0.5 * msg->axes[AXIS_BRAKE];
    if (msg->axes[AXIS_BRAKE] < 0.0) {
      data_.accelerator_pedal_joy = 0;
    }
  }

  // Gear
  if (msg->buttons[BTN_PARK]) {
    data_.gear_cmd = raptor_dbw_msgs::Gear::PARK;
  } else if (msg->buttons[BTN_REVERSE]) {
    data_.gear_cmd = raptor_dbw_msgs::Gear::REVERSE;
  } else if (msg->buttons[BTN_DRIVE]) {
    data_.gear_cmd = raptor_dbw_msgs::Gear::DRIVE;
  } else if (msg->buttons[BTN_NEUTRAL]) {
    data_.gear_cmd = raptor_dbw_msgs::Gear::NEUTRAL;
  } else {
    data_.gear_cmd = raptor_dbw_msgs::Gear::NONE;
  }

  // Steering
  data_.steering_joy = 470.0 * M_PI / 180.0 * ((fabs(msg->axes[AXIS_STEER_1]) > fabs(msg->axes[AXIS_STEER_2])) ? msg->axes[AXIS_STEER_1] : msg->axes[AXIS_STEER_2]);
  data_.steering_mult = msg->buttons[BTN_STEER_MULT_1] || msg->buttons[BTN_STEER_MULT_2];

  // Speed increment
  if (msg->axes[7] != joy_.axes[7]) {

        if (msg->axes[7] < -0.5) {
          data_.accelerator_pedal_joy -= 0.44704;
          if (data_.accelerator_pedal_joy < 0)
          {
            data_.accelerator_pedal_joy = 0;
          }
        } else if (msg->axes[7] > 0.5) {
          data_.accelerator_pedal_joy += 0.44704;
          if (data_.accelerator_pedal_joy > 4.4704)
          {
            data_.accelerator_pedal_joy = 4.4704;
          }
        }
    }

  // Turn signal
  if (msg->axes[AXIS_TURN_SIG] != joy_.axes[AXIS_TURN_SIG]) {
    switch (data_.turn_signal_cmd) {
      case raptor_dbw_msgs::TurnSignal::NONE:
        if (msg->axes[AXIS_TURN_SIG] < -0.5) {
          data_.turn_signal_cmd = raptor_dbw_msgs::TurnSignal::RIGHT;
        } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
          data_.turn_signal_cmd = raptor_dbw_msgs::TurnSignal::LEFT;
        }
        break;
      case raptor_dbw_msgs::TurnSignal::LEFT:
        if (msg->axes[AXIS_TURN_SIG] < -0.5) {
          data_.turn_signal_cmd = raptor_dbw_msgs::TurnSignal::RIGHT;
        } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
          data_.turn_signal_cmd = raptor_dbw_msgs::TurnSignal::NONE;
        }
        break;
      case raptor_dbw_msgs::TurnSignal::RIGHT:
        if (msg->axes[AXIS_TURN_SIG] < -0.5) {
          data_.turn_signal_cmd = raptor_dbw_msgs::TurnSignal::NONE;
        } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
          data_.turn_signal_cmd = raptor_dbw_msgs::TurnSignal::LEFT;
        }
        break;
    }
  }

  // Optional enable and disable buttons
  if (enable_) {
    const std_msgs::Empty empty;
    if (msg->buttons[BTN_ENABLE]) {
      pub_enable_.publish(empty);
    }
    if (msg->buttons[BTN_DISABLE]) {
      pub_disable_.publish(empty);
    }
  }

  data_.stamp = ros::Time::now();
  joy_ = *msg;
}

}
