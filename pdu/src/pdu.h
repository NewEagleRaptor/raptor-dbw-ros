/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 New Eagle
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
 *   * Neither the name of New Eagle nor the names of its
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
 
#ifndef NEWEAGLE_PDU_H_
#define NEWEAGLE_PDU_H_

#include <ros/ros.h>

// ROS messages
#include <can_msgs/Frame.h>
#include <pdu_msgs/FuseReport.h>
#include <pdu_msgs/RelayReport.h>
#include <pdu_msgs/RelayCommand.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <can_dbc_parser/DbcMessage.h>
#include <can_dbc_parser/DbcSignal.h>
#include <can_dbc_parser/Dbc.h>
#include <can_dbc_parser/DbcBuilder.h>

namespace NewEagle
{
  class pdu
  {
    enum {
      RELAY_STATUS_BASE_ADDR = 0x18ffa100,
      FUSE_STATUS_BASE_ADDR = 0x18ffa000,
      RELAY_COMMAND_BASE_ADDR = 0x18ef0000
    };

    public:
      pdu(ros::NodeHandle &node, ros::NodeHandle &priv_nh);

    private:
      uint32_t id_;
      uint32_t relayCommandAddr_;
      uint32_t relayStatusAddr_;
      uint32_t fuseStatusAddr_;

      uint32_t count_;

      NewEagle::Dbc pduDbc_;
      std::string pduFile_;

      void recvCAN(const can_msgs::Frame::ConstPtr& msg);
      void recvRelayCmd(const pdu_msgs::RelayCommand::ConstPtr& msg);

      // Subscribed topics
      ros::Subscriber sub_can_;
      ros::Subscriber sub_relay_cmd_;

      // Published topics
      ros::Publisher pub_can_;
      ros::Publisher fuse_report_pub_;
      ros::Publisher relay_report_pub_;
  };
}

#endif /* NEWEAGLE_PDU_H_ */
