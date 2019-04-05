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

// Example block for PDU relay blocks.
// pdu_msgs::RelayCommand msg;
// msg.relay_1.value = pdu_msgs::RelayState::RELAY_ON;
// pdu1_relay_pub_.publish(msg);

#include <sstream>

#include "pdu.h"

namespace NewEagle
{
  pdu::pdu(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
  {
    int32_t id;
    priv_nh.getParam("id", id);
    priv_nh.getParam("pdu_dbc_file", pduFile_);

    id_ = (uint32_t)id;

    relayCommandAddr_ = RELAY_COMMAND_BASE_ADDR + (id_ * 256);
    relayStatusAddr_ = RELAY_STATUS_BASE_ADDR + id_;
    fuseStatusAddr_ = FUSE_STATUS_BASE_ADDR + id_;

    // This should be a class, initialized with a unique CAN ID
    pduDbc_ = NewEagle::DbcBuilder().NewDbc(pduFile_);

    count_ = 0;
    // Set up Publishers
    pub_can_ = node.advertise<can_msgs::Frame>("can_tx", 10);

    relay_report_pub_ = node.advertise<pdu_msgs::RelayReport>("relay_report", 2);
    fuse_report_pub_ = node.advertise<pdu_msgs::FuseReport>("fuse_report", 2);

    // Set up Subscribers
    sub_can_ = node.subscribe("can_rx", 100, &pdu::recvCAN, this, ros::TransportHints().tcpNoDelay(true));

    sub_relay_cmd_ = node.subscribe("relay_cmd", 1, &pdu::recvRelayCmd, this, ros::TransportHints().tcpNoDelay(true));
  }

  void pdu::recvCAN(const can_msgs::Frame::ConstPtr& msg)
  {
    if (!msg->is_rtr && !msg->is_error && msg->is_extended)
    {
      if (msg->id == relayStatusAddr_)
      {
        ROS_INFO("Relay Status");

        NewEagle::DbcMessage* message = pduDbc_.GetMessage("RelayStatus");
        message->SetFrame(msg);

        pdu_msgs::RelayReport out;

        out.relay_1.value = message->GetSignal("Relay1")->GetResult();
        out.relay_2.value = message->GetSignal("Relay2")->GetResult();
        out.relay_3.value = message->GetSignal("Relay3")->GetResult();
        out.relay_4.value = message->GetSignal("Relay4")->GetResult();
        out.relay_5.value = message->GetSignal("Relay5")->GetResult();
        out.relay_6.value = message->GetSignal("Relay6")->GetResult();
        out.relay_7.value = message->GetSignal("Relay7")->GetResult();
        out.relay_8.value = message->GetSignal("Relay8")->GetResult();

        relay_report_pub_.publish(out);
      }
      else if (msg->id == fuseStatusAddr_)
      {
        ROS_INFO("Fuse Status");

        NewEagle::DbcMessage* message = pduDbc_.GetMessage("FuseStatus");
        message->SetFrame(msg);

        pdu_msgs::FuseReport out;

        out.fuse_1.value = message->GetSignal("Fuse1")->GetResult();
        out.fuse_2.value = message->GetSignal("Fuse2")->GetResult();
        out.fuse_3.value = message->GetSignal("Fuse3")->GetResult();
        out.fuse_4.value = message->GetSignal("Fuse4")->GetResult();
        out.fuse_5.value = message->GetSignal("Fuse5")->GetResult();
        out.fuse_6.value = message->GetSignal("Fuse6")->GetResult();
        out.fuse_7.value = message->GetSignal("Fuse7")->GetResult();
        out.fuse_8.value = message->GetSignal("Fuse8")->GetResult();
        out.fuse_9.value = message->GetSignal("Fuse9")->GetResult();
        out.fuse_10.value = message->GetSignal("Fuse10")->GetResult();
        out.fuse_11.value = message->GetSignal("Fuse11")->GetResult();
        out.fuse_12.value = message->GetSignal("Fuse12")->GetResult();
        out.fuse_13.value = message->GetSignal("Fuse13")->GetResult();
        out.fuse_14.value = message->GetSignal("Fuse14")->GetResult();
        out.fuse_15.value = message->GetSignal("Fuse15")->GetResult();
        out.fuse_16.value = message->GetSignal("Fuse16")->GetResult();

        fuse_report_pub_.publish(out);
      }
    }
  }

  void pdu::recvRelayCmd(const pdu_msgs::RelayCommand::ConstPtr& msg)
  {
    ROS_INFO("Relay Command");

    NewEagle::DbcMessage* message = pduDbc_.GetMessage("RelayCommand");

    message->GetSignal("MessageID")->SetResult(0x80); // Always 0x80
    message->GetSignal("GridAddress")->SetResult(0x00); // Always 0x00

    message->GetSignal("Relay1")->SetResult(msg->relay_1.value);
    message->GetSignal("Relay2")->SetResult(msg->relay_2.value);
    message->GetSignal("Relay3")->SetResult(msg->relay_3.value);
    message->GetSignal("Relay4")->SetResult(msg->relay_4.value);
    message->GetSignal("Relay5")->SetResult(msg->relay_5.value);
    message->GetSignal("Relay6")->SetResult(msg->relay_6.value);
    message->GetSignal("Relay7")->SetResult(msg->relay_7.value);
    message->GetSignal("Relay8")->SetResult(msg->relay_8.value);

    can_msgs::Frame frame = message->GetFrame();

    // DBC file has the base address.  Modify the ID to send to correct device
    frame.id = relayCommandAddr_;

    pub_can_.publish(frame);
  }
}
