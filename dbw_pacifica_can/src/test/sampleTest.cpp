#include <ros/ros.h>
#include <gtest/gtest.h>
#include <dbw_pacifica_msgs/GearCmd.h>
#include <can_msgs/Frame.h>

TEST(TestSuite, dbw_pacifica_can_framework)
{
  ASSERT_TRUE(true);
}


bool receivedMessage;
dbw_pacifica_msgs::GearCmd message;

void recvGearCmd(const dbw_pacifica_msgs::GearCmd::ConstPtr& msg)
{
    receivedMessage = true;
//    message = *msg;
}

void can_rx_callback(const can_msgs::Frame::ConstPtr& msg)
{
    receivedMessage = true;
}

TEST(TestSuite, dbw_pacifica_can_framework_pub_gear)
{
  ros::NodeHandle nh;
  
  ros::Subscriber sub = nh.subscribe("can_tx", 0, can_rx_callback);
  
  ros::Publisher pub = nh.advertise<dbw_pacifica_msgs::GearCmd>("gear_cmd", 0);

  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub.getNumPublishers(), 1U);
  
  dbw_pacifica_msgs::GearCmd gear_msg;
  gear_msg.cmd.gear = 4; //data_.gear_cmd;
  gear_msg.enable = true;
  gear_msg.rolling_counter = 0; 
  pub.publish(gear_msg);

  ros::spinOnce(); // Spin so that publication can get to subscription

  EXPECT_TRUE(receivedMessage); // This may or may not be true

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_suite");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}