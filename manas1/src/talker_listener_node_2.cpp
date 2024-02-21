
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

void subscriberCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Received: [%s]", msg->data.c_str());

  // Publish to topic2 using the existing NodeHandle
  static ros::NodeHandle nh;  // Static ensures this is initialized only once
  static ros::Publisher pub = nh.advertise<std_msgs::String>("topic2", 10);

  std_msgs::String new_msg;
  std::cout << "Enter a message to publish to topic2: ";
  std::getline(std::cin, new_msg.data);

  pub.publish(new_msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("topic1", 10, subscriberCallback);

  ros::spin();

  return 0;
}