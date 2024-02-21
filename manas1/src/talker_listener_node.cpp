
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

void subscriberCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Received: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher_node");
  ros::NodeHandle nh;

  // Publisher for topic1
  ros::Publisher pub_topic1 = nh.advertise<std_msgs::String>("topic1", 10);

  // Subscriber for topic2
  ros::Subscriber sub_topic2 = nh.subscribe("topic2", 10, subscriberCallback);

  ros::Rate loop_rate(1); // Publish at 1Hz

  while (ros::ok())
  {
    std_msgs::String msg;
    std::cout << "Enter a message to publish to topic1: ";
    std::getline(std::cin, msg.data);
    pub_topic1.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
