#include "ros/ros.h"
#include "std_msgs/String.h"

void callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Node 1 heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker_listener_node", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 10);
    ros::Subscriber sub = n.subscribe("chatter_2", 10, callback);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "hello world from Node 1";
        ROS_INFO("Node 1 says: [%s]", msg.data.c_str());
        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}