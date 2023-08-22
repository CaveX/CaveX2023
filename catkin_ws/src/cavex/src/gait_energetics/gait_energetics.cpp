#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <ostream>

void writeMsgToLog(const sensor_msgs::JointState msg){
    // log messages we are subscribing to
    for (int i = 0; i < msg.effort.size(); i++){
        std::cout << msg.effort.at(i) << std::endl;
        std::cout << "Trying to read" << std::endl;
    }
    // ROS_INFO("The message that we received was: %s", msg.effort);
}

int main(int argc, char **argv){
    // initalise ros publisher and subscriber node
    ros::init(argc, argv, "Subscriber");
    // ros::init(argc, argv, "Publisher");
    ros::NodeHandle nh_sub;
    // ros::NodeHandle nh_pub;

    // intialise ros publisher
    // ros::Publisher gait_selection = nh_sub.advertise<std_msgs::Int8>("/syropod_remote/gait_selection",1000);
    // ros::Publisher gait_selection = 
    ros::Subscriber joint_efforts = nh_sub.subscribe("/joint_states",1000,writeMsgToLog);
    ros::spin(); // loops and waits for incoming messages

    return 0;
}