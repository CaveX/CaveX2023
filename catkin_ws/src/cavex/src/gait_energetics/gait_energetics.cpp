#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <ostream>
#include <iostream>
#include <fstream>

std_msgs::Int8 gaitType;

void readJointEffort(const sensor_msgs::JointState msg){
    // log messages we are subscribing to
    // each packet has data for 18 motors
    std::cout << msg.header;
    std::cout << "Joint Efforts:" << std::endl;
    for (int i = 0; i < msg.effort.size(); i++){
        std::cout << msg.effort.at(i) << ";"; // print joint effort
        //dataFile << std::to_string(msg.effort.at(i));
        // if (msg.effort.at(i) > 0){
        //     gaitType.data = 1; // amble
        // }
        // else{
        //     gaitType.data = 70; // wave
        // }
    }
    std::cout << std::endl << std::endl;
    // ROS_INFO("The message that we received was: %s", msg.effort);
}

int main(int argc, char **argv){
    // initalise ros publisher and subscriber node
    ros::init(argc, argv, "GaitEnergeticsNode");
    ros::NodeHandle nh_sub;

    // intialise ros publisher and subscriber
    ros::Publisher gait_selection = nh_sub.advertise<std_msgs::Int8>("syropod_remote/gait_selection", 1);
    ros::Subscriber joint_efforts = nh_sub.subscribe("joint_states",10,readJointEffort);
    
    while (ros::ok()){
        //gait_selection.publish(gaitType);
        
        ros::spinOnce();
    }
    
    return 0;
}