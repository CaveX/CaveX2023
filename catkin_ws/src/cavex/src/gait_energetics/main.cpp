#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "gait_energetics.h"
#include <ostream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

// create instance of object used for monitoring power consumptions and calculating averages
GaitEnergetics* gaitEnergeticsTool = new GaitEnergetics();

void jointStatesCallback(const sensor_msgs::JointState msg){
    gaitEnergeticsTool->readJointEfforts(msg);
}

int main(int argc, char **argv){
    gaitTypes gaits;
    // initalise ros publisher and subscriber node
    ros::init(argc, argv, "GaitEnergeticsNode");
    ros::NodeHandle gaitEnergeticsNode;

    // intialise ros publisher and subscriber
    ros::Publisher gait_selection = gaitEnergeticsNode.advertise<std_msgs::Int8>("syropod_remote/gait_selection", 1);
    ros::Subscriber joint_efforts = gaitEnergeticsNode.subscribe("joint_states",1000,jointStatesCallback);
    
    // intialise the system by ensuring the default gait is tripod (SHC does this too)
    gaitEnergeticsTool->gaitType.data = gaits.tripod;
    gait_selection.publish(gaitEnergeticsTool->gaitType);

    ros::Rate rate(10);

    while (ros::ok()){
        ros::spinOnce();

        if (gaitEnergeticsTool->newGait){
            gait_selection.publish(gaitEnergeticsTool->getGaitType());
            gaitEnergeticsTool->newGait = false;
        }
        rate.sleep();
    }
    delete gaitEnergeticsTool;
    return 0;
}