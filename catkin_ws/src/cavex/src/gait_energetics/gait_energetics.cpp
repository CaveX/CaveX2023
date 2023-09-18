#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <ostream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

struct gaitTypes{
    int wave = 0;
    int amble = 1;
    int ripple = 2;
    int tripod = 3;
};

std_msgs::Int8 gaitType; // used to set gait Type under gait_selection topic
std::vector<float> powerConsumptionData; // used to store instantaneous motor power consumption
float longTime = 3; // time for long term power average in (s)
float shortTime = 0.5; // time for short term power average in (s)
float elapsedTime = 0; // time since last gait switch initalised as 0s
float tripodPowerTolerance = 9; // Maximum different in long-term and instantanoues power averages (W)
float wavePowerTolerance = 5; // Maximum different in long-term and instantanoues power averages (W)


void readJointEffort(const sensor_msgs::JointState msg){
    // log messages we are subscribing to
    // each packet has data for 18 motors
    std::cout << msg.header;
    std::cout << "Total Motor Power Consumption:" << std::endl;
    float currentConsumption = 0;
    float powerConsumption = 0;
    for (int i = 0; i < msg.effort.size(); i++){
        //std::cout << msg.effort.at(i) << ";"; // print individual joint effort
        currentConsumption += fabs(msg.effort.at(i));
    }
    powerConsumption = currentConsumption * 12; // multiply by operating voltage
    std::cout << powerConsumption << std::endl;
    // add powerConsumption to vector
    powerConsumptionData.push_back(powerConsumption);
    //setGait();
    // ROS_INFO("The message that we received was: %s", msg.effort);
}

// void setGait(){
//     // determine instantaneous power (for shorTime seconds)
//     // workout avgPower determined for longTime seconds
//      // determine elapsed time using timestamp
//     if (elapsedTime > longTime){
//         // used to allow for the power consumption data to stabilise after a gait switch
//         if (gaitType.data == 1){
//             // currently tripod gait
//             if (fabs(avgPower - power) > tripodPowerTolerance){
//                 // set gait type to  wave
//                 elapsedTime = 0;
//                 gaitType.data = 0;
//             }
//         }
//         else if (gaitType.data == 0){
//             // currently in wave gait
//             if (fabs(avgPower - power) > wavePowerTolerance){
//                 // set gait type to wave
//                 elapsedTime = 0;
//                 gaitType.data = 1;
//             }
//         }
//     }
// }

int main(int argc, char **argv){
    gaitTypes gaits;
    // initalise ros publisher and subscriber node
    ros::init(argc, argv, "GaitEnergeticsNode");
    ros::NodeHandle gaitEnergeticsNode;

    // intialise ros publisher and subscriber
    ros::Publisher gait_selection = gaitEnergeticsNode.advertise<std_msgs::Int8>("syropod_remote/gait_selection", 1);
    ros::Subscriber joint_efforts = gaitEnergeticsNode.subscribe("joint_states",1000,readJointEffort);
    
    // intialise the system by ensuring the default gait is tripod
    gaitType.data = gaits.tripod;
    gait_selection.publish(gaitType);

    while (ros::ok()){
        ros::spinOnce();
    }
    
    return 0;
}