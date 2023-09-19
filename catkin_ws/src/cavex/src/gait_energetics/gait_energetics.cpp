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
float tripodPowerTolerance = 20; // Maximum different in long-term and instantanoues power averages (W)
float wavePowerTolerance = 15; // Maximum different in long-term and instantanoues power averages (W)
float msgFrequency = 100; // Hz. Frequency of motor data being published
ros::Time previousTimeStamp; // used to store previous time stamp of data
float elapsedTime = 0; // time since last gait switch initalised as 0s
bool initalisation = true; // used to determine if the time data needs initalisation
bool newGait = false; // used to tell publisher whether or not to publish new gait type

void checkPowerVector(){
    // A helper function to ensure only the last 100 samples are recorded
    if (powerConsumptionData.size() > 500){
        powerConsumptionData.erase(powerConsumptionData.begin()); // remove oldest element which is at the front of the vector
    }
}

float powerAverage(float duration){
    // Helper function to work out the power for the last duration seconds
    int numData = duration * msgFrequency; // cast as integer
    // get position of most recent power consumption
    int endPosition = powerConsumptionData.size() - 1;
    float powerSum = 0; // store sum of power data
    int counter = 0;
    for (int i = endPosition; i > endPosition - numData; i--){
        // start at endPosition and read required number of data
        powerSum += powerConsumptionData.at(i);
        counter++;
    }
    // return average power
    return powerSum / numData;
}

void setGait(){
    gaitTypes gaits; // initalise struct for gait data selection

    if (elapsedTime > longTime){
        // used to allow for the power consumption data to stabilise after a gait switch
        float power = powerAverage(shortTime);
        float avgPower = powerAverage(longTime);
        std::cout << power << std::endl;
        std::cout << avgPower << std::endl;

        if (gaitType.data == gaits.tripod){
            // currently tripod gait
            if (fabs(avgPower - power) > tripodPowerTolerance){
                // set gait type to  wave
                elapsedTime = 0;
                gaitType.data = gaits.wave;
                newGait = true;
            }
        }
        else if (gaitType.data == gaits.wave){
            // currently in wave gait
            if (fabs(avgPower - power) > wavePowerTolerance){
                // set gait type to tripod
                elapsedTime = 0;
                gaitType.data = gaits.tripod;
                newGait = true;
            }
        }
    }
}

void readJointEffort(const sensor_msgs::JointState msg){
    // log messages we are subscribing to
    // each packet has data for 18 motors
    ros::Time timeStamp = msg.header.stamp;
    if (initalisation){
        previousTimeStamp = timeStamp;
        initalisation = false;
    }

    float currentConsumption = 0;
    float powerConsumption = 0;

    for (int i = 0; i < msg.effort.size(); i++){
        //std::cout << msg.effort.at(i) << ";"; // print individual joint effort
        currentConsumption += fabs(msg.effort.at(i));
    }
    powerConsumption = currentConsumption * 12; // multiply by operating voltage
    // add powerConsumption to vector
    powerConsumptionData.push_back(powerConsumption);
    checkPowerVector(); // check length does not exceed 100
    
    // udpate elapsed time using time stamp data
    elapsedTime = timeStamp.toSec() - previousTimeStamp.toSec();
    std::cout << "ELAPSED TIME: " << std::endl;
    std::cout << elapsedTime << std::endl;
    std::cout << "Total Motor Power Consumption:" << std::endl;
    std::cout << powerConsumption << std::endl;
    setGait();
    // ROS_INFO("The message that we received was: %s", msg.effort);
}

int main(int argc, char **argv){
    gaitTypes gaits;
    // initalise ros publisher and subscriber node
    ros::init(argc, argv, "GaitEnergeticsNode");
    ros::NodeHandle gaitEnergeticsNode;

    // intialise ros publisher and subscriber
    ros::Publisher gait_selection = gaitEnergeticsNode.advertise<std_msgs::Int8>("syropod_remote/gait_selection", 1);
    ros::Subscriber joint_efforts = gaitEnergeticsNode.subscribe("joint_states",1000,readJointEffort);
    
    // intialise the system by ensuring the default gait is tripod (SHC does this too)
    gaitType.data = gaits.tripod;
    gait_selection.publish(gaitType);

    while (ros::ok()){
        ros::spinOnce();

        if (newGait){
            gait_selection.publish(gaitType);
            newGait = false;
        }
    }
    
    return 0;
}