#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "gait_energetics.h"
#include <ostream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <string>
#include <unistd.h>

// create instance of object used for monitoring power consumptions and calculating averages
GaitEnergetics* gaitEnergeticsTool = new GaitEnergetics();
std::string controlMethod;
geometry_msgs::Twist previous_velocity; // store previous command to maintain traversal after switch

void jointStatesCallback(const sensor_msgs::JointState msg){
    if (controlMethod != "joy"){
        gaitEnergeticsTool->readJointEfforts(msg);
    }
}

void updateVelocity(const geometry_msgs::Twist msg){
	previous_velocity = msg;
}

int main(int argc, char **argv){
    gaitTypes gaits;
    // initalise ros publisher and subscriber node
    ros::init(argc, argv, "GaitEnergeticsNode");
    ros::NodeHandle gaitEnergeticsNode;
    geometry_msgs::Twist desired_velocity_msg_;
    desired_velocity_msg_.linear.x = 0.5;
    desired_velocity_msg_.linear.y = 0;
    desired_velocity_msg_.linear.z = 0;
    desired_velocity_msg_.angular.x = 0;
    desired_velocity_msg_.angular.y = 0;
    desired_velocity_msg_.angular.z = 0;

    // get current control method parameter
    gaitEnergeticsNode.getParam("/syropod_remote/control_method",controlMethod);

    // intialise ros publisher and subscriber
    ros::Publisher gait_selection = gaitEnergeticsNode.advertise<std_msgs::Int8>("syropod_remote/gait_selection", 1);
    ros::Publisher velocity_command = gaitEnergeticsNode.advertise<geometry_msgs::Twist>("syropod_remote/desired_velocity", 1);
    ros::Subscriber joint_efforts = gaitEnergeticsNode.subscribe("joint_states",1000,jointStatesCallback);
    ros::Subscriber previous_velocity_command = gaitEnergeticsNode.subscribe("syropod_remote/desired_velocity",100,updateVelocity);
    
    // intialise the system by ensuring the default gait is tripod (SHC does this too)
    gaitEnergeticsTool->gaitType.data = gaits.tripod;
    gait_selection.publish(gaitEnergeticsTool->gaitType);

    ros::Rate rate(10);

    while (ros::ok()){
        ros::spinOnce();
        //std::string controlMethod;
        // get current control method parameter (may change here)
        gaitEnergeticsNode.getParam("/syropod_remote/control_method",controlMethod);
	//std::cout << controlMethod << "\n";

        if (gaitEnergeticsTool->newGait && controlMethod != "joy"){
		std::cout << "Here!" << std::endl;
            gait_selection.publish(gaitEnergeticsTool->getGaitType());
            gaitEnergeticsTool->newGait = false;
	    if (gaitEnergeticsTool->getGaitType().data == gaits.wave){
		sleep(8); // wave takes longer to switch sleep until switch is done
	    }else if (gaitEnergeticsTool->getGaitType().data == gaits.tripod){
		sleep(5); // tripod is quicker to switch
	    }
	    velocity_command.publish(previous_velocity);
        }
        rate.sleep();
    }
    delete gaitEnergeticsTool;
    return 0;
}
