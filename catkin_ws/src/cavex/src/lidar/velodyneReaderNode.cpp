#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include "velodynePCAPReader.h"

velodynePCAPReader reader("/wsl.localhost/Ubuntu-18.04/home/succinyl/PCAP Files/VLP-16");

int main(int argc, char **argv) {
    ros::init(argc, argv, "velodyneReaderNode");
    ros::NodeHandle nh;

    ros::Publisher veloPublisher = nh.advertise<std_msgs::String>("/velodyneReader", 1000);
    ros::Rate loop_rate(10);
    int count = 0;

    

    while(ros::ok()) {
        reader.readFile();
        
       // veloPublisher.publish(rosMsg);

        ros::spinOnce();
        loop_rate.sleep();

        ++count;
    }

    return 0;
}