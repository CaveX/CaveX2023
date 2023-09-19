#include "ros/ros.h"
#include "nodelet/loader.h"
#include <string>

int main(int argc, char** argv) {
    ros::init(argc, argv, "cavexNodeletManager");
    nodelet::Loader nodeletLoader;
    nodelet::M_string remappings(ros::names::getRemappings());
    nodelet::V_string nargv;
    
    // F-LOAM Nodelet
    std::string nodeletName = "cavex/floamNodelet";
    std::string nodeletType = "cavex::floamNodelet";
    // std::string nodeletType = "nodelets/floamNodelet";
    nodeletLoader.load(nodeletName, nodeletType, remappings, nargv);

    
    ros::spin();
}