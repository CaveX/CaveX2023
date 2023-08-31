#include "ros/ros.h"
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "cavex/utils.h"
#include "cavex/robot_state/extendedKalmanFilter.h"

cavex::Vector3D currentPosition;
cavex::Quaternion currentOrientation;
cavex::Vector3D currentLinearVelocity;
cavex::Vector3D currentAngularVelocity;
cavex::Vector3D currentLinearAcceleration;
cavex::Vector3D currentAngularAcceleration;

void imuCallback(const geometry_msgs::Vector3Stamped::ConstPtr& imuMsg) {
    ROS_INFO("I heard: [%f, %f, %f]", imuMsg->vector.x, imuMsg->vector.y, imuMsg->vector.z);
}

void slamCallback(const geometry_msgs::PoseStamped::ConstPtr& slamMsg) {
    ROS_INFO("I heard: [%f, %f, %f]", slamMsg->pose.position.x, slamMsg->pose.position.y, slamMsg->pose.position.z);
}

int main(int argc, char **argv) {
    // ROS setup
    ros::init(argc, argv, "robotStateNode");
    ros::NodeHandle nh;

    ros::Publisher robotPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/robotPose", 100); // Publishes currentPosition and currentOrientation
    ros::Publisher robotAccelPublisher = nh.advertise<geometry_msgs::AccelStamped>("/robotAccel", 100); // Publishes currentLinearAcceleration and currentAngularAcceleration
    ros::Publisher robotVelocityPublisher = nh.advertise<geometry_msgs::Vector3Stamped>("/robotVelocity", 100); // Publishes currentLinearVelocity
    ros::Publisher robotAngularVelocityPublisher = nh.advertise<geometry_msgs::Vector3Stamped>("/robotAngularVelocity", 100); // Publishes currentAngularVelocity

    ros::Subscriber imuSub = nh.subscribe("/data", 1000, imuCallback);
    ros::Subscriber slamSub = nh.subscribe("/cavex/slam/odom", 1000, slamCallback);

    ros::Rate loop_rate(100);
    int count = 0;

    // Extended Kalman Filter setup
    // cavex::ExtendedKalmanFilter positionFilter;
    // cavex::ExtendedKalmanFilter orientationFilter;
    // cavex::ExtendedKalmanFilter accelerationFilter;

    while(ros::ok()) {
        std::cout << "[robotStateNode] Running [" << count << "]\n";

        geometry_msgs::PoseStamped poseMsg;
        geometry_msgs::AccelStamped accelMsg;
        geometry_msgs::Vector3Stamped velocityMsg;

        robotPosePublisher.publish(poseMsg);
        robotAccelPublisher.publish(accelMsg);
        robotVelocityPublisher.publish(velocityMsg);

        ros::spinOnce();
        loop_rate.sleep();

        ++count;
    }
    return 0;
}