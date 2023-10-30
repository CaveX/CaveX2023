#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>
#include <vector>

// ROS message includes
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// Eigen includes
#include <Eigen/Dense>

namespace arachnida {
	class RobotStateNodelet : public nodelet::Nodelet {
		public:
			RobotStateNodelet() {}
		private:
			virtual void onInit() {
				ROS_INFO("[robotStateNodelet.cpp] Initializing Robot State nodelet...");
                ros::NodeHandle &nh = getNodeHandle();
                floamSub = nh.subscribe("arachnida/floam_odom", 100, &RobotStateNodelet::floamCallback, this);
                curPosePub = nh.advertise<nav_msgs::Odometry>("arachnida/robot_state/current_pose", 100);
                cumPosePub = nh.advertise<nav_msgs::Odometry>("arachnida/robot_state/cumulative_pose", 100);
				ROS_INFO("[robotStateNodelet.cpp] Initialized Robot State nodelet...");
			};

            void floamCallback(const nav_msgs::Odometry floam_msg) {
                RobotState newState;

                if(stateHistory.size() > 0) {
                    // Calculate new cumulative location
                    double newXLoc = stateHistory.back().location.x() + floam_msg.pose.pose.position.x;
                    double newYLoc = stateHistory.back().location.y() + floam_msg.pose.pose.position.y;
                    double newZLoc = stateHistory.back().location.z() + floam_msg.pose.pose.position.z;
                    Eigen::Vector3d newLoc( newXLoc, newYLoc, newZLoc );
                    newState.location = newLoc;

                    // Calculate new cumulative orientation
                    double newWQuat = stateHistory.back().cumOrientation.w() + floam_msg.pose.pose.orientation.w;
                    double newXQuat = stateHistory.back().cumOrientation.x() + floam_msg.pose.pose.orientation.x;
                    double newYQuat = stateHistory.back().cumOrientation.y() + floam_msg.pose.pose.orientation.y;
                    double newZQuat = stateHistory.back().cumOrientation.z() + floam_msg.pose.pose.orientation.z;
                    Eigen::Quaterniond newOrientation(newWQuat, newXQuat, newYQuat, newZQuat);
                    newState.cumOrientation = newOrientation;

                    // Set instantaenous translation and orientation
                    double newXInstTrans = floam_msg.pose.pose.position.x;
                    double newYInstTrans = floam_msg.pose.pose.position.y;
                    double newZInstTrans = floam_msg.pose.pose.position.z;
                    Eigen::Vector3d newInstTrans(newXInstTrans, newYInstTrans, newZInstTrans);
                    newState.instantaneousTranslation = newInstTrans;

                    double newWInstOrient = floam_msg.pose.pose.orientation.w;
                    double newXInstOrient = floam_msg.pose.pose.orientation.x;
                    double newYInstOrient = floam_msg.pose.pose.orientation.y;
                    double newZInstOrient = floam_msg.pose.pose.orientation.z;
                    Eigen::Quaterniond newInstOrient(newWInstOrient, newXInstOrient, newYInstOrient, newZInstOrient);
                    newState.instantaneousOrientation = newInstOrient;
                } else {
                    // Calculate new cumulative location
                    double newXLoc = floam_msg.pose.pose.position.x;
                    double newYLoc = floam_msg.pose.pose.position.y;
                    double newZLoc = floam_msg.pose.pose.position.z;
                    Eigen::Vector3d newLoc( newXLoc, newYLoc, newZLoc );
                    newState.location = newLoc;

                    // Calculate new cumulative orientation
                    double newWQuat = floam_msg.pose.pose.orientation.w;
                    double newXQuat = floam_msg.pose.pose.orientation.x;
                    double newYQuat = floam_msg.pose.pose.orientation.y;
                    double newZQuat = floam_msg.pose.pose.orientation.z;
                    Eigen::Quaterniond newOrientation(newWQuat, newXQuat, newYQuat, newZQuat);
                    newState.cumOrientation = newOrientation;

                    // Set instantaenous translation and orientation
                    double newXInstTrans = floam_msg.pose.pose.position.x;
                    double newYInstTrans = floam_msg.pose.pose.position.y;
                    double newZInstTrans = floam_msg.pose.pose.position.z;
                    Eigen::Vector3d newInstTrans(newXInstTrans, newYInstTrans, newZInstTrans);
                    newState.instantaneousTranslation = newInstTrans;

                    double newWInstOrient = floam_msg.pose.pose.orientation.w;
                    double newXInstOrient = floam_msg.pose.pose.orientation.x;
                    double newYInstOrient = floam_msg.pose.pose.orientation.y;
                    double newZInstOrient = floam_msg.pose.pose.orientation.z;
                    Eigen::Quaterniond newInstOrient(newWInstOrient, newXInstOrient, newYInstOrient, newZInstOrient);
                    newState.instantaneousOrientation = newInstOrient;

                }


                // Append new robot state to state history
                stateHistory.push_back(newState);

                // Populate and publish messages
                nav_msgs::Odometry curPose;
                nav_msgs::Odometry cumPose;

                curPose.header.seq = floam_msg.header.seq;
                curPose.pose.pose.position.x = newState.instantaneousTranslation.x();
                curPose.pose.pose.position.y = newState.instantaneousTranslation.y();
                curPose.pose.pose.position.z = newState.instantaneousTranslation.z();
                curPose.pose.pose.orientation.w = newState.instantaneousOrientation.w();
                curPose.pose.pose.orientation.x = newState.instantaneousOrientation.x();
                curPose.pose.pose.orientation.y = newState.instantaneousOrientation.y();
                curPose.pose.pose.orientation.z = newState.instantaneousOrientation.z();

                cumPose.header.seq = floam_msg.header.seq;
                cumPose.pose.pose.position.x = newState.location.x();
                cumPose.pose.pose.position.y = newState.location.y();
                cumPose.pose.pose.position.z = newState.location.z();
                cumPose.pose.pose.orientation.w = newState.cumOrientation.w();
                cumPose.pose.pose.orientation.x = newState.cumOrientation.x();
                cumPose.pose.pose.orientation.y = newState.cumOrientation.y();
                cumPose.pose.pose.orientation.z = newState.cumOrientation.z();

                curPosePub.publish(curPose);
                cumPosePub.publish(cumPose);
				ROS_INFO("[robotStateNodelet.cpp] Received F-LOAM Odom Message");
            };

            typedef struct {
                Eigen::Vector3d location; // Current location relative to starting location
                Eigen::Vector3d instantaneousTranslation; // Translation from previous measurement to current measurement
                Eigen::Quaterniond instantaneousOrientation; // Orientation from previous measurement to current measurement
                Eigen::Quaterniond cumOrientation; // Current (cumulative) orientation relative to starting orientation
            } RobotState;

            
            ros::Subscriber floamSub;
            ros::Publisher curPosePub; // Current (instantaneous) pose (position and orientation) publisher
            ros::Publisher cumPosePub; // Cumulative pose (position and orientation) publisher
            std::vector<RobotState> stateHistory;
	};

	PLUGINLIB_EXPORT_CLASS(arachnida::RobotStateNodelet, nodelet::Nodelet);
};
