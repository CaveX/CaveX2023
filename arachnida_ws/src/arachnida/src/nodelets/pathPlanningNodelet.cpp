#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>

// Messages
#include "arachnida/ObstacleList.h"
#include <geometry_msgs/Vector3.h>

namespace arachnida {
	class PathPlanningNodelet : public nodelet::Nodelet {
		public:
			PathPlanningNodelet() {}
		private:
			virtual void onInit() {
				ROS_INFO("[pathPlanningNodelet.cpp] Initializing Path Planning nodelet...");
				ros::NodeHandle &nh = getNodeHandle();
				objectsSub = nh.subscribe("arachnida/object_detection/objects_detected", 100, &PathPlanningNodelet::obstaclesCallback, this);
				instantaneousPathPub = nh.advertise<geometry_msgs::Vector3>("arachnida/path_planning/instantaneous_path_vector", 100);
				ROS_INFO("[pathPlanningNodelet.cpp] Initialized Path Planning nodelet...");
			};

			void obstaclesCallback(const arachnida::ObstacleListConstPtr& obsList_msg) {

				// instantaneousPathPub.publish(obsMsg);
				ROS_INFO("[pathPlanningNodelet.cpp] Received obstacle list message");

			};

			std::chrono::time_point<std::chrono::high_resolution_clock> lastObjectDetectionTimestamp = std::chrono::high_resolution_clock::now(); // Stores the last time path planning was run - Used for throttling
			ros::Subscriber objectsSub;
			ros::Publisher instantaneousPathPub;

	};

	PLUGINLIB_EXPORT_CLASS(arachnida::PathPlanningNodelet, nodelet::Nodelet);
};

