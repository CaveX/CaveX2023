#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>

namespace arachnida {
	class PathPlanningNodelet : public nodelet::Nodelet {
		public:
			PathPlanningNodelet() {}
		private:
			virtual void onInit() {
				ROS_INFO("[pathPlanningNodelet.cpp] Initializing Path Planning nodelet...");
			};
	};

	PLUGINLIB_EXPORT_CLASS(arachnida::PathPlanningNodelet, nodelet::Nodelet);
};

