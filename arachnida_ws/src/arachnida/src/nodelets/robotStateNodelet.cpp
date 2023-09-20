#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>

namespace arachnida {
	class RobotStateNodelet : public nodelet::Nodelet {
		public:
			RobotStateNodelet() {}
		private:
			virtual void onInit() {
				ROS_INFO("[robotStateNodelet.cpp] Initializing Robot State nodelet...");
			};
	};

	PLUGINLIB_EXPORT_CLASS(arachnida::RobotStateNodelet, nodelet::Nodelet);
};

