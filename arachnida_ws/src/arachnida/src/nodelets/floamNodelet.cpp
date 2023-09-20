#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>

namespace arachnida {
	class FLOAMNodelet : public nodelet::Nodelet {
		public:
			FLOAMNodelet() {}
		private:
			virtual void onInit() {
				ROS_INFO("[floamNodelet.cpp] Initializing F-LOAM nodelet...");
			};
	};

	PLUGINLIB_EXPORT_CLASS(arachnida::FLOAMNodelet, nodelet::Nodelet);
};

