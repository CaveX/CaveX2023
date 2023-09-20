#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>

namespace cavexws {
	class FLOAMNodelet : public nodelet::Nodelet {
		public:
			FLOAMNodelet() {}
		private:
			virtual void onInit() {
				ROS_INFO("Initializing floam nodelet...");
			};
	};

	PLUGINLIB_EXPORT_CLASS(cavexws::FLOAMNodelet, nodelet::Nodelet);
};

