#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>

namespace cavex {
	class floamNodelet : public nodelet::Nodelet {
		public:
			floamNodelet() {}
			~floamNodelet() {}
		private:
			virtual void onInit() {
				ROS_INFO("Initializing floam nodelet...");
			};
	};

	PLUGINLIB_EXPORT_CLASS(cavex::floamNodelet, nodelet::Nodelet);
};

