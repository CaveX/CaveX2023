#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>

namespace arachnida {
	class ObjectDetectionNodelet : public nodelet::Nodelet {
		public:
			ObjectDetectionNodelet() {}
		private:
			virtual void onInit() {
				ROS_INFO("[objectDetectionNodelet.cpp] Initializing Object Detection nodelet...");
			};
	};

	PLUGINLIB_EXPORT_CLASS(arachnida::ObjectDetectionNodelet, nodelet::Nodelet);
};

