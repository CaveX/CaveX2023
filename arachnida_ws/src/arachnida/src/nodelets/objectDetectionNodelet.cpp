#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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

