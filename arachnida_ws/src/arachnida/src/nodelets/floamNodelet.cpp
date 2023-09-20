#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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

