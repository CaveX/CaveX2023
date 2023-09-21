#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS message includes
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

namespace arachnida {
	class FLOAMNodelet : public nodelet::Nodelet {
		public:
			FLOAMNodelet() {}
		private:
			virtual void onInit() {
				ROS_INFO("[floamNodelet.cpp] Initializing F-LOAM nodelet...");
				ros::NodeHandle &pnh = getPrivateNodeHandle();
				pcSub = pnh.subscribe("arachnida/point_cloud/pcl", 100, &FLOAMNodelet::cloudCallback, this);
				floamOdomPub = pnh.advertise<nav_msgs::Odometry>("arachnida/floam_odom", 100);
				ROS_INFO("[floamNodelet.cpp] Initialized F-LOAM nodelet...");
			};

			void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
				ROS_INFO("[floamNodelet.cpp] Received point cloud message");
			};

			ros::Subscriber pcSub;
			ros::Publisher floamOdomPub;
	};

	PLUGINLIB_EXPORT_CLASS(arachnida::FLOAMNodelet, nodelet::Nodelet);
};

