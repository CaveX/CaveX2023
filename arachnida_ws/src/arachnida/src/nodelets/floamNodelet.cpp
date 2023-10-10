#include "pcl/conversions.h"
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

// F-LOAM includes
#include "floam_cpu/laserMappingClass.h"
#include "floam_cpu/laserProcessingClass.h"
#include "floam_cpu/lidarOptimisation.h"
#include "floam_cpu/odomEstimationClass.h"

namespace arachnida {
	class FLOAMNodelet : public nodelet::Nodelet {
		public:
			FLOAMNodelet() {}
		private:
			virtual void onInit() {
				ROS_INFO("[floamNodelet.cpp] Initializing F-LOAM nodelet...");
				ros::NodeHandle &nh = getNodeHandle();
				floamOdomPub = nh.advertise<nav_msgs::Odometry>("arachnida/floam_odom", 100);
				pcSub = nh.subscribe("arachnida/point_cloud/pcl", 100, &FLOAMNodelet::cloudCallback, this);
				ROS_INFO("[floamNodelet.cpp] Initialized F-LOAM nodelet...");
			};

			void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
				nav_msgs::Odometry odom;
				odom.header.seq = cloud_msg->header.seq;
                
				pcl::PointCloud<pcl::PointXYZI>::Ptr pcFrame(new pcl::PointCloud<pcl::PointXYZI>());
				pcl::fromROSMsg(*cloud_msg, *pcFrame);

				pcl::PointCloud<pcl::PointXYZI>::Ptr pcEdges(new pcl::PointCloud<pcl::PointXYZI>());
				pcl::PointCloud<pcl::PointXYZI>::Ptr pcSurfaces(new pcl::PointCloud<pcl::PointXYZI>());

				laserProcessing.featureExtraction(pcFrame, pcEdges, pcSurfaces);

				if(pcEdges->size() > 0 && pcSurfaces->size() > 0) {
                    if(isOdomInitialised) {
                        odomEstimation.updatePointsToMap(pcEdges, pcSurfaces);
                    } else {
                        odomEstimation.init(0.8);
                        odomEstimation.initMapWithPoints(pcEdges, pcSurfaces);
                        isOdomInitialised = true;
                    }
					Eigen::Quaterniond qCurrent(odomEstimation.odom.rotation());
					Eigen::Vector3d tCurrent(odomEstimation.odom.translation());

					tf::Transform transform;
					transform.setOrigin(tf::Vector3(tCurrent.x(), tCurrent.y(), tCurrent.z()));
					tf::Quaternion q(qCurrent.x(), qCurrent.y(), qCurrent.z(), qCurrent.w());
					transform.setRotation(q);

					odom.pose.pose.position.x = tCurrent.x();
					odom.pose.pose.position.y = tCurrent.y();
					odom.pose.pose.position.z = tCurrent.z();

					odom.pose.pose.orientation.w = qCurrent.w();
					odom.pose.pose.orientation.x = qCurrent.x();
					odom.pose.pose.orientation.y = qCurrent.y();
					odom.pose.pose.orientation.z = qCurrent.z();
				}

				floamOdomPub.publish(odom);
				ROS_INFO("[floamNodelet.cpp] Received point cloud message");

			};

			ros::Subscriber pcSub;
			ros::Publisher floamOdomPub;

			LaserMappingClass laserMapping;
			LaserProcessingClass laserProcessing;
			odomEstimationClass odomEstimation;
			bool isOdomInitialised = false;
	};

	PLUGINLIB_EXPORT_CLASS(arachnida::FLOAMNodelet, nodelet::Nodelet);
};

