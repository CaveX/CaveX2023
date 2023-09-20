#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

// Arachnida includes
#include "object_detection_cpu/objBox.h"

namespace arachnida {
	class VisualisationNodelet : public nodelet::Nodelet {
		public:
			VisualisationNodelet() {}
		private:
            ros::Subscriber pclSub; // Subscribes to the point cloud data
            ros::Subscriber slamSub; // Subscribes to the SLAM (F-LOAM) data -> transform, odometry, etc.
            ros::Subscriber objectDetectSub; // Subscribes to the object detection data (i.e vertices of the detected objects' bounding boxes)
            pcl::visualization::PCLVisualizer::Ptr viewer; // The PCL visualizer object
            int cloudCount = 0; // The number of point clouds (frames) received

			virtual void onInit() {
				ros::NodeHandle &private_nh = getPrivateNodeHandle();

                // viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Arachnida Point Cloud Viewer"));
                // viewer->setBackgroundColor(0, 0, 0);
                // viewer->addCoordinateSystem(1.0);
                // viewer->initCameraParameters();
                // viewer->setCameraPosition(0, 16, 0, 0, 0, 1);

				// pclSub = private_nh.subscribe("arachnida/point_cloud/pcl", 10, &VisualisationNodelet::pclCallback, this);
				ROS_INFO("[visualisationNodelet.cpp] Initializing Visualisation nodelet...");
			};

            void pclCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &msg) {
                ROS_INFO("[visualisationNodelet.cpp] Received point cloud data");
                cloudCount++;
            };

            void floamCallback(const std::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZI>::ConstPtr>> &msg) {
                ROS_INFO("[visualisationNodelet.cpp] Received F-LOAM data");
            };
            
            void objectDetectCallback(const std::shared_ptr<std::vector<Box>> &msg) {
                ROS_INFO("[visualisationNodelet.cpp] Received object detection data");
            };
	};

	PLUGINLIB_EXPORT_CLASS(arachnida::VisualisationNodelet, nodelet::Nodelet);
};