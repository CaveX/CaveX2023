#include <iostream>
#include <chrono>
#include <queue>
#include <fstream>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "velodynePCAPReader.h"
#include "floam_cpu/laserMappingClass.h"
#include "floam_cpu/laserProcessingClass.h"
#include "floam_cpu/lidarOptimisation.h"
#include "floam_cpu/odomEstimationClass.h"
#include "velodyneSocketReader.h"
#include "object_detection_cpu/objPointCloudProcessor.h"
#include "object_detection_cpu/objKdtree.h"
#include "object_detection_cpu/objCluster.h"
#include "object_detection_cpu/objRansac.h"
#include "arachnida/path_planning/ppArtificialPotentialField.h"

// velodynePCAPReader reader("/cavex_workspace/dev/CaveX2023/Sample Velodyne Data/MyRoom1.pcap");
velodynePCAPReader reader("/mnt/c/Users/lukap/OneDrive/Desktop/Study/Fifth Year/Honours/Sample Velodyne Data/2014-11-10-11-32-17_Velodyne-VLP_10Hz_Monterey Highway_SPLIT1.pcap");
// velodynePCAPReader reader("/mnt/c/Users/lukap/OneDrive/Desktop/Study/Fifth Year/Honours/Sample Velodyne Data/RoboticsLab.pcap");
velodyneSocketReader sockRead;

LaserMappingClass laserMapping;
LaserProcessingClass laserProcessing;

odomEstimationClass odomEstimation;

// std::array<char, 94037> frameBuffer; // stores the raw binary data from the lidar
// std::array<std::array<char, 94037>, 4701840> frameBufferQueue; // stores the raw binary data from the lidar as frames in a queue

std::vector<char> frameBuffer;
std::vector<std::vector<char>> frameBufferQueue;

objPointCloudProcessor objProcessor;

int main(int argc, char **argv) {
    // std::cout << __cplusplus << "\n";
    ros::init(argc, argv, "velodyneReaderNode");
    ros::NodeHandle nh;
    std::cout << "Instantiating veloPublisher\n"; 
    // ros::Publisher veloPublisher = nh.advertise<sensor_msgs::PointCloud2>("/velodyneReader", 100);
    // ros::Publisher pcPublisher = nh.advertise<sensor_msgs::PointCloud2ConstPtr>("arachnida/point_cloud/pcl", 100);
    ros::Publisher pcPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("arachnida/point_cloud/pcl", 100);
    // ros::Publisher pcPublisher = nh.advertise<sensor_msgs::PointCloud2>("arachnida/point_cloud/pcl", 100);
    
    ros::Rate loop_rate(10);
    int count = 0;

    int totalFramesProcessed = 0;
    int totalTimeElapsed = 0;
    bool isOdomInitialised = false;

    sockRead.connect(frameBuffer, frameBufferQueue, pcPublisher);
    
    std::ofstream movementFile("movementData.txt"); // this was used for debugging - remove later if not needed
    
    while(ros::ok()) {
        reader.readFile(); // return a vector of frames
        for(int i = 1; i < reader.getFrameClouds().size(); i++) { // loop through the frames (start at index 1 because we need to compare the current frame to the previous frame)
            pcl::PointCloud<pcl::PointXYZI>::Ptr frame = reader.getFrameClouds()[i];
            if(frame->size() < 3000) continue;
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudEdge(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the edge points from the frame
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudSurf(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the surface points from the frame

            laserProcessing.featureExtraction(frame, pointCloudEdge, pointCloudSurf); // extract the edge and surface points from the frame

            totalFramesProcessed++;

            // TODO: may need to create a pointCloudFiltered PointCloud to store the surface + edge points (laserMappingNode listens to it for some reason)
            std::cout << "pointCloudEdge Size:" << pointCloudEdge->size() << "points \n";
            std::cout << "pointCloudSurf Size:" << pointCloudSurf->size() << "points \n";

            
            if(pointCloudEdge->size() > 0 && pointCloudSurf->size() > 0) {
                if(isOdomInitialised) {
                    auto t1 = std::chrono::high_resolution_clock::now();
                    odomEstimation.updatePointsToMap(pointCloudEdge, pointCloudSurf);
                    auto t2 = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
                    std::cout << "updatePointsToMap:" << duration << "us \n";
                } else {
                    odomEstimation.init(0.4);
                    odomEstimation.initMapWithPoints(pointCloudEdge, pointCloudSurf);
                    isOdomInitialised = true;
                    // ROS_INFO("Odom initialised");
                }

                Eigen::Quaterniond qCurrent(odomEstimation.odom.rotation());
                Eigen::Vector3d tCurrent = odomEstimation.odom.translation();

                // static tf::TransformBroadcaster br;
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(tCurrent.x(), tCurrent.y(), tCurrent.z()));
                tf::Quaternion q(qCurrent.x(), qCurrent.y(), qCurrent.z(), qCurrent.w());
                transform.setRotation(q);

                movementFile << "Transform --------- Frame " << i << "\n";
                movementFile << "Translation: " << tCurrent.x() << ", " << tCurrent.y() << ", " << tCurrent.z() << "\n";
                movementFile << "Rotation: " << qCurrent.x() << ", " << qCurrent.y() << ", " << qCurrent.z() << ", " << qCurrent.w() << "\n\n\n";
                
            }
            if(i > 40) break;
        }
        sensor_msgs::PointCloud2 pcFrameMsg;
       // veloPublisher.publish(rosMsg);
        break;
        ros::spinOnce();
        loop_rate.sleep();

        ++count;
    }

    movementFile.close();

    return 0;
}
