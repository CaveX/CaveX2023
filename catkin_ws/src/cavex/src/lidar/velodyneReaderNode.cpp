#include <iostream>
#include <chrono>
#include <queue>
#include <fstream>

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

velodynePCAPReader reader("/cavex_workspace/dev/CaveX2023/Sample Velodyne Data/MyRoom1.pcap");
// velodynePCAPReader reader("/mnt/c/Users/lukap/OneDrive/Desktop/Study/Fifth Year/Honours/Sample Velodyne Data/2014-11-10-11-32-17_Velodyne-VLP_10Hz_Monterey Highway_SPLIT1.pcap");
// velodynePCAPReader reader("/mnt/c/Users/lukap/OneDrive/Desktop/Study/Fifth Year/Honours/Sample Velodyne Data/RoboticsLab.pcap");
velodyneSocketReader sockRead;

LaserMappingClass laserMapping;
LaserProcessingClass laserProcessing;

odomEstimationClass odomEstimation;

std::vector<char> packetBuffer; // stores the raw binary data from the lidar

objPointCloudProcessor objProcessor;

int main(int argc, char **argv) {
    ros::init(argc, argv, "velodyneReaderNode");
    ros::NodeHandle nh;
    std::cout << "Instantiating veloPublisher\n"; 
    ros::Publisher veloPublisher = nh.advertise<sensor_msgs::PointCloud2>("/velodyneReader", 100);
    ros::Rate loop_rate(10);
    int count = 0;

    int totalFramesProcessed = 0;
    int totalTimeElapsed = 0;
    bool isOdomInitialised = false;

    sockRead.connect(packetBuffer);
    
    std::ofstream movementFile("movementData.txt"); // this was used for debugging - remove later if not needed
    
    while(ros::ok()) {
        reader.readFile(); // return a vector of frames
        for(int i = 1; i < reader.getFrameClouds().size(); i++) { // loop through the frames (start at index 1 because we need to compare the current frame to the previous frame)
            pcl::PointCloud<pcl::PointXYZI>::Ptr frame = reader.getFrameClouds()[i];
            if(frame->size() < 3000) continue;
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudEdge(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the edge points from the frame
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudSurf(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the surface points from the frame

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            laserProcessing.featureExtraction(frame, pointCloudEdge, pointCloudSurf); // extract the edge and surface points from the frame
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsedSeconds = end - start;
            std::cout << "featureExtraction dur (s): " << elapsedSeconds.count() << "\n";
            

            totalFramesProcessed++;

            float timeTemp = elapsedSeconds.count() * 1000;
            totalTimeElapsed += timeTemp;

            // TODO: may need to create a pointCloudFiltered PointCloud to store the surface + edge points (laserMappingNode listens to it for some reason)
            std::cout << "pointCloudEdge Size:" << pointCloudEdge->size() << "points \n";
            std::cout << "pointCloudSurf Size:" << pointCloudSurf->size() << "points \n";

            
            if(pointCloudEdge->size() > 0 && pointCloudSurf->size() > 0) {
                if(isOdomInitialised) {
                    std::chrono::time_point<std::chrono::system_clock> start, end;
                    start = std::chrono::system_clock::now();
                    odomEstimation.updatePointsToMap(pointCloudEdge, pointCloudSurf);
                    end = std::chrono::system_clock::now();
                    std::chrono::duration<float> elapsedSeconds = end - start;
                    float timeTempOdom = elapsedSeconds.count() * 1000;
                    totalTimeElapsed += timeTemp;
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

                // std::cout << "Transform --------- Frame " << i << "\n";
                // std::cout << "Translation: " << tCurrent.x() << ", " << tCurrent.y() << ", " << tCurrent.z() << "\n";
                // std::cout << "Rotation: " << qCurrent.x() << ", " << qCurrent.y() << ", " << qCurrent.z() << ", " << qCurrent.w() << "\n";

                movementFile << "Transform --------- Frame " << i << "\n";
                movementFile << "Translation: " << tCurrent.x() << ", " << tCurrent.y() << ", " << tCurrent.z() << "\n";
                movementFile << "Rotation: " << qCurrent.x() << ", " << qCurrent.y() << ", " << qCurrent.z() << ", " << qCurrent.w() << "\n\n\n";
                
            }
            // if(i > 20) break;
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
