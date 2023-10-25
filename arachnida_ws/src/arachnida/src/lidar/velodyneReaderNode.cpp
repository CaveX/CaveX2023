#include <iostream>
#include <chrono>
#include <pcl/PCLPointCloud2.h>
#include <queue>
#include <fstream>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include "nodelet/loader.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ros/names.h"
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

std::string pcapFileName = "Combined_Pcap_SLAM_Verification_Data";
double MAP_RESOLUTION = 0.20;
// velodynePCAPReader reader("/cavex_workspace/dev/CaveX2023/Sample Velodyne Data/MyRoom1.pcap");
// velodynePCAPReader reader("/mnt/c/Users/lukap/OneDrive/Desktop/Study/Fifth Year/Honours/Sample Velodyne Data/2014-11-10-11-32-17_Velodyne-VLP_10Hz_Monterey Highway_SPLIT1.pcap");
// velodynePCAPReader reader("/mnt/c/Users/lukap/OneDrive/Desktop/Study/Fifth Year/Honours/Sample Velodyne Data/" + pcapFileName + ".pcap");
// velodynePCAPReader reader("/mnt/c/misc/Naracoorte Cave Recordings 2023/velodyneSocketReader_LiDAR_Recording1.pcap");
velodynePCAPReader reader("/mnt/c/Users/lukap/OneDrive/Desktop/Study/Fifth Year/Honours/Final Report/Testing/SLAM Verification/SLAM_Testing_Combined.pcap");
velodyneSocketReader sockRead;

LaserMappingClass laserMapping;
LaserProcessingClass laserProcessing;

odomEstimationClass odomEstimation;

std::vector<char> frameBuffer;
std::vector<std::vector<char>> frameBufferQueue;

objPointCloudProcessor objProcessor;

enum ArachnidaParams {
    VELODYNE_READER_LIDAR_DATA_SOURCE,
    VELODYNE_READER_LIDAR_DATA_RECORD_TO_FILE,
    VELODYNE_READER_LIDAR_DATA_RECORD_PATH,
    VELODYNE_READER_LIDAR_DATA_RECORD_MAX_FILE_SIZE,
    FLOAM_NODELET_DEBUG_MESSAGES,
    FLOAM_NODELET_MAP_RESOLUTION,
    FLOAM_NODELET_FREQUENCY,
    FLOAM_NODELET_RESULTS_RECORD_TO_FILE,
    OBSTACLE_DETECTION_NODELET_DISTANCE_TOLERANCE,
    OBSTACLE_DETECTION_NODELET_DEBUG_MESSAGES,
    OBSTACLE_DETECTION_NODELET_FREQUENCY,
    VELODYNE_SOCKET_READER_DEBUG_MESSAGES,
    VELODYNE_READER_DEBUG_MESSAGES,
};

typedef struct ReaderSettings {
    std::string lidar_data_source;
    bool lidar_data_record_to_file;
    std::string lidar_data_record_path;
    int lidar_data_record_max_file_size;
} ReaderSettings;

static std::string getArachnidaParamStr(int arachnidaParam) {
    if(arachnidaParam == VELODYNE_READER_LIDAR_DATA_SOURCE) {
        return "lidar_data_source";
    } else if(arachnidaParam == VELODYNE_READER_LIDAR_DATA_RECORD_TO_FILE) {
        return "lidar_data_record_to_file";
    } else if(arachnidaParam == VELODYNE_READER_LIDAR_DATA_RECORD_PATH) {
        return "lidar_data_record_path";
    } else if(arachnidaParam == VELODYNE_READER_LIDAR_DATA_RECORD_MAX_FILE_SIZE) {
        return "lidar_data_record_max_file_size";
    } else if(arachnidaParam == FLOAM_NODELET_MAP_RESOLUTION) {
        return "floam_map_resolution";
    } else if(arachnidaParam == FLOAM_NODELET_FREQUENCY) {
        return "floam_frequency";
    } else if(arachnidaParam == OBSTACLE_DETECTION_NODELET_DISTANCE_TOLERANCE) {
        return "obstacle_detection_distance_tolerance";
    } else return NULL;
}

static ReaderSettings readerSettings;

int main(int argc, char **argv) {
    ros::init(argc, argv, "velodyneReaderNode");
    ros::NodeHandle nh;
    std::cout << "Instantiating velodyneReaderNode\n"; 

    // Launch nodelets
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    nodelet.load("arachnida/Hello_World", "arachnida/Hello_World", remap, nargv);
    nodelet.load("arachnida/FLOAMNodelet", "arachnida/FLOAMNodelet", remap, nargv);
    nodelet.load("arachnida/ObjectDetectionNodelet", "arachnida/ObjectDetectionNodelet", remap, nargv);
    nodelet.load("arachnida/RobotStateNodelet", "arachnida/RobotStateNodelet", remap, nargv);

    ros::Publisher pcPublisher = nh.advertise<pcl::PCLPointCloud2>("arachnida/point_cloud/pcl", 100);

    nh.param<std::string>(getArachnidaParamStr(VELODYNE_READER_LIDAR_DATA_SOURCE), readerSettings.lidar_data_source, "lidar_data_source_lidar");
    nh.param<bool>(getArachnidaParamStr(VELODYNE_READER_LIDAR_DATA_RECORD_TO_FILE), readerSettings.lidar_data_record_to_file, false);
    nh.param<int>(getArachnidaParamStr(VELODYNE_READER_LIDAR_DATA_RECORD_TO_FILE), readerSettings.lidar_data_record_max_file_size, 62914560);
    nh.param<std::string>(getArachnidaParamStr(VELODYNE_READER_LIDAR_DATA_RECORD_PATH), readerSettings.lidar_data_record_path, "/home/cavex/Documents/");

    ros::Rate loop_rate(10);

    int count = 0;
    int totalFramesProcessed = 0;
    int totalTimeElapsed = 0;
    bool isOdomInitialised = false;

    // sockRead.connect(frameBuffer, frameBufferQueue, pcPublisher, true);
    
    int freq = 1; // i.e 1/2 Hz
    double hz = 1.0/freq;
    
    std::ofstream movementFile("movementData_" + pcapFileName + " MR_" + std::to_string(MAP_RESOLUTION) + "_" + std::to_string(hz) + "Hz" + ".txt"); // this was used for debugging - remove later if not needed
	std::ofstream translationCsv("floamTranslation_" + pcapFileName + " MR_" + std::to_string(MAP_RESOLUTION) + "_" + std::to_string(hz) + "Hz" + ".csv");
	std::ofstream rotationCsv("floamRotation_" + pcapFileName + " MR_" + std::to_string(MAP_RESOLUTION) + "_" + std::to_string(hz) + "Hz" + ".csv");

    while(ros::ok()) {
        ros::spinOnce();
        reader.readFile(); // return a vector of frames
        
        auto startTime = std::chrono::high_resolution_clock::now();
        for(int i = 1; i < reader.getFrameClouds().size(); i++) { // loop through the frames (start at index 1 because we need to compare the current frame to the previous frame)
            if(i > 30) break;
            if(i % freq*10 != 0) continue;
            pcl::PointCloud<pcl::PointXYZI>::Ptr frame = reader.getFrameClouds()[i];
            if(frame->size() < 3000) continue;

            pcl::PCLPointCloud2 cloud2;
            // pcl::PCLPointCloud2ConstPtr(new pcl::PCLPointCloud2(cloud2)) cloud2ptr;
            cloud2.header.seq = i;
            pcl::toPCLPointCloud2(*frame, cloud2);
            pcl::PCLPointCloud2ConstPtr ptrCloud2(new pcl::PCLPointCloud2(cloud2));
            // lidarPub.publish(cloud2);
            pcPublisher.publish(ptrCloud2); // Need to test this - might break stuff
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudEdge(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the edge points from the frame
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudSurf(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the surface points from the frame
            objProcessor.clusterCloud(frame, 0.1, 50, 400);
            laserProcessing.featureExtraction(frame, pointCloudEdge, pointCloudSurf); // extract the edge and surface points from the frame

            totalFramesProcessed++;

            pcl::PointCloud<pcl::PointXYZI>::Ptr pcFilter(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the surface points from the frame
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudInliers(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the surface points from the frame
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudOutliers(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the surface points from the frame
            Eigen::Vector4f minVec = Eigen::Vector4f(-10, -6.2, -2, 1);
            Eigen::Vector4f maxVec = Eigen::Vector4f(15, 7, 10, 1);

            pcFilter = objProcessor.filterCloud(frame, 0.4, minVec, maxVec);

            std::unordered_set<int> inliers = ransacPlane(pcFilter, 10, 0.2);


            for(int index = 0; index < pcFilter->points.size(); index++) {
                pcl::PointXYZI point = pcFilter->points[index];

                if(inliers.count(index)) {
                    pointCloudInliers->points.push_back(point);
                } else {
                    pointCloudOutliers->points.push_back(point);
                }
            }

            KdTree *tree = new KdTree;
            std::vector<std::vector<float>> pointVectors;

            for(int j = 0; j < pointCloudOutliers->points.size(); j++) {
                std::vector<float> pointVector;
                pointVector.push_back(pointCloudOutliers->points[j].x);
                pointVector.push_back(pointCloudOutliers->points[j].y);
                pointVector.push_back(pointCloudOutliers->points[j].z);
                pointVectors.push_back(pointVector);
                tree->insert(pointVector, j);
            }

					std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = euclideanCluster(pointVectors, tree, 0.1, 20);
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
                    odomEstimation.init(MAP_RESOLUTION);
                    odomEstimation.initMapWithPoints(pointCloudEdge, pointCloudSurf);
                    isOdomInitialised = true;
                }

                Eigen::Quaterniond qCurrent(odomEstimation.odom.rotation());
                Eigen::Vector3d tCurrent = odomEstimation.odom.translation();

                tf::Transform transform;
                transform.setOrigin(tf::Vector3(tCurrent.x(), tCurrent.y(), tCurrent.z()));
                tf::Quaternion q(qCurrent.x(), qCurrent.y(), qCurrent.z(), qCurrent.w());
                transform.setRotation(q);

                movementFile << "Transform --------- Frame " << i << "\n";
                movementFile << "Translation: " << tCurrent.x() << ", " << tCurrent.y() << ", " << tCurrent.z() << "\n";
                movementFile << "Rotation: " << qCurrent.x() << ", " << qCurrent.y() << ", " << qCurrent.z() << ", " << qCurrent.w() << "\n\n\n";

                translationCsv << tCurrent.x() << "," << tCurrent.y() << "," << tCurrent.z() << "\n";
                rotationCsv << qCurrent.x() << "," << qCurrent.y() << "," << qCurrent.z() << "," << qCurrent.w() << "\n";
                
            }
            // if(i > 20) break;
        }
        break;
    }
    // reader.readFile(); // return a vector of frames
    // 
    // auto startTime = std::chrono::high_resolution_clock::now();
    // for(int i = 1; i < reader.getFrameClouds().size(); i++) { // loop through the frames (start at index 1 because we need to compare the current frame to the previous frame)
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr frame = reader.getFrameClouds()[i];
    //     if(frame->size() < 3000) continue;
    //     //
    //     //
    //     // pcl::PCLPointCloud2 cloud2;
    //     // cloud2.header.seq = i;
    //     //
    //     // pcl::toPCLPointCloud2(*frame, cloud2);
    //     // 
    //     // pcPublisher.publish(cloud2);
    //     
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudEdge(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the edge points from the frame
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudSurf(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the surface points from the frame
    //
    //     laserProcessing.featureExtraction(frame, pointCloudEdge, pointCloudSurf); // extract the edge and surface points from the frame
    //
    //     totalFramesProcessed++;
    //
    //     // TODO: may need to create a pointCloudFiltered PointCloud to store the surface + edge points (laserMappingNode listens to it for some reason)
    //     std::cout << "pointCloudEdge Size:" << pointCloudEdge->size() << "points \n";
    //     std::cout << "pointCloudSurf Size:" << pointCloudSurf->size() << "points \n";
    //
    //     
    //     if(pointCloudEdge->size() > 0 && pointCloudSurf->size() > 0) {
    //         if(isOdomInitialised) {
    //             auto t1 = std::chrono::high_resolution_clock::now();
    //             odomEstimation.updatePointsToMap(pointCloudEdge, pointCloudSurf);
    //             auto t2 = std::chrono::high_resolution_clock::now();
    //             auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
    //             std::cout << "updatePointsToMap:" << duration << "us \n";
    //         } else {
    //             odomEstimation.init(MAP_RESOLUTION);
    //             odomEstimation.initMapWithPoints(pointCloudEdge, pointCloudSurf);
    //             isOdomInitialised = true;
    //         }
    //
    //         Eigen::Quaterniond qCurrent(odomEstimation.odom.rotation());
    //         Eigen::Vector3d tCurrent = odomEstimation.odom.translation();
    //
    //         tf::Transform transform;
    //         transform.setOrigin(tf::Vector3(tCurrent.x(), tCurrent.y(), tCurrent.z()));
    //         tf::Quaternion q(qCurrent.x(), qCurrent.y(), qCurrent.z(), qCurrent.w());
    //         transform.setRotation(q);
    //
    //         movementFile << "Transform --------- Frame " << i << "\n";
    //         movementFile << "Translation: " << tCurrent.x() << ", " << tCurrent.y() << ", " << tCurrent.z() << "\n";
    //         movementFile << "Rotation: " << qCurrent.x() << ", " << qCurrent.y() << ", " << qCurrent.z() << ", " << qCurrent.w() << "\n\n\n";
    //
    //         translationCsv << tCurrent.x() << "," << tCurrent.y() << "," << tCurrent.z() << "\n";
    //         rotationCsv << qCurrent.x() << "," << qCurrent.y() << "," << qCurrent.z() << "," << qCurrent.w() << "\n";
    //         
    //     }
    //     // if(i > 20) break;
    // }
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-startTime).count();
    // 
    // std::cout << "---------------------------------------------------------------------\n";
    // std::cout << "F-LOAM Total Duration (us): " << duration << "\n";
    // std::cout << "---------------------------------------------------------------------\n";
    //
    // while(ros::ok()) {
    //     ros::spinOnce();
    // }
   //  while(ros::ok()) {
   //      reader.readFile(); // return a vector of frames
   //      for(int i = 1; i < reader.getFrameClouds().size(); i++) { // loop through the frames (start at index 1 because we need to compare the current frame to the previous frame)
   //          pcl::PointCloud<pcl::PointXYZI>::Ptr frame = reader.getFrameClouds()[i];
   //          if(frame->size() < 3000) continue;
			//
			//
			// pcl::PCLPointCloud2 cloud2;
			// cloud2.header.seq = i;
			//
			// pcl::toPCLPointCloud2(*frame, cloud2);
			// 
			// pcPublisher.publish(cloud2);
   //          
   //          pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudEdge(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the edge points from the frame
   //          pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudSurf(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the surface points from the frame
			//
   //          laserProcessing.featureExtraction(frame, pointCloudEdge, pointCloudSurf); // extract the edge and surface points from the frame
			//
   //          totalFramesProcessed++;
			//
   //          // TODO: may need to create a pointCloudFiltered PointCloud to store the surface + edge points (laserMappingNode listens to it for some reason)
   //          std::cout << "pointCloudEdge Size:" << pointCloudEdge->size() << "points \n";
   //          std::cout << "pointCloudSurf Size:" << pointCloudSurf->size() << "points \n";
			//
   //          
   //          if(pointCloudEdge->size() > 0 && pointCloudSurf->size() > 0) {
   //              // if(isOdomInitialised) {
   //              //     auto t1 = std::chrono::high_resolution_clock::now();
   //              //     odomEstimation.updatePointsToMap(pointCloudEdge, pointCloudSurf);
   //              //     auto t2 = std::chrono::high_resolution_clock::now();
   //              //     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
   //              //     std::cout << "updatePointsToMap:" << duration << "us \n";
   //              // } else {
   //              //     odomEstimation.init(0.1);
   //              //     odomEstimation.initMapWithPoints(pointCloudEdge, pointCloudSurf);
   //              //     isOdomInitialised = true;
   //              //     // ROS_INFO("Odom initialised");
   //              // }
			//
   //  //             Eigen::Quaterniond qCurrent(odomEstimation.odom.rotation());
   //  //             Eigen::Vector3d tCurrent = odomEstimation.odom.translation();
			// 	//
   //  //             // static tf::TransformBroadcaster br;
   //  //             tf::Transform transform;
   //  //             transform.setOrigin(tf::Vector3(tCurrent.x(), tCurrent.y(), tCurrent.z()));
   //  //             tf::Quaternion q(qCurrent.x(), qCurrent.y(), qCurrent.z(), qCurrent.w());
   //  //             transform.setRotation(q);
			// 	//
   //  //             movementFile << "Transform --------- Frame " << i << "\n";
   //  //             movementFile << "Translation: " << tCurrent.x() << ", " << tCurrent.y() << ", " << tCurrent.z() << "\n";
   //  //             movementFile << "Rotation: " << qCurrent.x() << ", " << qCurrent.y() << ", " << qCurrent.z() << ", " << qCurrent.w() << "\n\n\n";
			// 	//
			// 	// translationCsv << tCurrent.x() << "," << tCurrent.y() << "," << tCurrent.z() << "\n";
			// 	// rotationCsv << qCurrent.x() << "," << qCurrent.y() << "," << qCurrent.z() << "," << qCurrent.w() << "\n";
   //              
   //          }
   //          // if(i > 20) break;
   //      }
   //      sensor_msgs::PointCloud2 pcFrameMsg;
   //     // veloPublisher.publish(rosMsg);
   //      break;
   //      ros::spinOnce();
   //      loop_rate.sleep();
			//
   //      ++count;
   //  }

    movementFile.close();

    return 0;
}
