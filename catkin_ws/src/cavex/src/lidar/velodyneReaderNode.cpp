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


std::string charToHex2(unsigned char hexChar) {
    char charToConvert = (char) hexChar;
    if(charToConvert == '\x00') return "00";
    else if(charToConvert == '\x01') return "01";
    else if(charToConvert == '\x02') return "02";
    else if(charToConvert == '\x03') return "03";
    else if(charToConvert == '\x04') return "04";
    else if(charToConvert == '\x05') return "05";
    else if(charToConvert == '\x06') return "06";
    else if(charToConvert == '\x07') return "07";
    else if(charToConvert == '\x08') return "08";
    else if(charToConvert == '\x09') return "09";
    else if(charToConvert == '\x0A') return "0A";
    else if(charToConvert == '\x0B') return "0B";
    else if(charToConvert == '\x0C') return "0C";
    else if(charToConvert == '\x0D') return "0D";
    else if(charToConvert == '\x0E') return "0E";
    else if(charToConvert == '\x0F') return "0F";

    else if(charToConvert == '\x10') return "10";
    else if(charToConvert == '\x11') return "11";
    else if(charToConvert == '\x12') return "12";
    else if(charToConvert == '\x13') return "13";
    else if(charToConvert == '\x14') return "14";
    else if(charToConvert == '\x15') return "15";
    else if(charToConvert == '\x16') return "16";
    else if(charToConvert == '\x17') return "17";
    else if(charToConvert == '\x18') return "18";
    else if(charToConvert == '\x19') return "19";
    else if(charToConvert == '\x1A') return "1A";
    else if(charToConvert == '\x1B') return "1B";
    else if(charToConvert == '\x1C') return "1C";
    else if(charToConvert == '\x1D') return "1D";
    else if(charToConvert == '\x1E') return "1E";
    else if(charToConvert == '\x1F') return "1F";

    else if(charToConvert == '\x20') return "20";
    else if(charToConvert == '\x21') return "21";
    else if(charToConvert == '\x22') return "22";
    else if(charToConvert == '\x23') return "23";
    else if(charToConvert == '\x24') return "24";
    else if(charToConvert == '\x25') return "25";
    else if(charToConvert == '\x26') return "26";
    else if(charToConvert == '\x27') return "27";
    else if(charToConvert == '\x28') return "28";
    else if(charToConvert == '\x29') return "29";
    else if(charToConvert == '\x2A') return "2A";
    else if(charToConvert == '\x2B') return "2B";
    else if(charToConvert == '\x2C') return "2C";
    else if(charToConvert == '\x2D') return "2D";
    else if(charToConvert == '\x2E') return "2E";
    else if(charToConvert == '\x2F') return "2F";

    else if(charToConvert == '\x30') return "30";
    else if(charToConvert == '\x31') return "31";
    else if(charToConvert == '\x32') return "32";
    else if(charToConvert == '\x33') return "33";
    else if(charToConvert == '\x34') return "34";
    else if(charToConvert == '\x35') return "35";
    else if(charToConvert == '\x36') return "36";
    else if(charToConvert == '\x37') return "37";
    else if(charToConvert == '\x38') return "38";
    else if(charToConvert == '\x39') return "39";
    else if(charToConvert == '\x3A') return "3A";
    else if(charToConvert == '\x3B') return "3B";
    else if(charToConvert == '\x3C') return "3C";
    else if(charToConvert == '\x3D') return "3D";
    else if(charToConvert == '\x3E') return "3E";
    else if(charToConvert == '\x3F') return "3F";

    else if(charToConvert == '\x40') return "40";
    else if(charToConvert == '\x41') return "41";
    else if(charToConvert == '\x42') return "42";
    else if(charToConvert == '\x43') return "43";
    else if(charToConvert == '\x44') return "44";
    else if(charToConvert == '\x45') return "45";
    else if(charToConvert == '\x46') return "46";
    else if(charToConvert == '\x47') return "47";
    else if(charToConvert == '\x48') return "48";
    else if(charToConvert == '\x49') return "49";
    else if(charToConvert == '\x4A') return "4A";
    else if(charToConvert == '\x4B') return "4B";
    else if(charToConvert == '\x4C') return "4C";
    else if(charToConvert == '\x4D') return "4D";
    else if(charToConvert == '\x4E') return "4E";
    else if(charToConvert == '\x4F') return "4F";

    else if(charToConvert == '\x50') return "50";
    else if(charToConvert == '\x51') return "51";
    else if(charToConvert == '\x52') return "52";
    else if(charToConvert == '\x53') return "53";
    else if(charToConvert == '\x54') return "54";
    else if(charToConvert == '\x55') return "55";
    else if(charToConvert == '\x56') return "56";
    else if(charToConvert == '\x57') return "57";
    else if(charToConvert == '\x58') return "58";
    else if(charToConvert == '\x59') return "59";
    else if(charToConvert == '\x5A') return "5A";
    else if(charToConvert == '\x5B') return "5B";
    else if(charToConvert == '\x5C') return "5C";
    else if(charToConvert == '\x5D') return "5D";
    else if(charToConvert == '\x5E') return "5E";
    else if(charToConvert == '\x5F') return "5F";

    else if(charToConvert == '\x60') return "60";
    else if(charToConvert == '\x61') return "61";
    else if(charToConvert == '\x62') return "62";
    else if(charToConvert == '\x63') return "63";
    else if(charToConvert == '\x64') return "64";
    else if(charToConvert == '\x65') return "65";
    else if(charToConvert == '\x66') return "66";
    else if(charToConvert == '\x67') return "67";
    else if(charToConvert == '\x68') return "68";
    else if(charToConvert == '\x69') return "69";
    else if(charToConvert == '\x6A') return "6A";
    else if(charToConvert == '\x6B') return "6B";
    else if(charToConvert == '\x6C') return "6C";
    else if(charToConvert == '\x6D') return "6D";
    else if(charToConvert == '\x6E') return "6E";
    else if(charToConvert == '\x6F') return "6F";

    else if(charToConvert == '\x70') return "70";
    else if(charToConvert == '\x71') return "71";
    else if(charToConvert == '\x72') return "72";
    else if(charToConvert == '\x73') return "73";
    else if(charToConvert == '\x74') return "74";
    else if(charToConvert == '\x75') return "75";
    else if(charToConvert == '\x76') return "76";
    else if(charToConvert == '\x77') return "77";
    else if(charToConvert == '\x78') return "78";
    else if(charToConvert == '\x79') return "79";
    else if(charToConvert == '\x7A') return "7A";
    else if(charToConvert == '\x7B') return "7B";
    else if(charToConvert == '\x7C') return "7C";
    else if(charToConvert == '\x7D') return "7D";
    else if(charToConvert == '\x7E') return "7E";
    else if(charToConvert == '\x7F') return "7F";

    else if(charToConvert == '\x80') return "80";
    else if(charToConvert == '\x81') return "81";
    else if(charToConvert == '\x82') return "82";
    else if(charToConvert == '\x83') return "83";
    else if(charToConvert == '\x84') return "84";
    else if(charToConvert == '\x85') return "85";
    else if(charToConvert == '\x86') return "86";
    else if(charToConvert == '\x87') return "87";
    else if(charToConvert == '\x88') return "88";
    else if(charToConvert == '\x89') return "89";
    else if(charToConvert == '\x8A') return "8A";
    else if(charToConvert == '\x8B') return "8B";
    else if(charToConvert == '\x8C') return "8C";
    else if(charToConvert == '\x8D') return "8D";
    else if(charToConvert == '\x8E') return "8E";
    else if(charToConvert == '\x8F') return "8F";

    else if(charToConvert == '\x90') return "90";
    else if(charToConvert == '\x91') return "91";
    else if(charToConvert == '\x92') return "92";
    else if(charToConvert == '\x93') return "93";
    else if(charToConvert == '\x94') return "94";
    else if(charToConvert == '\x95') return "95";
    else if(charToConvert == '\x96') return "96";
    else if(charToConvert == '\x97') return "97";
    else if(charToConvert == '\x98') return "98";
    else if(charToConvert == '\x99') return "99";
    else if(charToConvert == '\x9A') return "9A";
    else if(charToConvert == '\x9B') return "9B";
    else if(charToConvert == '\x9C') return "9C";
    else if(charToConvert == '\x9D') return "9D";
    else if(charToConvert == '\x9E') return "9E";
    else if(charToConvert == '\x9F') return "9F";

    else if(charToConvert == '\xA0') return "A0";
    else if(charToConvert == '\xA1') return "A1";
    else if(charToConvert == '\xA2') return "A2";
    else if(charToConvert == '\xA3') return "A3";
    else if(charToConvert == '\xA4') return "A4";
    else if(charToConvert == '\xA5') return "A5";
    else if(charToConvert == '\xA6') return "A6";
    else if(charToConvert == '\xA7') return "A7";
    else if(charToConvert == '\xA8') return "A8";
    else if(charToConvert == '\xA9') return "A9";
    else if(charToConvert == '\xAA') return "AA";
    else if(charToConvert == '\xAB') return "AB";
    else if(charToConvert == '\xAC') return "AC";
    else if(charToConvert == '\xAD') return "AD";
    else if(charToConvert == '\xAE') return "AE";
    else if(charToConvert == '\xAF') return "AF";

    else if(charToConvert == '\xB0') return "B0";
    else if(charToConvert == '\xB1') return "B1";
    else if(charToConvert == '\xB2') return "B2";
    else if(charToConvert == '\xB3') return "B3";
    else if(charToConvert == '\xB4') return "B4";
    else if(charToConvert == '\xB5') return "B5";
    else if(charToConvert == '\xB6') return "B6";
    else if(charToConvert == '\xB7') return "B7";
    else if(charToConvert == '\xB8') return "B8";
    else if(charToConvert == '\xB9') return "B9";
    else if(charToConvert == '\xBA') return "BA";
    else if(charToConvert == '\xBB') return "BB";
    else if(charToConvert == '\xBC') return "BC";
    else if(charToConvert == '\xBD') return "BD";
    else if(charToConvert == '\xBE') return "BE";
    else if(charToConvert == '\xBF') return "BF";

    else if(charToConvert == '\xC0') return "C0";
    else if(charToConvert == '\xC1') return "C1";
    else if(charToConvert == '\xC2') return "C2";
    else if(charToConvert == '\xC3') return "C3";
    else if(charToConvert == '\xC4') return "C4";
    else if(charToConvert == '\xC5') return "C5";
    else if(charToConvert == '\xC6') return "C6";
    else if(charToConvert == '\xC7') return "C7";
    else if(charToConvert == '\xC8') return "C8";
    else if(charToConvert == '\xC9') return "C9";
    else if(charToConvert == '\xCA') return "CA";
    else if(charToConvert == '\xCB') return "CB";
    else if(charToConvert == '\xCC') return "CC";
    else if(charToConvert == '\xCD') return "CD";
    else if(charToConvert == '\xCE') return "CE";
    else if(charToConvert == '\xCF') return "CF";

    else if(charToConvert == '\xD0') return "D0";
    else if(charToConvert == '\xD1') return "D1";
    else if(charToConvert == '\xD2') return "D2";
    else if(charToConvert == '\xD3') return "D3";
    else if(charToConvert == '\xD4') return "D4";
    else if(charToConvert == '\xD5') return "D5";
    else if(charToConvert == '\xD6') return "D6";
    else if(charToConvert == '\xD7') return "D7";
    else if(charToConvert == '\xD8') return "D8";
    else if(charToConvert == '\xD9') return "D9";
    else if(charToConvert == '\xDA') return "DA";
    else if(charToConvert == '\xDB') return "DB";
    else if(charToConvert == '\xDC') return "DC";
    else if(charToConvert == '\xDD') return "DD";
    else if(charToConvert == '\xDE') return "DE";
    else if(charToConvert == '\xDF') return "DF";

    else if(charToConvert == '\xE0') return "E0";
    else if(charToConvert == '\xE1') return "E1";
    else if(charToConvert == '\xE2') return "E2";
    else if(charToConvert == '\xE3') return "E3";
    else if(charToConvert == '\xE4') return "E4";
    else if(charToConvert == '\xE5') return "E5";
    else if(charToConvert == '\xE6') return "E6";
    else if(charToConvert == '\xE7') return "E7";
    else if(charToConvert == '\xE8') return "E8";
    else if(charToConvert == '\xE9') return "E9";
    else if(charToConvert == '\xEA') return "EA";
    else if(charToConvert == '\xEB') return "EB";
    else if(charToConvert == '\xEC') return "EC";
    else if(charToConvert == '\xED') return "ED";
    else if(charToConvert == '\xEE') return "EE";
    else if(charToConvert == '\xEF') return "EF";

    else if(charToConvert == '\xF0') return "F0";
    else if(charToConvert == '\xF1') return "F1";
    else if(charToConvert == '\xF2') return "F2";
    else if(charToConvert == '\xF3') return "F3";
    else if(charToConvert == '\xF4') return "F4";
    else if(charToConvert == '\xF5') return "F5";
    else if(charToConvert == '\xF6') return "F6";
    else if(charToConvert == '\xF7') return "F7";
    else if(charToConvert == '\xF8') return "F8";
    else if(charToConvert == '\xF9') return "F9";
    else if(charToConvert == '\xFA') return "FA";
    else if(charToConvert == '\xFB') return "FB";
    else if(charToConvert == '\xFC') return "FC";
    else if(charToConvert == '\xFD') return "FD";
    else if(charToConvert == '\xFE') return "FE";
    else if(charToConvert == '\xFF') return "FF";
    else return "NN";
}


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

<<<<<<< HEAD
    sockRead.connect();
    return;
=======
    sockRead.connect(packetBuffer);
>>>>>>> d0fcf5967c73dc0e15bca2f021ae298bf119a630
    
    std::ofstream movementFile("movementData.txt"); // this was used for debugging - remove later if not needed
    
    while(ros::ok()) {
        if(packetBuffer.size() != 0) {
            for(int i = 0; i < packetBuffer.size(); i++) {
                std::cout << charToHex2(packetBuffer[i]) << " ";
            }
            std::cout << "\n";
        }
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
