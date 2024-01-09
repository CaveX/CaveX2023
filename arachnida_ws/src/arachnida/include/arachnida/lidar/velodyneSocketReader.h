#pragma once
#include <fstream>
#include <vector>
#include <iterator>
#include <iostream>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <chrono>
#include "ros/ros.h"

#define FRAME_SIZE_BYTES 94037 // approx. 94037 bytes per frame (VLP-16 manual reports data rate of 940368 bytes/sec)
#define MAX_FRAME_BUFFER_QUEUE_SIZE_BYTES 4701840 // approx. 5 seconds worth of data (940368 bytes/sec * 5 sec)

namespace arachnida {
    struct sock_velodyneVLP16Point {
        float distance;
        float reflectivity;
        int channel;
    };

    struct sock_velodyneVLP16DataBlock {
        float azimuth;
        std::vector<sock_velodyneVLP16Point> points; // array store of points (distances) for VLP-16
    };  

    struct sock_velodyneVLP16Packet {
        unsigned int timestamp;
        bool dualReturnMode = false;
        std::vector<sock_velodyneVLP16DataBlock> dataBlocks;
    };

    struct sock_velodyneVLP16Frame {
        std::vector<sock_velodyneVLP16Packet> packets;
    };

    // Stores the data blocks (sock_velodyneVLP16DataBlocks) for a single frame
    // 1 frame = points from one full rotation of the VLP16
    struct sock_velodyneVLP16FrameDataBlocks {
        std::vector<sock_velodyneVLP16DataBlock> dataBlocks; // all the dataBlocks in a frame
    };

    class velodyneSocketReader {
        private:
            int PORT = 2368;
            int sockfd;
            int socketID;
            int valRead;
            sockaddr_in address;
            int opt = 1;
            int addrlen = sizeof(address);
            char buffer[1206] = { 0 }; // deprecated (superceded by packetBuffer)
            int laserAngles[16] = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15}; // laser angles in deg (index of entry is the laser ID) // deprecated (superceded by velodyneUtils.cpp)
            std::chrono::time_point<std::chrono::system_clock> lastPacketTimestamp;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;
        public:
            velodyneSocketReader();
            int getPort() { return PORT; }
            int getSockFileDescriptor() { return sockfd; }
            int getSocketID() { return socketID; }
            int getValRead() { return valRead; }
            sockaddr_in getAddress() { return address; }
            int getOpt() { return opt; }

            // Connects to the LiDAR via a socket listening to 
            // the specified port and stores the data in the 
            // packetBuffer until a full frame is received.
            // A full frame is 100ms of data, which is roughly 
            // 94037 bytes of data (VLP-16 manual reports data 
            // rate of 940368 bytes/sec)
            // @param frameBuffer: stores the raw binary data from 
            // the lidar until a full frame is received.
            // @param frameBufferQueue: stores yet-to-be-processed
            // lidar frames as needed
            // @param lidarPub: the ROS publisher object 
            // the lidar data should be published to.
            // @param writeToFile: true if the lidar 
            // data should be written to a file, false
            // if not. NOTE: The file contains only 
            // the LiDAR portion of the whole UDP packet.
            // Existing software for viewing the data as
            // a point cloud (e.g VeloView) needs the 
            // UDP headers to be in the file to render
            // the data. Hence, to use the data, you'll
            // need to write a script which appends dummy
            // UDP headers to each packet in the file.
            void connect(
                std::vector<char> &frameBuffer, 
                std::vector<std::vector<char>> &frameBufferQueue, 
                ros::Publisher &lidarPub, 
                bool writeToFile
            );
            
            // Close the socket listening to the VLP-16 and
            // print a message to indicate succesful closure.
            void disconnect();
            // static std::vector<char> packetBuffer; // stores the raw binary data from the lidar
            static std::vector<sock_velodyneVLP16Packet> packets;
            static std::vector<sock_velodyneVLP16Frame> frames;
            static std::vector<sock_velodyneVLP16FrameDataBlocks> frameDataBlocks;
            static std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> frameClouds; // vector of point clouds; one point cloud per frame (this is what would be passed to SLAM)
    };
}