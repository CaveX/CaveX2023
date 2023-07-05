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

struct velodyneVLP16Point {
    float distance;
    float reflectivity;
    int channel;
};

struct velodyneVLP16DataBlock {
    float azimuth;
    std::vector<velodyneVLP16Point> points; // array store of points (distances) for VLP-16
};  

struct velodyneVLP16Packet {
    unsigned int timestamp;
    bool dualReturnMode = false;
    std::vector<velodyneVLP16DataBlock> dataBlocks;
};

struct velodyneVLP16Frame {
    std::vector<velodyneVLP16Packet> packets;
};

// 1 frame = points from one full rotation of the VLP16
struct velodyneVLP16FrameDataBlocks {
    std::vector<velodyneVLP16DataBlock> dataBlocks; // all the dataBlocks in a frame
};

class velodyneSocketReader {
    private:
        int PORT;
        int sockfd;
        int socketID;
        int valRead;
        sockaddr_in address;
        int opt = 1;
        int addrlen = sizeof(address);
        char buffer[1248] = { 0 };
        std::vector<char> pcapBuffer;
        std::vector<velodyneVLP16Packet> packets;
        std::vector<velodyneVLP16Frame> frames;
        std::vector<velodyneVLP16FrameDataBlocks> frameDataBlocks;
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> frameClouds; // vector of point clouds; one point cloud per frame (this is what would be passed to SLAM)
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;
        int laserAngles[16] = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15}; // laser angles in deg (index of entry is the laser ID)
    public:
        velodyneSocketReader();
        int getPort() { return PORT; }
        int getSockFileDescriptor() { return sockfd; }
        int getSocketID() { return socketID; }
        int getValRead() { return valRead; }
        sockaddr_in getAddress() { return address; }
        int getOpt() { return opt; }
        void connect();
        void disconnect();
};