#pragma once
#include <vector>
#include <iterator>
#include <iostream>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <chrono>
#include <cmath>
#include "velodyneSocketReader.h"

// // Stores laser angles in radians for VLP16
// // - Stores them in order of channel ID (i.e first element is the angle for channel 0, second element is the angle for channel 1, etc.)
// // - Stores them twice for performance improvement in retrieval
// // -- Each data block contains two firings from each laser and thus the channel ID can go up to 32 rather than 16
// // -- Storing them twice means we don't have to subtract 16 from the channel ID when it is > 15
// double laserChannelAngles[32] = { -0.261799,  0.0174533, -0.226893, 0.0523599, -0.191986, 0.0872665, -0.15708,  0.122173, 
//                                  -0.122173,  0.15708,   -0.0872665, 0.191986, -0.0523599, 0.226893, -0.0174533, 0.261799,
//                                  -0.261799,  0.0174533, -0.226893, 0.0523599, -0.191986, 0.0872665, -0.15708,  0.122173, 
//                                  -0.122173,  0.15708,   -0.0872665, 0.191986, -0.0523599, 0.226893, -0.0174533, 0.261799 }; 

double getLaserAngleFromChannelID(int channelID);

double getLaserChannelTiming(int channelID, int dataBlockID);

// Parses packet assuming Single Return mode
void parsePacketToPointCloud(std::vector<char> const &packet, pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud);

// Parses packet assuming Single Return Mode
void parsePacketToPointCloudSocketReaderPcap(std::vector<char> const &packet, pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud);

// Parses packet assuming Single Return mode
// void parsePacketToDataBlocks(char *packet, std::vector<sock_velodyneVLP16DataBlock> &dataBlocks);
void parsePacketToDataBlocks(std::vector<char> const &packet, std::vector<sock_velodyneVLP16DataBlock> &dataBlocks);

// Parses packet assuming Single Return mode
void parsePacketToDataBlocksSocketReaderPcap(std::vector<char> const &packet, std::vector<sock_velodyneVLP16DataBlock> &dataBlocks);

// Parses frame assuming Single Return mode
void parseFrameToPointCloud(std::vector<char> &frame, pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud);

// Parses frame from pcap files created from the writeToFile option in velodyneSocketReader to point cloud
void parseFrameToPointCloudForSocketReaderPcapFiles(std::vector<char> &frame, pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud);

// Parses frame assuming Single Return mode
void parseFrameToPackets(char *frame, std::vector<sock_velodyneVLP16Packet> &packets);

// Parses frame assuming Single Return mode
void parseFrameToDataBlocks(char *frame, std::vector<sock_velodyneVLP16DataBlock> &dataBlocks);

// Parses a raw buffer of bytes (such as what is produced by velodynePCAPReader) to a vector of frames - Intended to be used with pcap files 
// recorded using the writeToFile option in velodyneSocketReader
void parseSocketReaderPcapToPointCloud(char* pcapBuffer, int pcapBufferSize, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &pointCloudFramesVec);
