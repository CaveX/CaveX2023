#pragma once
#include <fstream>
#include <vector>
#include <iterator>
// This class is primarily for developing the GPU-accelerated version of F-LOAM

#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

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

class velodynePCAPReader {    
    private:
        std::string absolutePath;
        std::vector<char> pcapBuffer;
        std::vector<velodyneVLP16Packet> packets;
        std::vector<velodyneVLP16Frame> frames;
        std::vector<velodyneVLP16FrameDataBlocks> frameDataBlocks;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;
        int laserAngles[16] = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15}; // laser angles in deg (index of entry is the laser ID)       

    public:
        velodynePCAPReader(std::string absolutePath);
        
        std::string getAbsolutePath() { return absolutePath; }

        std::vector<char> getPCAPBuffer() { return pcapBuffer; }

        std::vector<velodyneVLP16Packet> getPackets() { return packets; }

        std::vector<velodyneVLP16Frame> getFrames() { return frames; }

        pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud() { return pointCloud; }

        std::string charToHex(unsigned char charToConvert);

        void readFile();

        void readNextPacket();

        void readBytes(int byteCount);
};