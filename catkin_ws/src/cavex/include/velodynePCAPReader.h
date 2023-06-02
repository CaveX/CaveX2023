#pragma once
#include <fstream>
#include <vector>
#include <iterator>
// This class is primarily for developing the GPU-accelerated version of F-LOAM

#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct velodyneVLP16Point {
    float distance;
    float reflectivity;
    int channel;
};

struct velodyneVLP16DataBlock {
    float azimuth;
    std::array<velodyneVLP16Point, 32> points; // array store of points (distances) for VLP-16
};

struct velodyneVLP16Packet {
    float timestamp;
    bool dualReturnMode = false;
    std::array<velodyneVLP16DataBlock, 12> dataBlocks;
};

class velodynePCAPReader {    
    private:
        std::string absolutePath;
        std::vector<char> pcapBuffer;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;
        int laserAngles[16] = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15}; // laser angles in deg (index of entry is the laser ID)       

    public:
        velodynePCAPReader(std::string absolutePath);
        
        std::string getAbsolutePath() { return absolutePath; }

        std::vector<char> getPCAPBuffer() { return pcapBuffer; }

        pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud() { return pointCloud; }

        void readFile();

        void readNextPacket();

        void readBytes(int byteCount);
};