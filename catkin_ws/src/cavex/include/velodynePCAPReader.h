#pragma once
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class velodynePCAPReader {    
    private:
        std::string absolutePath;
        std::vector<char> pcapBuffer;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;        

    public:
        velodynePCAPReader(std::string absolutePath);
        
        std::string getAbsolutePath() { return absolutePath; }

        std::vector<char> getPCAPBuffer() { return pcapBuffer; }

        void readFile();

        void readNextPacket();

        void readBytes(int byteCount);
};