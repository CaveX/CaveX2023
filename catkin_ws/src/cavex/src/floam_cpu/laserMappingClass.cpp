#include "laserMappingClass.h"

void LaserMappingClass::init(double mapResolution) {
    for(int i = 0; i < LASER_CELL_RANGE_HORIZONTAL*2+1; i++) {
        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> mapHeightTemp;
        for(int j = 0; j < LASER_CELL_RANGE_HORIZONTAL*2+1; j++) {
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> mapDepthTemp;
            for(int k=0; k < LASER_CELL_RANGE_VERTICAL*2+1; k++) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudTemp(new pcl::PointCloud<pcl::PointXYZI>);
                mapDepthTemp.push_back(pointCloudTemp);
            }
            mapHeightTemp.push_back(mapDepthTemp);
        }
        map.push_back(mapHeightTemp);
    }

    originInMapX = LASER_CELL_RANGE_HORIZONTAL;
    originInMapY = LASER_CELL_RANGE_HORIZONTAL;
    originInMapZ = LASER_CELL_RANGE_VERTICAL;
    mapWidth = LASER_CELL_RANGE_HORIZONTAL*2+1;
    mapHeight = LASER_CELL_RANGE_HORIZONTAL*2+1;
    mapDepth = LASER_CELL_RANGE_HORIZONTAL*2+1;

    // downsampling size
    downSampledFilter.setLeafSize(mapResolution, mapResolution, mapResolution);
}

// // void LaserMappingClass::addWidthCellNegative() {
//     std::vector<std::vector<pcl::PointCloud
// }