#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/impl/transforms.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>
#include <math.h>
#include <vector>

#define LASER_CELL_WIDTH 50.0
#define LASER_CELL_HEIGHT 50.0
#define LASER_CELL_DEPTH 50.0

#define LASER_CELL_RANGE_HORIZONTAL 2
#define LASER_CELL_RANGE_VERTICAL 2

class LaserMappingClass {
    private:
        int originInMapX;
        int originInMapY;
        int originInMapZ;
        int mapWidth;
        int mapHeight;
        int mapDepth;
        std::vector<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>>> map;
        pcl::VoxelGrid<pcl::PointXYZI> downSampledFilter;

        void addWidthCellNegative();
        void addWidthCellPositive();
        void addHeightCellNegative();
        void addHeightCellPositive();
        void addDepthCellNegative();
        void addDepthCellPositive();
        void checkPoints(int &x, int &y, int &z);
    
    public:
        LaserMappingClass();
        void init(double mapResolution);
        void updateCurrentPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn, const Eigen::Isometry3d &poseCurrent);
        pcl::PointCloud<pcl::PointXYZI>::Ptr getMap();
};