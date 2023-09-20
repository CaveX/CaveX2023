#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

class Double2d {
    public:
        int id;
        double value;
        Double2d(int idIn, double valueIn);
};

class PointsInfo {
    public:
        int layer;
        double time;
        PointsInfo(int layerIn, double timeIn);
};

class LaserProcessingClass {
    public:
        LaserProcessingClass();
        void featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutEdges, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutSurfaces);
        void featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn, std::vector<Double2d> &cloudCurvature, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutEdges, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutSurfaces);
};