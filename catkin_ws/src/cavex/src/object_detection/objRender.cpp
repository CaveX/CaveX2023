#include "object_detection_cpu/objRender.h"

int countRays = 0;

void renderRays(pcl::visualization::PCLVisualizer::Ptr &viewer, const Vect3 &origin, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
    // render rays
    for (int i = 0; i < cloud->points.size(); i++) {
        pcl::PointXYZI point = cloud->points[i];
        viewer->addLine<pcl::PointXYZI>(point, origin, 0, 0, 1, "ray" + std::to_string(countRays));
        countRays++;
    }
}

void clearRays(pcl::visualization::PCLVisualizer::Ptr &viewer) {
    while(countRays) {
        countRays--;
        viewer->removeShape("ray" + std::to_string(countRays));
    }
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::string name, Colour colour) {
    if (colour.r == -1) {
        // Select colour based off of cloud intensity
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
        viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
    } else {
        viewer->addPointCloud<pcl::PointXYZI>(cloud, name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colour.r, colour.g, colour.b, name);
    }
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

void renderBox(pcl::visualization::PCLVisualizer::Ptr &viewer, Box box, int id, Colour colour, float opacity) {
    if(opacity > 1.0) opacity = 1.0;
    if(opacity < 0.0) opacity = 0.0;

    std::string cube = "box" + std::to_string(id);
}
