#include "object_detection_cpu/objRansac.h"

std::unordered_set<int> ransacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const int maxIterations, const float distanceTolerance) {
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    const int pointsToFit = 3;
    for(int i = 0; i < maxIterations; i++) {
        // Select three random points
        std::unordered_set<int> inliers;
        for(int i = 0; i < pointsToFit; i++) {
            inliers.insert((rand() % cloud->points.size()));
        }

        float x1, y1, z1;
        float x2, y2, z2;
        float x3, y3, z3;

        auto itr = inliers.begin();

        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;

        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;

        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        // Calculate the plane coefficients (a, b, c, d)
        // Equation of a plane: Ax + By + Cz - d = 0
        const float a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
        const float b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
        const float c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
        const float d = -(a*x1 + b*y1 + c*z1);

        for(size_t i = 0; cloud->points.size(); i++) {
            if(inliers.count(i) > 0) continue;

            pcl::PointXYZI p = cloud->points[i];
            float x4 = p.x;
            float y4 = p.y;
            float z4 = p.z;

            // Estimate hte distance of each point and check if it's within the distanceTolerance
            const float D = fabs(a*x4 + b*y4 + c*z4 + d) / sqrt(a*a + b*b + c*c);
            if(D < distanceTolerance) inliers.insert(i);
        }

        if(inliers.size() > inliersResult.size()) inliersResult = inliers;
    }

    return inliersResult;
}