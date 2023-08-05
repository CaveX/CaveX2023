#include "object_detection_cpu/objRansac.h"
#include <iostream>

std::unordered_set<int> ransacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const int maxIterations, const float distanceTolerance) {
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    const int pointsToFit = 3;
    for(int i = 0; i < maxIterations; i++) {
        // Select three random points
        std::unordered_set<int> inliers;
        for(int j = 0; j < pointsToFit; j++) {
            inliers.insert((rand() % cloud->points.size()));
        }
        if(inliers.size() > 2) {

            std::cout << "objRansaic - 3\n";
            float x1, y1, z1;
            float x2, y2, z2;
            float x3, y3, z3;

            std::cout << "objRansaic - 4\n";
            auto itr = inliers.begin();

            std::cout << "objRansaic - 4.05 : " << inliers.size() << "\n";

            std::cout << "objRansaic - 4.1 : " << *itr << "\n";

            std::cout << "objRansaic - 5\n";
            x1 = cloud->points[*itr].x;
            y1 = cloud->points[*itr].y;
            z1 = cloud->points[*itr].z;

            std::cout << "objRansaic - 5.1 : " << *itr << "\n";

            std::cout << "objRansaic - 6\n";
            itr++;
            x2 = cloud->points[*itr].x;
            y2 = cloud->points[*itr].y;
            z2 = cloud->points[*itr].z;

            std::cout << "objRansaic - 6.1 : " << *itr << "\n";

            std::cout << "objRansaic - 7\n";
            itr++;

            // Segfault happens when accessing *itr
            std::cout << "objRansaic - 7.05\n";
            std::cout << "objRansaic - 7.1 : " << *itr << "\n";
            x3 = cloud->points[*itr].x;

            std::cout << "objRansaic - 7.2\n";
            y3 = cloud->points[*itr].y;

            std::cout << "objRansaic - 7.3\n";
            z3 = cloud->points[*itr].z;

            // Calculate the plane coefficients (a, b, c, d)
            // Equation of a plane: Ax + By + Cz - d = 0

            std::cout << "objRansaic - 8\n";
            const float a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
            const float b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
            const float c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
            const float d = -(a*x1 + b*y1 + c*z1);


            std::cout << "objRansaic - 9\n";
            for(int k = 0; k < cloud->points.size(); k++) {
                if(inliers.count(k) > 0) continue;

                // std::cout << "8\n";
                pcl::PointXYZI p = cloud->points[k];
                float x4 = p.x;
                float y4 = p.y;
                float z4 = p.z;

                // Estimate the distance of each point and check if it's within the distanceTolerance
                const float D = fabs(a*x4 + b*y4 + c*z4 + d) / sqrt(a*a + b*b + c*c);
                if(D < distanceTolerance)  {
                    inliers.insert(k);
                }
            }
        }

        // The fit with the most inliers will be selected
        if(inliers.size() > inliersResult.size()) {
            std::cout << "objRansaic - 1\n";
            inliersResult = inliers;
            std::cout << "objRansaic - 2\n";
        }
    }

    return inliersResult;
}