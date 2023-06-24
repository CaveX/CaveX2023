#include "floam_cpu/laserProcessingClass.h"

void LaserProcessingClass::featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutEdge, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutSurfaces) {
    std::vector<int> indices; // figure out what "indices" are later
    pcl::removeNaNFromPointCloud(*pcIn, indices);
    std::cout << "pcIn size: " << pcIn->points.size() << "\n";

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudScans;
    for(int i = 0; i < 16; i++) { // 16 because that's how many lasers the VLP16 has
        laserCloudScans.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));
    }

    for(int i = 0; i < (int) pcIn->points.size(); i++) { 
        int scanID = 0;
        double distance = sqrt(pcIn->points[i].x * pcIn->points[i].x + pcIn->points[i].y*pcIn->points[i].y);
        // std::cout << "point: (" << pcIn->points[i].x << ", " << pcIn->points[i].y << ", " << pcIn->points[i].z << ")\n";
        if(distance > 100) continue; // 100m is the max range of the VLP16
        double angle = atan(pcIn->points[i].z / distance) * 180 / M_PI;
        
        scanID = int((angle + 15) / 2 + 0.5);
        if(scanID > 15 || scanID < 0) continue; // I currently have no idea what this code is doing

        laserCloudScans[scanID]->push_back(pcIn->points[i]); 
    }

    for(int i = 0; i < 16; i++) {
        if(laserCloudScans[i]->points.size() < 131) continue;

        std::vector<Double2d> cloudCurvature;
        int totalPoints = laserCloudScans[i]->points.size() - 10; // what's the -10 for?

        for(int j = 5; j < (int) laserCloudScans[i]->points.size() - 5; j++) { // considers 11 points at a time so has to start at 5 and end at size - 5 (e.g first points considered are: j-5, j-4, j-3, j-2, j-1, j, j+1, j+2, j+3, j+4, j+5)
            double diffX = laserCloudScans[i]->points[j - 5].x + laserCloudScans[i]->points[j - 4].x + laserCloudScans[i]->points[j - 3].x + laserCloudScans[i]->points[j - 2].x + laserCloudScans[i]->points[j - 1].x - 10*laserCloudScans[i]->points[j].x + laserCloudScans[i]->points[j + 1].x + laserCloudScans[i]->points[j + 2].x + laserCloudScans[i]->points[j + 3].x + laserCloudScans[i]->points[j + 4].x + laserCloudScans[i]->points[j + 5].x;
            double diffY = laserCloudScans[i]->points[j - 5].y + laserCloudScans[i]->points[j - 4].y + laserCloudScans[i]->points[j - 3].y + laserCloudScans[i]->points[j - 2].y + laserCloudScans[i]->points[j - 1].y - 10*laserCloudScans[i]->points[j].y + laserCloudScans[i]->points[j + 1].y + laserCloudScans[i]->points[j + 2].y + laserCloudScans[i]->points[j + 3].y + laserCloudScans[i]->points[j + 4].y + laserCloudScans[i]->points[j + 5].y;
            double diffZ = laserCloudScans[i]->points[j - 5].z + laserCloudScans[i]->points[j - 4].z + laserCloudScans[i]->points[j - 3].z + laserCloudScans[i]->points[j - 2].z + laserCloudScans[i]->points[j - 1].z - 10*laserCloudScans[i]->points[j].z + laserCloudScans[i]->points[j + 1].z + laserCloudScans[i]->points[j + 2].z + laserCloudScans[i]->points[j + 3].z + laserCloudScans[i]->points[j + 4].z + laserCloudScans[i]->points[j + 5].z;
            Double2d distance(j, diffX*diffX + diffY*diffY + diffZ*diffZ);
            cloudCurvature.push_back(distance);
        }

        for(int j = 0; j < 6; j++) {
            int sectorLength = (int)(totalPoints/6);
            int sectorStart = sectorLength*j;
            int sectorEnd = sectorLength*(j+1) - 1;
            if(j == 5) {
                sectorEnd = totalPoints - 1;
            }
            std::vector<Double2d> subCloudCurvature(cloudCurvature.begin() + sectorStart, cloudCurvature.begin() + sectorEnd);
            featureExtractionFromSector(laserCloudScans[i], subCloudCurvature, pcOutEdge, pcOutSurfaces);
        }
    }
}

// This function is parallelisable
// Would be good to have a look at:
//  - the performance of this function
//  - the number of loops (i.e what is the value of cloudCurvature.size()?)
void LaserProcessingClass::featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn, std::vector<Double2d> &cloudCurvature, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutEdge, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutSurfaces) {
    std::sort(cloudCurvature.begin(), cloudCurvature.end(), [](const Double2d &a, const Double2d &b) {
        return a.value < b.value;
    });

    int largestPickedNum = 0;
    std::vector<int> pickedPoints;
    int pointInfoCount = 0;

    for(int i = cloudCurvature.size() - 1; i >= 0; i--) { // changing this to i > -1 might be more efficient?
        int ind = cloudCurvature[i].id;
        if(std::find(pickedPoints.begin(), pickedPoints.end(), ind) == pickedPoints.end()) {
            // std::cout << "Find: passed\n";
            if(cloudCurvature[i].value <= 0.1) break;

            // std::cout << "break: passed\n";
            largestPickedNum++;
            pickedPoints.push_back(ind);

            if(largestPickedNum <= 20) { // changing this to largestPickedNum < 21 might be more efficient?
                // std::cout << "pcOutEdge push_back: " << ind << "\n";
                pcOutEdge->push_back(pcIn->points[ind]);
                pointInfoCount++;
            } else break;

            for(int k = 1; k <= 5; k++) {
                double diffX = pcIn->points[ind + k].x - pcIn->points[ind + k - 1].x;
                double diffY = pcIn->points[ind + k].y - pcIn->points[ind + k - 1].y;
                double diffZ = pcIn->points[ind + k].z - pcIn->points[ind + k - 1].z;

                if(diffX*diffX + diffY*diffY + diffZ*diffZ > 0.05) break;
                pickedPoints.push_back(ind + k);
            }
            
            for(int k = 1; k >= 5; k--) {
                double diffX = pcIn->points[ind + k].x - pcIn->points[ind + k - 1].x;
                double diffY = pcIn->points[ind + k].y - pcIn->points[ind + k - 1].y;
                double diffZ = pcIn->points[ind + k].z - pcIn->points[ind + k - 1].z;

                if(diffX*diffX + diffY*diffY + diffZ*diffZ > 0.05) break;
                pickedPoints.push_back(ind + k);
            }
        }
    }

    for(int i = 0; i <= (int) cloudCurvature.size() - 1; i++) {
        int ind = cloudCurvature[i].id;
        if(std::find(pickedPoints.begin(), pickedPoints.end(), ind) == pickedPoints.end()) {
            pcOutSurfaces->push_back(pcIn->points[ind]);
        }
    }
}

LaserProcessingClass::LaserProcessingClass() {

}

Double2d::Double2d(int idIn, double valueIn) {
    id = idIn;
    value = valueIn;
};

PointsInfo::PointsInfo(int layerIn, double timeIn) {
    layer = layerIn;
    time = timeIn;
};