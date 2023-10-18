#pragma once

#include <cstdio>
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>
#include <string>

#include "../../arachnida/utils.h"

#include <eigen3/Eigen/Dense>

namespace arachnida {
    namespace path_planning {

        typedef struct Obstacle {
            int id;
            Eigen::Vector3d location;
            float radius;
            float gainRepulsiveForce = 0.05;
        } Obstacle;

        class artificialPotentialField{
            private:
                Eigen::Vector3d start; // start position of robot
                Eigen::Vector3d goal; // goal position of robot
                Eigen::Vector3d currPosition; // instantaneous position of the robot
                std::vector<Obstacle> obstacles; // vector of obstacles detected
                std::map<Eigen::Vector3d, double> distanceToGoal;
                float gainAttractiveForce = 1.0;
                float minimumRadius = 0.5; // Keep at least 0.5 m away from obstacle
                ros::Publisher desired_direction_pub_;
            public:
                artificialPotentialField(Eigen::Vector3d start, Eigen::Vector3d goal, Eigen::Vector3d curPosition);
                float calculateDistanceToGoal(Eigen::Vector3d currPosition, Eigen::Vector3d goal);
                float calculateDistanceToObstacle(Eigen::Vector3d currPosition, Eigen::Vector3d obstacleLocation);
                float calculateGoalPotential(float distance);
                float calculateObstaclePotential(float distance, Obstacle obstacle);
                float calculateOverallLocalPotential(std::vector<Obstacle> obstacles);
                void updateRepulsiveGain(std::vector<Obstacle> obstacles);
                Eigen::Vector3d generateDirectionVector(Eigen::Vector3d currPosition);
                
        };
    }
}