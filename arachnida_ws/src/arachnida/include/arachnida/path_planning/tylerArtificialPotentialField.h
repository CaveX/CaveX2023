#pragma once

#include <iostream>
#include <Eigen/Dense>

#include "../../arachnida/utils.h"

namespace arachnida {
    namespace path_planning {

        typedef struct Obstacle {
            int id;
            Eigen::Vector3D location;
            float radius;
            float gainRepulsiveForce = 0.05;
        } Obstacle;

        class artificialPotentialField{
            private:
                Eigen::Vector3D start; // start position of robot
                Eigen::Vector3D goal; // goal position of robot
                Eigen::Vector3D currPosition; // instantaneous position of the robot
                std::vector<Obstacle> obstacles; // vector of obstacles detected
                std::map<Eigen::Vector3D, double> distanceToGoal;
                float gainAttractiveForce = 1.0;
                float minimumRadius = 0.5; // Keep at least 0.5 m away from obstacle
            public:
                float calculateDistanceToGoal(Eigen::Vector3D currPosition, Eigen::Vector3D goal);
                float calculateDistanceToObstacle(Eigen::Vector3D currPosition, Eigen::Vector3D obstacleLocation);
                float calculateGoalPotential(float distance);
                float calculateObstaclePotential(float distance);
                float calculateOverallLocalPotential(std::vector<Obstacle> obstacles);
                void updateRepulsiveGain(Obstacle obstacle);
                Eigen::Vector3D generateDirectionVector(Eigen::Vector3D currPosition);

        };
    }
}