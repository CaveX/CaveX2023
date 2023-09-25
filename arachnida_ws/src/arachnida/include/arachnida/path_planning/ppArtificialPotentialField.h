#pragma once

#include <cstdio>
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>
#include <string>

#include "arachnida/utils.h"

#include <Eigen/Dense>

namespace arachnida {
    namespace path_planning {
        
        typedef struct Obstacle {
            int id;
            std::string name;
			Eigen::Vector3d location;
            float radius;
			float alpha; // depth of attractant/repellant in artificial potential field
			float sigma; // width of attractant/repellant in artificial potential field

            Obstacle(int id, std::string name, Eigen::Vector3d location, float radius, float alpha, float sigma) : id(id), name(name), location(location), radius(radius), alpha(alpha), sigma(sigma) {};

            Obstacle() : id(0), name(""), location(Eigen::Vector3d(0,0,0)), radius(0), alpha(0.0), sigma(0.0) {};

            double computeDistance(Eigen::Vector3d point) {
                return sqrt(pow(point.x() - location.x(), 2) + pow(point.y() - location.y(), 2) + pow(point.z() - location.z(), 2));
            };

			double computeDistance2D(Eigen::Vector3d point) {
                return sqrt(pow(point.x() - location.x(), 2) + pow(point.y() - location.y(), 2));
			};

            double computeDistance(Obstacle obstacle) {
                return sqrt(pow(obstacle.location.x() - location.x(), 2) + pow(obstacle.location.y() - location.y(), 2) + pow(obstacle.location.z() - location.z(), 2));
            };
            
			double computeDistance2D(Obstacle obstacle) {
                return sqrt(pow(obstacle.location.x() - location.x(), 2) + pow(obstacle.location.y() - location.y(), 2));
            };

			Eigen::Vector3d computeDistanceVec(Eigen::Vector3d point) {
				return Eigen::Vector3d((point.x() - location.x()),(point.y() - location.y()),(point.z() - location.z()));
			};
			
			Eigen::Vector3d computeDistanceVec2D(Eigen::Vector3d point) {
				return Eigen::Vector3d((point.x() - location.x()),(point.y() - location.y()), 0);
			};

            inline bool operator == (const Obstacle &o) {
                return (id == o.id && name == o.name && location == o.location && radius == o.radius && alpha == o.alpha && sigma == o.sigma);
            };

            inline bool operator != (const Obstacle &o) {
                return (id != o.id || name != o.name || location != o.location || radius != o.radius || alpha != o.alpha || sigma != o.sigma);
            };

            inline friend std::ostream& operator << (std::ostream &out, const Obstacle o) {
                out << "id: " << o.id << ", name: " << o.name << ", location: " << o.location << ", radius: " << o.radius << ", alpha: " << o.alpha << ", sigma: " << o.sigma;
                return out;
            };

        } Obstacle;

        class artificialPotentialField {
            private:
                Eigen::Vector3d start; // start position of the robot
                Eigen::Vector3d goal; // goal position of the robot
                Eigen::Vector3d curPosition; // current position of the robot
                std::vector<Obstacle> obstacles; // list of obstacles in the environment
				std::map<Obstacle, double> distanceToGoal; // stores the current distance from an obstacle to the goal
                float goalThreshold = 1.0; // distance from the goal position that is considered "close enough" to the goal
                float gainRepulsiveForce = 0.1; // gain for the repulsive force
                float gainAttractiveForce = 0.04; // gain for the attractive force
                float stepSize; // step size for the robot to take
				int iterationCounter = 0; // counts the number of iterations (number of times new cost functions have been calculated for everything)
            public:
				artificialPotentialField(Eigen::Vector3d start, Eigen::Vector3d goal, Eigen::Vector3d curPosition) : start(start), goal(goal), curPosition(curPosition) {};
                Eigen::Vector3d getStartPosition() { return start; };
                void setStartPosition(Eigen::Vector3d pos) { start = pos; };
                Eigen::Vector3d getGoalPosition() { return goal; };
                void setGoalPosition(Eigen::Vector3d pos) { goal = pos; };
                Eigen::Vector3d getCurrentPosition() { return curPosition; };
                void setCurrentPosition(Eigen::Vector3d pos) { curPosition = pos; };

				void updateTheta(); // Calculates angle thetas responsible for finding x and y position of artificial points 
				void applyCostFunction(); // Calculates the total potential of a point with respect to the goal and obstacles
				void applyCostFunctionToGoal(); // Calculates the potential of the curPosition with respect to the current position (curPosition)
				void applyCostFunctionToObstacles(); // Calculates the potential of all obstacles with respect to curPosition
				void updateArtificialPointsCoordinates(); // Updates all artificial points of robot
				void updateArtificialPoints(); // Updates potential, distance to goal, error, and fitness of artificial points
				void calculateError(); // Calculates the error in potential and distance to goal of artificial points with respect to goal and obstacles
				void calculateFitness(); // Calculates the fitness based on error of artificial points with respect to the goal and obstaacles
				void decideNextMove();
				void takeNextMove();

				Eigen::Vector3d calculateRepulsiveForce(Obstacle obstacle, Eigen::Vector3d target, double radius); // Calculates the repulsive force (x, y, z vector) between an obstacle and a target
				Eigen::Vector3d calculateAttractiveForce(Obstacle obstacle, Eigen::Vector3d target); // Calculates the repulsive force (x, y, z vector) between an obstacle and a target
				
        };
    };
};
