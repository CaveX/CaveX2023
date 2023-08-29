#pragma once

#include <cstdio>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>

// #include "utils.h"
// #include "../../utils.h"
#include "cavex/utils.h"

namespace cavex {
    namespace path_planning {
        
        typedef struct Obstacle {
            int id;
            std::string name;
            Vector3D location;
            float radius;

            Obstacle(int id, std::string name, cavex::Vector3D location, float radius) : id(id), name(name), location(location), radius(radius) {};

            Obstacle() : id(0), name(""), location(Vector3D()), radius(0) {};

            double computeDistance(cavex::Vector3D point) {
                return sqrt(pow(point.x - location.x, 2) + pow(point.y - location.y, 2) + pow(point.z - location.z, 2));
            };

            double computeDistance(Obstacle obstacle) {
                return sqrt(pow(obstacle.location.x - location.x, 2) + pow(obstacle.location.y - location.y, 2) + pow(obstacle.location.z - location.z, 2));
            };

            inline bool operator == (const Obstacle &o) {
                return (id == o.id && name == o.name && location == o.location && radius == o.radius);
            };

            inline bool operator != (const Obstacle &o) {
                return (id != o.id || name != o.name || location != o.location || radius != o.radius);
            };

            inline friend std::ostream& operator << (std::ostream &out, const Obstacle o) {
                out << "id: " << o.id << ", name: " << o.name << ", location: " << o.location << ", radius: " << o.radius;
                return out;
            };

        } Obstacle;

        class artificialPotentialField {
            private:
                cavex::Vector3D start; // start position of the robot
                cavex::Vector3D goal; // goal position of the robot
                cavex::Vector3D curPosition; // current position of the robot
                std::vector<Obstacle> obstacles; // list of obstacles in the environment
                float goalThreshold; // distance from the goal position that is considered "close enough" to the goal
                float gainRepulsiveForce; // gain for the repulsive force
                float gainAttractiveForce; // gain for the attractive force
                float stepSize; // step size for the robot to take
            public:
                cavex::Vector3D getStartPosition() { return start; };
                void setStartPosition(cavex::Vector3D pos) { start = pos; };
                cavex::Vector3D getGoalPosition() { return goal; };
                void setGoalPosition(cavex::Vector3D pos) { goal = pos; };
                cavex::Vector3D getCurrentPosition() { return curPosition; };
                void setCurrentPosition(cavex::Vector3D pos) { curPosition = pos; };
        };
    };
};