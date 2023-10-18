#include "../../include/arachnida/path_planning/tylerArtificialPotentialField.h"

arachnida::path_planning::artificialPotentialField::artificialPotentialField(Eigen::Vector3d start, Eigen::Vector3d goal, Eigen::Vector3d currPosition) {
    start = start;
    goal = goal;
    currPosition = currPosition;
}

float arachnida::path_planning::artificialPotentialField::calculateDistanceToGoal(Eigen::Vector3d currPosition, Eigen::Vector3d goal) {
    return sqrt(pow(goal.x() - currPosition.x(), 2) + pow(goal.y() - currPosition.y(), 2) + pow(goal.z() - currPosition.z(), 2));
}

float arachnida::path_planning::artificialPotentialField::calculateDistanceToObstacle(Eigen::Vector3d currPosition, Eigen::Vector3d obstacleLocation) {
    return sqrt(pow(obstacleLocation.x() - currPosition.x(), 2) + pow(obstacleLocation.y() - currPosition.y(), 2) + pow(obstacleLocation.z() - currPosition.z(), 2));
}

float arachnida::path_planning::artificialPotentialField::calculateGoalPotential(float distance) {
    float goalDistance = calculateDistanceToGoal(currPosition, goal);
    return gainAttractiveForce / goalDistance;
}

float arachnida::path_planning::artificialPotentialField::calculateObstaclePotential(float distance, Obstacle obstacle) {
    float obstacleDistance = calculateDistanceToObstacle(currPosition, obstacle.location);
    float safeDistance = obstacleDistance - minimumRadius;
    return obstacle.gainRepulsiveForce / safeDistance;
}

float arachnida::path_planning::artificialPotentialField::calculateOverallLocalPotential(std::vector<Obstacle> obstacles) {
    float localPotential = 0.0;
    for (int i = 0; i < obstacles.size() - 1; i++) {
        localPotential += obstacles.at(i).gainRepulsiveForce;
    }

    return gainAttractiveForce + localPotential;
}

void arachnida::path_planning::artificialPotentialField::updateRepulsiveGain(std::vector<Obstacle> obstacles) {
    for (int i = 0; i < obstacles.size() - 1; i++) {
        float distance = calculateDistanceToObstacle(currPosition,obstacles.at(i).location);
        obstacles.at(i).gainRepulsiveForce -= exp(-2*distance);
    }
}

Eigen::Vector3d arachnida::path_planning::artificialPotentialField::generateDirectionVector(Eigen::Vector3d currPosition) {

}

int main() {
    return 0;
}