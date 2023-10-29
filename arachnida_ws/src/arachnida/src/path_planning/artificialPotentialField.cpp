#include "../../include/arachnida/path_planning/artificialPotentialField.h"

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

float arachnida::path_planning::artificialPotentialField::calculateObstaclePotential(Obstacle obstacle) {
    float obstacleDistance = calculateDistanceToObstacle(currPosition, obstacle.location);
    float safeDistance = obstacleDistance - minimumRadius;
    return obstacle.gainRepulsiveForce / safeDistance;
}

float arachnida::path_planning::artificialPotentialField::calculateObstaclePotential(Eigen::Vector3d position, Obstacle obstacle) {
    float obstacleDistance = calculateDistanceToObstacle(position, obstacle.location);
    float safeDistance = obstacleDistance - minimumRadius;
    return obstacle.gainRepulsiveForce / safeDistance;
}

float arachnida::path_planning::artificialPotentialField::calculateOverallLocalPotential(float goalDistance, std::vector<Obstacle> obstacles) {
    float localPotential = 0.0;
    for (int i = 0; i < obstacles.size() - 1; i++) {
        localPotential += calculateObstaclePotential(obstacles.at(i));
    }

    return calculateGoalPotential(goalDistance) + localPotential;
}

void arachnida::path_planning::artificialPotentialField::updateRepulsiveGain(std::vector<Obstacle> obstacles) {
    for (int i = 0; i < obstacles.size() - 1; i++) {
        float distance = calculateDistanceToObstacle(currPosition,obstacles.at(i).location);
        obstacles.at(i).gainRepulsiveForce -= exp(-2*distance);

        if (obstacles.at(i).gainRepulsiveForce > 1) {
            obstacles.at(i).gainRepulsiveForce = 1;
        } else if (obstacles.at(i).gainRepulsiveForce < 0.05) {
            obstacles.at(i).gainRepulsiveForce = 0.05;
        }
    }
}

Eigen::Vector3d arachnida::path_planning::artificialPotentialField::generateDirectionVector(Eigen::Vector3d currPosition) {
    float x_ahead = currPosition.x() + stepSize;
    float y_ahead = currPosition.y() + stepSize;
    float z_ahead = currPosition.z() + stepSize;
    float x_behind = currPosition.x() - stepSize;
    float y_behind = currPosition.y() - stepSize;
    float z_behind = currPosition.z() - stepSize;

    Eigen::Vector3d aheadPosition(x_ahead,y_ahead,z_ahead);
    Eigen::Vector3d behindPosition(x_behind,y_behind,z_behind);

    float goal_pot_ahead = calculateGoalPotential(calculateDistanceToGoal(aheadPosition,goal));
    float goal_pot_behind = calculateGoalPotential(calculateDistanceToGoal(behindPosition,goal));

    float obs_pot_ahead = 0.0;
    float obs_pot_behind = 0.0;

    for (int i = 0; i < obstacles.size(); i++) {
        obs_pot_ahead += calculateObstaclePotential(aheadPosition,obstacles.at(i));
        obs_pot_behind += calculateObstaclePotential(behindPosition,obstacles.at(i));
    }

    float pot_ahead = goal_pot_ahead + obs_pot_ahead;
    float pot_behind = goal_pot_behind + obs_pot_behind;

    
}

int main() {
    return 0;
}