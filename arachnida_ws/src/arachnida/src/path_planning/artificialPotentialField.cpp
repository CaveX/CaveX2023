#include "arachnida/path_planning/ppArtificialPotentialField.h"

void arachnida::path_planning::artificialPotentialField::updateTheta() {

};

void arachnida::path_planning::artificialPotentialField::applyCostFunction() {

};

void arachnida::path_planning::artificialPotentialField::applyCostFunctionToGoal() {

};

void arachnida::path_planning::artificialPotentialField::applyCostFunctionToObstacles() {

};

void arachnida::path_planning::artificialPotentialField::updateArtificialPointsCoordinates() {

};

void arachnida::path_planning::artificialPotentialField::updateArtificialPoints() {

};

void arachnida::path_planning::artificialPotentialField::calculateError() {

};

void arachnida::path_planning::artificialPotentialField::calculateFitness() {
	for(int i = 0; i < obstacles.size(); i++) {
		
	}
};

void arachnida::path_planning::artificialPotentialField::decideNextMove() {

};

void arachnida::path_planning::artificialPotentialField::takeNextMove() {

};

Eigen::Vector3d arachnida::path_planning::artificialPotentialField::calculateRepulsiveForce(Obstacle obstacle, Eigen::Vector3d target, double radius) {
	Eigen::Vector3d F_rep(0,0,0); // Repulsive force initialised to zero (zero in x, y, and z directions)
	double zeta = 3*radius;
	int n = 2;
	double dist = obstacle.computeDistance2D(target); // euclidean distance between obstacle and target
	
	Eigen::Vector3d distVec = obstacle.computeDistanceVec2D(target); // x and y distance between obstacle and target (z = 0 for 2D function)
	

	if(dist <= zeta) {
			F_rep = gainRepulsiveForce * ((1/dist)-(1/zeta))*dist*distVec;
	}
	
	return F_rep;

};

Eigen::Vector3d arachnida::path_planning::artificialPotentialField::calculateAttractiveForce(Obstacle obstacle, Eigen::Vector3d target) {
	Eigen::Vector3d F_att(0,0,0); // Attractive force initialised to zero (zero in x, y, and z directions)
	Eigen::Vector3d dist = obstacle.location - target;
	F_att = gainAttractiveForce * dist;
	return F_att;
};
