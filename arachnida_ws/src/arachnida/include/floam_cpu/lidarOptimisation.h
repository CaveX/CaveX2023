#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// SE3 = Special Euclidean group 3D
// q = quarternion = orientation
// i'm guessing the t vector is the translation vector
void getTransformFromSe3(const Eigen::Matrix<double,6,1> &se3, Eigen::Quaterniond &q, Eigen::Vector3d &t);

Eigen::Matrix3d skew(Eigen::Vector3d &matIn);

class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
    public:
        EdgeAnalyticCostFunction(Eigen::Vector3d currentPoint, Eigen::Vector3d lastPointA, Eigen::Vector3d lastPointB);
        virtual ~EdgeAnalyticCostFunction() {}
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

        Eigen::Vector3d currentPoint;
        Eigen::Vector3d lastPointA;
        Eigen::Vector3d lastPointB;
};

class SurfNormAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
    public:
        SurfNormAnalyticCostFunction(Eigen::Vector3d currentPoint, Eigen::Vector3d planeUnitNormalised, double negativeOADotNormalised);
        virtual ~SurfNormAnalyticCostFunction() {}
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

        Eigen::Vector3d currentPoint;
        Eigen::Vector3d planeUnitNormalised;
        double negativeOADotNormalised;
};

class PoseSE3Parameterisation : public ceres::LocalParameterization {
    public:
        PoseSE3Parameterisation() {}
        virtual ~PoseSE3Parameterisation() {}
        virtual bool Plus(const double *x, const double *delta, double *xPlusDelta) const;
        virtual bool ComputeJacobian(const double *x, double *jacobian) const;
        virtual int GlobalSize() const { return 7; }
        virtual int LocalSize() const { return 6; }
};