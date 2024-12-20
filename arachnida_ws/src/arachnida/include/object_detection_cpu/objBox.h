#pragma once

#include <Eigen/Geometry>

struct BoxQ {
    Eigen::Vector3f bboxTransform;
    Eigen::Quaternionf bboxQuaternion;
    float cubeLength;
    float cubeWidth;
    float cubeHeight;
};

struct Box {
    float x_min;
    float y_min;
    float z_min;
    float x_max;
    float y_max;
    float z_max;
};