#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include "./artificialPotentialField.h"

#include "../../arachnida/utils.h"
#include "arachnida/ObstacleList.h"

#include <eigen3/Eigen/Dense>

namespace arachnida {
    class pathPlanner {
        public:
            pathPlanner(void);
            void currentPositionCallback(const geometry_msgs::PoseStamped &currentPoseMsg);
            void cumulativePositionCallback(const geometry_msgs::PoseStamped &cumulativePoseMsg);
            void objectsDetectedCallback(const arachnida::ObstacleList &obstacleList);

        private:
            // subscriber to current position of the robot from SLAM
            ros::Subscriber current_position_sub_;

            // subscriber to cumulative position of the robot from SLAM
            ros::Subscriber cumulative_position_sub_;

            // subscriber to list of obstacles
            ros::Subscriber obstacle_list_sub_;

            // publisher to topic /syropod_remote/desired_velocity
            ros::Publisher desired_velocity_pub_;

            // message published to topic /syropod_remote/desired_velocity
            geometry_msgs::Twist desired_velocity_msg_;
    };
}
