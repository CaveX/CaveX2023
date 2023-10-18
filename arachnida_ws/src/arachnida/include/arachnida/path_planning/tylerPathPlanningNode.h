#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include "tylerArtificialPotentialField.h"

#include "../../arachnida/utils.h"

#include <eigen3/Eigen/Dense>

class pathPlanner {
    public:
        pathPlanner(void);

    private:
        // subscriber to current position of the robot from SLAM
        ros::Subscriber curr_position_sub_;

        // subscriber to list of obstacles
        ros::Subscriber obstacle_list_sub_;

        // subscriber to desired direction
        ros::Subscriber desired_direction_sub_;

        // publisher to topic /syropod_remote/desired_velocity
        ros::Publisher desired_velocity_pub_;

        // message published to topic /syropod_remote/desired_velocity
        geometry_msgs::Twist desired_velocity_msg_;
};
