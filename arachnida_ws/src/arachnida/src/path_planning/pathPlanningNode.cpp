#include "arachnida/path_planning/pathPlanningNode.h"

pathPlanner::pathPlanner(void) {
    ros::NodeHandle nh;

    // current_position_sub_ = nh.subscribe("/arachnida/robot_state/current_pose", 1, currentPositionCallback); 
    // cumulative_position_sub_ = nh.subscribe("/arachnida/robot_state/cumulative_pose", 1, cumulativePositionCallback);
}

void currentPositionCallback(const geometry_msgs::PoseStamped &currentPoseMsg) {

}

void cumulativePositionCallback(const geometry_msgs::PoseStamped &cumulativePoseMsg) {

}

