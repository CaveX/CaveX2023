#include "../../include/arachnida/path_planning/pathPlanningNode.h"
#include "../../include/arachnida/path_planning/artificialPotentialField.h"

arachnida::pathPlanner::pathPlanner(void) {
    ros::NodeHandle nh;

    current_position_sub_ = nh.subscribe("arachnida/robot_state/current_pose", 1, &arachnida::pathPlanner::currentPositionCallback, this); 
    cumulative_position_sub_ = nh.subscribe("arachnida/robot_state/cumulative_pose", 1, &arachnida::pathPlanner::cumulativePositionCallback, this);
    obstacle_list_sub_ = nh.subscribe("arachnida/object_detection/objects_detected", 1, &arachnida::pathPlanner::objectsDetectedCallback, this);
    
    desired_velocity_pub_ = nh.advertise<geometry_msgs::Twist>("syropod_remote/desired_velocity",1);
}

void arachnida::pathPlanner::currentPositionCallback(const geometry_msgs::PoseStamped &currentPoseMsg) {

}

void arachnida::pathPlanner::cumulativePositionCallback(const geometry_msgs::PoseStamped &cumulativePoseMsg) {

}

void arachnida::pathPlanner::objectsDetectedCallback(const arachnida::ObstacleList &obstacleList) {

}

void arachnida::pathPlanner::

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pathPlanNode");

  ros::Rate loop_rate(10);
  
  Eigen::Vector3d start(0,0,0);
  Eigen::Vector3d end(10,0,0);
  Eigen::Vector3d currPosition(0,0,0);
  Eigen::Vector3d desiredPosition(0,0,0);
  
  arachnida::path_planning::artificialPotentialField APF(start,end,currPosition);

  while(ros::ok())
  {
    desiredPosition = APF.generateDirectionVector(currPosition);
    //desired_velocity_msg_.linear.x = desiredPosition.x();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}