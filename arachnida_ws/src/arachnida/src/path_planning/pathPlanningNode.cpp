#include "../../include/arachnida/path_planning/pathPlanningNode.h"
#include "../../include/arachnida/path_planning/artificialPotentialField.h"

void arachnida::pathPlanner::pathPlannerInitialiser(void) {
  ROS_INFO("\nInitialising Path Planner Node...\n");
  APF = arachnida::path_planning::artificialPotentialField();

  Eigen::Vector3d start(0,0,0);
  Eigen::Vector3d currPos(0,0,0);
  Eigen::Vector3d goal(0,0,0);

  APF.setStart(start);
  APF.setCurrPosition(currPos);
  APF.setGoal(goal);

  ROS_INFO("\nPath Planner Node Finished Initialising.\n");
}

arachnida::pathPlanner::pathPlanner(void) {
    pathPlannerInitialiser();
    ros::NodeHandle nh;

    current_position_sub_ = nh.subscribe("arachnida/robot_state/current_pose", 1, &arachnida::pathPlanner::currentPositionCallback, this); 
    //cumulative_position_sub_ = nh.subscribe("arachnida/robot_state/cumulative_pose", 1, &arachnida::pathPlanner::cumulativePositionCallback, this);
    obstacle_list_sub_ = nh.subscribe("arachnida/object_detection/objects_detected", 1, &arachnida::pathPlanner::objectsDetectedCallback, this);
    
    desired_velocity_pub_ = nh.advertise<geometry_msgs::Twist>("syropod_remote/desired_velocity",1);
}

void arachnida::pathPlanner::currentPositionCallback(const nav_msgs::Odometry &currentPoseMsg) {
  double x = currentPoseMsg.pose.pose.position.x;
  double y = currentPoseMsg.pose.pose.position.y;
  double z = currentPoseMsg.pose.pose.position.z;

  Eigen::Vector3d currPos(x,y,z);

  if (currPos == APF.getGoal()) {
    Eigen::Vector3d goalPos(rand() % 5, rand() % 5, 0);
    APF.setGoal(goalPos);
    ROS_INFO("\nSetting New Goal Position\n");
  }
  APF.setCurrPosition(currPos);
}

// void arachnida::pathPlanner::cumulativePositionCallback(const geometry_msgs::PoseStamped &cumulativePoseMsg) {
//   //APF.setCumulativePosition(cumulativePoseMsg);
//   ROS_INFO("Cumulative Pose Recieved... Doing nothing\n");
// }

void arachnida::pathPlanner::objectsDetectedCallback(const arachnida::ObstacleList &obstacleList) {
  APF.setObstacleList(obstacleList);
}

void arachnida::pathPlanner::planPath(void) {
  desiredDirection = APF.generateDirectionVector();
  publishDirection(desiredDirection);
  ROS_INFO("\nPlanning Path:");
  std::cout << "x: " << desiredDirection.x() << " y: " << desiredDirection.y() << " z: " << desiredDirection.z() << std::endl;
}

void arachnida::pathPlanner::publishDirection(Eigen::Vector3d direction) {
  desired_velocity_msg_.linear.x = direction.x();
  desired_velocity_msg_.linear.y = direction.y();
  desired_velocity_msg_.linear.z = 0;
  desired_velocity_msg_.angular.x = 0;
  desired_velocity_msg_.angular.y = 0;
  desired_velocity_msg_.angular.z = 0;

  desired_velocity_pub_.publish(desired_velocity_msg_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pathPlanNode");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  
  Eigen::Vector3d start(0,0,0);
  arachnida::pathPlanner pathPlanner;


  while(ros::ok())
  {
    pathPlanner.planPath();
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}