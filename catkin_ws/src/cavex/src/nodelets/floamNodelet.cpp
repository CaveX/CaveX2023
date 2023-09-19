// #include "cavex/nodelets/floamNodelet.h"
#include <pluginlib/class_list_macros.h>

#include "geometry_msgs/PoseStamped.h"

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <string>


namespace cavex {

	class floamNodelet : public nodelet::Nodelet {
		public:
			// virtual void onInit();
			// floamNodelet();
			// ~floamNodelet();
			ros::Publisher pub;
			ros::Subscriber sub;
		private:
			virtual void onInit() {
				ROS_INFO("[floamNodelet] Initialising");
				// do stuff
				ros::NodeHandle &private_nh = getPrivateNodeHandle();
				pub = private_nh.advertise<geometry_msgs::PoseStamped>("/floam_pose", 10);
				
				ROS_INFO("[floamNodelet] Initialised");
			}
	};
	// floamNodelet::floamNodelet() {
	// 	// baseNodelet();
	// };

	// floamNodelet::~floamNodelet() {
	// 	// baseNodelet::~Nodelet();
	// };

	// void floamNodelet::onInit() {
	// 	ROS_INFO("[floamNodelet] Initialising");
	// 	// do stuff
	// 	// ros::NodeHandle &private_nh = nodelet::getPrivateNodeHandle();
	// 	// pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("/floam_pose", 10);
		
	// 	ROS_INFO("[floamNodelet] Initialised");
	// };
};

PLUGINLIB_EXPORT_CLASS(cavex::floamNodelet, nodelet::Nodelet)