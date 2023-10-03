#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <stdio.h>

namespace arachnida {
	class Hello_World : public nodelet::Nodelet {
		public:
			Hello_World() {}

		private:
			virtual void onInit() {
				ros::NodeHandle &private_nh = getPrivateNodeHandle();
				pub = private_nh.advertise<std_msgs::String>("ros_out",10);
				sub = private_nh.subscribe("ros_in",10, &Hello_World::callback, this);
			};
		
		ros::Publisher pub;
		ros::Subscriber sub;

		void callback(const std_msgs::String::ConstPtr &input) {
			std_msgs::String output;
			output.data = input->data;
			ROS_INFO("msg data = %s", output.data.c_str());
			pub.publish(output);
		};
	};

	PLUGINLIB_EXPORT_CLASS(arachnida::Hello_World, nodelet::Nodelet);
};
