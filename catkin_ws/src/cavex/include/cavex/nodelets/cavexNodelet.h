#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <string>

namespace cavex {
	class baseNodelet : public nodelet::Nodelet {

		public:
			ros::Publisher pub_;
			ros::Subscriber sub_;

			baseNodelet() {
				ROS_INFO("[%s] Created", nodelet::Nodelet::getName().c_str());
			};

			~baseNodelet() {
				ROS_INFO("[%s] Destroyed", nodelet::Nodelet::getName().c_str());
			};

			virtual void onInit();			
	};
};
