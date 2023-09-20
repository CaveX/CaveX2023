#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>

namespace cavex {
	class Nodelet : public nodelet::Nodelet {
		ros::Publisher pub;
		ros::Subscriber sub;

		public:
			virtual void onInit();
			Nodelet();
			~Nodelet();
	};
}
