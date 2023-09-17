#include "cavex/nodelets/floamNodelet.h"
#include <pluginlib/class_list_macros.h>

namespace cavex {

	floamNodelet::floamNodelet() {
		ROS_INFO("[floamNodelet] Created");
	}

	floamNodelet::~floamNodelet() {
		ROS_INFO("[floamNodelet] Destroyed");
	}

	void floamNodelet::onInit() {
		NODELET_INFO("[floamNodelet] Initialising");
		// do stuff
		NODELET_INFO("[floamNodelet] Initialised");
	}

}

PLUGINLIB_EXPORT_CLASS(cavex::floamNodelet, nodelet::Nodelet)
