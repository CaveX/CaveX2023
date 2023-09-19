#include "cavex/nodelets/objectDetectionNodelet.h"
#include <pluginlib/class_list_macros.h>

namespace cavex {

	objectDetectionNodelet::objectDetectionNodelet() {
		ROS_INFO("[objectDetectionNodelet] Created");
	}

	objectDetectionNodelet::~objectDetectionNodelet() {
		ROS_INFO("[objectDetectionNodelet] Destroyed");
	}

	void objectDetectionNodelet::onInit() {
		ROS_INFO("[objectDetectionNodelet] Initialising");
		// do stuff
		
		ROS_INFO("[objectDetectionNodelet] Initialised");
	}

}

PLUGINLIB_EXPORT_CLASS(cavex::objectDetectionNodelet, nodelet::Nodelet)