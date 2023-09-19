#pragma once

// #include "cavexNodelet.h"

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <string>

namespace cavex {
	// class floamNodelet : public cavex::baseNodelet {
	class floamNodelet : public nodelet::Nodelet {
		public:
			virtual void onInit();
			// floamNodelet();
			// ~floamNodelet();
	};
};
