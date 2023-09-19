#pragma once

#include "cavexNodelet.h"

namespace cavex {
	class objectDetectionNodelet : public cavex::baseNodelet {
		public:
			virtual void onInit();
			objectDetectionNodelet();
			~objectDetectionNodelet();
	};
};