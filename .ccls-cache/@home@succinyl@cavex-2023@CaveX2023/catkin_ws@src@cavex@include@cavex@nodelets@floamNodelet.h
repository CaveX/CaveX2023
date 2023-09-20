#pragma once

#include "cavexNodelet.h"

namespace cavex {
	class floamNodelet : public cavex::Nodelet {
		public:
			virtual void onInit();
			floamNodelet();
			~floamNodelet();
	};
}
