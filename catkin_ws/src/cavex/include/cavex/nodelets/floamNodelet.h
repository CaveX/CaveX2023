#pragma once

#include "cavexNodelet.h"

namespace cavex {
	class floamNodelet : public cavex::cavexNodelet {
		public:
			virtual void onInit();
			floamNodelet();
			~floamNodelet();
	}
}
