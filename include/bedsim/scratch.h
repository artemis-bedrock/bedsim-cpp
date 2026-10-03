#pragma once

#include "bedsim/aabb.h"
#include "bedsim/types.h"

#include <vector>

namespace bedsim {
	struct Scratch {
		std::vector<AABB> boxes;
		std::vector<AABB> probe;
		std::vector<AABB> blockBoxes;
		std::vector<BlockPos> water;
		std::vector<BlockPos> lava;
		std::vector<float> firstCuts;
		std::vector<float> secondCuts;
	};
}
