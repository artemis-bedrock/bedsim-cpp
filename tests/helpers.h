#pragma once

#include "bedsim/format.h"

#include <cmath>

namespace bedsim::test {
	[[nodiscard]] inline bool near(const float value, const float expected, const float tolerance = 1e-6f) {
		return std::abs(value - expected) <= tolerance;
	}
}
