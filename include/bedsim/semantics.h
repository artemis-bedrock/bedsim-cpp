#pragma once

#include "bedsim/constants.h"

#include <cmath>
#include <cstdint>
#include <string_view>

namespace bedsim {
	enum class Bounce : uint8_t {
		None,
		Slime,
		Bed
	};

	enum class Inside : uint8_t {
		None,
		SweetBerryBush,
		PowderSnow
	};

	enum class Traversal : uint8_t {
		None,
		Scaffolding,
		PowderSnow
	};

	struct BlockMovement {
		float groundFriction{ kBlockFriction };
		float accelerationFriction{ 1.0f };
		Bounce bounce{};
		Inside inside{};
		Traversal traversal{};
		bool air{};
		bool climbable{};
		bool cobweb{};
		bool honey{};
		bool fenceLike{};
		bool soulSpeedNeutralizesFriction{};

		[[nodiscard]] BlockMovement validated(const BlockMovement& fallback) const {
			BlockMovement result = *this;
			if (!(groundFriction > 0.0f) || std::isinf(groundFriction)) {
				result.groundFriction = fallback.groundFriction;
			}

			if (!(accelerationFriction > 0.0f) || std::isinf(accelerationFriction)) {
				result.accelerationFriction = fallback.accelerationFriction;
			}

			return result;
		}

		[[nodiscard]] BlockMovement validated() const {
			return validated(BlockMovement{});
		}
	};

	[[nodiscard]] BlockMovement vanillaMovement(std::string_view fullName);
}
