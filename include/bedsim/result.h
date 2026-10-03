#pragma once

#include "bedsim/body.h"
#include "bedsim/vec.h"

#include <cstdint>
#include <vector>

namespace bedsim {
	enum class Outcome : uint8_t {
		Normal,
		Teleport,
		Unreliable,
		UnloadedChunk,
		ImmobileOrNotReady,
		Mounted,
		InvalidInput
	};

	struct Result {
		Vec3 position{};
		Vec3 velocity{};
		Vec3 movement{};
		Vec2 inputMoveVector{};
		bool sprintMovementBlocked{};

		bool onGround{};
		bool collideX{};
		bool collideY{};
		bool collideZ{};

		Vec3 positionDelta{};
		Vec3 velocityDelta{};
		bool needsCorrection{};

		Outcome outcome{};

		bool landed{};
		float landingFallDistance{};
	};

	struct HeadLiquid {
		Vec3 position{};
		bool water{};
		bool lava{};
		bool known{};
	};

	struct ReplayFrame {
		Body body;
		Result result;
		HeadLiquid headLiquid;
	};

	struct ReplayResult {
		Body body;
		std::vector<ReplayFrame> frames;
	};
}
