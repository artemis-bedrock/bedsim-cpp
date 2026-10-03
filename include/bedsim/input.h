#pragma once

#include "bedsim/vec.h"

#include <optional>

namespace bedsim {
	struct Input {
		Vec2 moveVector{};
		bool moveVectorIsRaw{};

		float pitch{};
		float yaw{};
		float headYaw{};

		Vec3 clientPosition{};
		Vec3 clientVelocity{};

		bool horizontalCollision{};
		bool verticalCollision{};

		bool startFlying{};
		bool stopFlying{};

		bool startSprinting{};
		bool stopSprinting{};
		bool sprintDown{};

		bool startSneaking{};
		bool stopSneaking{};
		bool sneakDown{};
		bool sneaking{};

		bool startJumping{};
		bool jumping{};
		bool autoJumpingInWater{};
		bool ascendBlock{};

		bool startSwimming{};
		bool stopSwimming{};
		bool wantDown{};
		bool wantDownSlow{};
		bool startCrawling{};
		bool stopCrawling{};
		bool descendBlock{};

		bool stopGliding{};
		bool startGliding{};

		std::optional<float> itemUseMovementModifier;

		bool usingConsumable{};
		bool usingItem{};
		bool usingSpear{};
		bool inventoryAction{};

		bool startSpinAttack{};
		bool stopSpinAttack{};
	};
}
