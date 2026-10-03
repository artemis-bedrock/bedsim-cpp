#pragma once

#include <cstdint>

namespace bedsim {
	inline constexpr float kJumpHeight = 0.42f;
	inline constexpr float kAirFriction = 0.91f;
	inline constexpr float kBlockFriction = 0.6f;
	inline constexpr float kGravity = 0.08f;
	inline constexpr float kGravityDrag = 0.98f;
	inline constexpr float kSlowFallingGravity = 0.01f;
	inline constexpr float kLevitationSpeed = 0.05f;
	inline constexpr float kStepHeight = 0.5625f;
	inline constexpr float kSlideOffsetMultiplier = 0.4f;
	inline constexpr float kSlimeBounce = -1.0f;
	inline constexpr float kBedBounce = -0.75f;
	inline constexpr float kClimbSpeed = 0.2f;
	inline constexpr float kMaxConsumingImpulse = 0.1225f;
	inline constexpr float kMaxSneakImpulse = 0.3f;
	inline constexpr float kUnderwaterMovementSpeed = 0.02f;
	inline constexpr float kLavaMovementSpeed = 0.02f;
	inline constexpr float kSwimSpeedMultiplier = 1.0f;
	inline constexpr float kGlideLookEpsilon = 1e-4f;
	inline constexpr float kGlideFallDistanceVelocity = -0.5f;
	inline constexpr float kWalkAirSpeed = 0.02f;
	inline constexpr float kSprintAirSpeed = 0.026f;
	inline constexpr float kSprintSpeedMultiplier = 1.3f;
	inline constexpr float kDefaultMovementSpeed = 0.1f;
	inline constexpr float kImpulseScale = 0.98f;
	inline constexpr float kSprintJumpBoost = 0.2f;
	inline constexpr float kHoneyJumpMultiplier = 0.6f;
	inline constexpr float kWaterDrag = 0.8f;
	inline constexpr float kSoulSandAccelerationFriction = 1.225000023841858f;
	inline constexpr float kPlayerWidth = 0.6f;
	inline constexpr float kPlayerHeight = 1.8f;
	inline constexpr float kSneakingHeight = 1.49f;
	inline constexpr float kCrawlingHeight = 0.6f;
	inline constexpr float kEyeHeight = 1.62f;
	inline constexpr float kSneakingEyeHeight = 1.27f;
	inline constexpr float kCompactEyeHeight = 0.4f;
	inline constexpr float kTerminalVelocity = -3.92f;
	inline constexpr float kMinimumBlockCoordinate = -2147483648.0f;
	inline constexpr float kMaximumBlockCoordinate = 2147483520.0f;
	inline constexpr uint64_t kJumpDelayTicks = 10;
	inline constexpr int64_t kGlideBoostTicks = 20;
	inline constexpr int64_t kSwimWaterGraceTicks = 10;
	inline constexpr int kRiptideTicks = 20;
}
