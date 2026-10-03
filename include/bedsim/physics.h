#pragma once

#include "bedsim/aabb.h"
#include "bedsim/body.h"
#include "bedsim/input.h"
#include "bedsim/semantics.h"
#include "bedsim/types.h"
#include "bedsim/vec.h"

#include <cstdint>
#include <span>
#include <string_view>

namespace bedsim {
	inline constexpr float kContactEpsilon = 1e-6f;

	struct Clip {
		Vec3 velocity{};
		Vec3 penetration{};
	};

	struct StepResult {
		AABB box{};
		Vec3 velocity{};
		Vec3 up{};
		Vec3 x{};
		Vec3 z{};
		Vec3 down{};
		int boxCount{};
	};

	enum class ModifierOperation : uint8_t {
		Addition,
		MultiplyBase,
		MultiplyTotal,
		Cap
	};

	enum class ModifierOperand : uint8_t {
		Min,
		Max,
		Current
	};

	struct AttributeModifier {
		std::string_view id;
		ModifierOperation operation{};
		ModifierOperand operand{};
		float amount{};
	};

	[[nodiscard]] float mcSin(float radians);
	[[nodiscard]] float mcCos(float radians);

	[[nodiscard]] bool isFinite(float value);
	[[nodiscard]] bool isFinite(const Vec2& value);
	[[nodiscard]] bool isFinite(const Vec3& value);
	[[nodiscard]] bool isFinite(const Body& body);
	[[nodiscard]] bool isFinite(const Input& input);

	[[nodiscard]] Clip clipCollide(const AABB& stationary, const AABB& moving, const Vec3& velocity, bool oneWay, const Vec3& penetration = {});
	[[nodiscard]] Clip clipAll(std::span<const AABB> boxes, const AABB& moving, const Vec3& velocity, bool oneWay, const Vec3& penetration = {});
	[[nodiscard]] StepResult autoStep(const AABB& box, const Vec3& velocity, std::span<const AABB> boxes, bool oneWay);
	[[nodiscard]] AABB recoverRoundedContacts(const AABB& box, const Vec3& position, std::span<const AABB> boxes);
	[[nodiscard]] AABB movementProbeArea(const AABB& box);
	[[nodiscard]] bool isMovementRangeValid(const AABB& area);

	[[nodiscard]] Vec3 jumpImpulse(Vec3 velocity, float jumpHeight, float yaw, bool sprinting);
	[[nodiscard]] Vec3 moveRelative(Vec3 velocity, const Vec2& impulse, float yaw, float speed);
	void moveRelative(Body& body, float speed);
	[[nodiscard]] float groundAcceleration(float movementSpeed, float groundFriction, float accelerationFriction);
	[[nodiscard]] float effectiveAirSpeed(const Body& body);
	[[nodiscard]] float effectiveGravity(const Body& body, const Vec3& velocity);
	[[nodiscard]] bool sprintMovementBlocked(const Vec3& requested, const Vec3& actual);
	void updateFallDistance(Body& body, float oldY);
	[[nodiscard]] bool attemptKnockback(Body& body);
	void simulateGlide(Body& body);
	[[nodiscard]] Vec3 riptideImpulse(const Body& body, int level, bool wasInWater, bool headInWater);

	void queueStuckSpeedMultiplier(Body& body, const Vec3& multiplier);
	[[nodiscard]] bool applyStuckSpeedMultiplier(Body& body);
	void applyInsideBlockMovement(Body& body, Inside inside);
	[[nodiscard]] bool applyAscendableMovement(Body& body, Traversal traversal, bool leatherBoots);
	[[nodiscard]] bool honeySlideResetsFallDistance(const Body& body, const BlockPos& position);
	void walkOnBlock(Body& body, const BlockMovement& under);
	void landOnBlock(Body& body, const Vec3& previousVelocity, const BlockMovement& under);
	void setPostCollisionMotion(Body& body, const Vec3& previousVelocity, bool previousOnGround, const BlockMovement& under);

	void setSwimmingPoseFlags(Body& body);
	void applyLegacySprint(Body& body, const Input& input);
	void tickState(Body& body, bool advanceTeleport);
	void completeTeleport(Body& body);

	[[nodiscard]] float liquidHeight(const Liquid& liquid);
	[[nodiscard]] int liquidDecay(const Liquid& liquid);
	[[nodiscard]] float liquidGravity(bool swimming, bool water);
	[[nodiscard]] bool liquidIntersects(const AABB& box, const BlockPos& position, const Liquid& liquid);
	[[nodiscard]] AABB shrinkLiquidBox(const AABB& box, const Vec3& offset);
	void applyBubbleColumn(Body& body, BubbleDirection direction, bool surface);

	[[nodiscard]] float movementSpeedWithoutSprint(float value, std::span<const AttributeModifier> modifiers);
	[[nodiscard]] float blockSupportHeight(std::span<const AABB> localBoxes);
}
