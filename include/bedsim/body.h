#pragma once

#include "bedsim/aabb.h"
#include "bedsim/constants.h"
#include "bedsim/types.h"
#include "bedsim/vec.h"

#include <cstdint>
#include <limits>
#include <optional>

namespace bedsim {
	struct ClientState {
		Vec3 position{};
		Vec3 lastPosition{};
		Vec3 velocity{};
		Vec3 lastVelocity{};
		Vec3 movement{};
		Vec3 lastMovement{};
		bool horizontalCollision{};
		bool verticalCollision{};
		bool toggledFly{};
	};

	struct RetainedBox {
		AABB box{};
		Vec3 position{};
		Vec2 dimensions{};
		float yOffset{};
	};

	struct Body {
		ClientState client;

		Vec3 position{};
		Vec3 lastPosition{};
		Vec3 velocity{};
		Vec3 lastVelocity{};
		Vec3 movement{};
		Vec3 lastMovement{};
		Vec3 rotation{};
		Vec3 lastRotation{};

		Vec2 slideOffset{};
		Vec2 impulse{};
		Vec3 size{ kPlayerWidth, kPlayerHeight, 1.0f };
		Vec3 stuckSpeedMultiplier{};
		float standingHeight{};
		float sneakingHeight{};
		float crawlingHeight{};

		std::optional<BlockPos> supportingBlock;
		std::optional<RetainedBox> retainedBox;

		float gravity{};
		float jumpHeight{};
		float jumpStrength{};
		float fallDistance{};
		float movementSpeed{ kDefaultMovementSpeed };
		float defaultMovementSpeed{ kDefaultMovementSpeed };
		float airSpeed{ kWalkAirSpeed };
		float underwaterMovementSpeed{};
		float lavaMovementSpeed{};
		float swimSpeedMultiplier{};
		int64_t dolphinBoostTicks{};
		bool serverUpdatedSpeed{};

		Vec3 knockback{};
		uint64_t ticksSinceKnockback{ 1 };
		bool knockbackPending{};

		Vec3 pendingTeleportPosition{};
		int pendingTeleports{};
		Vec3 teleportPosition{};
		uint64_t ticksSinceTeleport{ 1 };
		uint64_t teleportCompletionTicks{};
		bool teleportSmoothed{};
		bool teleportPending{};

		bool sprinting{};
		bool pressingSprint{};
		bool serverSprint{};
		bool serverSprintApplied{};

		bool sneaking{};
		bool pressingSneak{};
		bool pressingAscend{};
		bool pressingDescend{};

		bool jumping{};
		bool pressingJump{};
		bool effectiveJumping{};
		uint64_t jumpDelay{};

		bool swimming{};
		float swimAmount{};
		bool stoppedSwimmingThisTick{};
		bool swimWaterContact{};
		int64_t swimWaterGraceTicks{};
		bool autoJumpingInWater{};
		bool wantDown{};
		bool wantDownSlow{};

		bool collideX{};
		bool collideY{};
		bool collideZ{};
		bool onGround{};
		bool sprintMovementBlocked{};

		bool penetratedLastFrame{};
		bool stuckInCollider{};

		bool immobile{};
		bool noClip{};

		bool gliding{};
		int64_t glideBoostTicks{};

		bool hasGravity{ true };
		bool slowFalling{};

		bool crawling{};
		int ticksSinceCanSlowdown{};
		int riptideTicks{};
		bool startingSpinAttack{};
		bool riptideReady{};
		bool riptideCollision{};
		bool riptideInRain{};
		bool inVehicle{};

		bool flying{};
		bool mayFly{};
		bool trustFlyStatus{};
		bool justDisabledFlight{};

		int64_t allowedInputs{};
		bool hasFirstInput{};
		int pendingCorrections{};
		bool inCorrectionCooldown{};

		bool ready{ true };
		bool alive{ true };
		GameMode gameMode{ GameMode::Survival };

		void setPosition(const Vec3& value) {
			lastPosition = position;
			position = value;
		}

		void setVelocity(const Vec3& value) {
			lastVelocity = velocity;
			velocity = value;
		}

		void setMovement(const Vec3& value) {
			lastMovement = movement;
			movement = value;
		}

		void setRotation(const Vec3& value) {
			lastRotation = rotation;
			rotation = value;
		}

		[[nodiscard]] bool hasKnockback() const {
			return knockbackPending || (ticksSinceKnockback == 0 && knockback != Vec3{});
		}

		[[nodiscard]] bool hasTeleport() const {
			if (teleportPending || pendingTeleports > 0) {
				return true;
			}

			if (teleportCompletionTicks == 0) {
				return ticksSinceTeleport == 0 && teleportPosition != Vec3{};
			}

			return ticksSinceTeleport <= teleportCompletionTicks;
		}

		[[nodiscard]] int remainingTeleportTicks() const {
			if (!hasTeleport() || ticksSinceTeleport >= teleportCompletionTicks) {
				return 0;
			}

			const auto remaining = teleportCompletionTicks - ticksSinceTeleport;
			constexpr auto limit = static_cast<uint64_t>(std::numeric_limits<int>::max());
			return remaining > limit ? std::numeric_limits<int>::max() : static_cast<int>(remaining);
		}

		void queueKnockback(const Vec3& value) {
			knockback = value;
			knockbackPending = true;
			ticksSinceKnockback = 0;
		}

		void queueTeleport(const Vec3& destination, const bool smoothed, const uint64_t completionTicks) {
			pendingTeleportPosition = destination;
			teleportPosition = destination;
			teleportSmoothed = smoothed;
			teleportCompletionTicks = completionTicks;
			ticksSinceTeleport = 0;
			teleportPending = true;
		}

		void ensurePoseHeights() {
			if (standingHeight <= 0.0f) {
				standingHeight = !sneaking && !crawling && size.y > 0.0f ? size.y : kPlayerHeight;
			}

			if (sneakingHeight <= 0.0f) {
				sneakingHeight = kSneakingHeight;
			}

			if (crawlingHeight <= 0.0f) {
				crawlingHeight = kCrawlingHeight;
			}
		}

		[[nodiscard]] bool swimPose() const {
			return swimming && (swimWaterContact || swimWaterGraceTicks > 0);
		}

		[[nodiscard]] Vec3 eyePosition() const {
			float offset = kEyeHeight;
			if (swimPose() || crawling || gliding || riptideTicks > 0) {
				offset = kCompactEyeHeight;
			} else if (sneaking) {
				offset = kSneakingEyeHeight;
			}

			const float scale = size.z > 0.0f ? size.z : 1.0f;
			return position + Vec3{ 0.0f, offset * scale, 0.0f };
		}

		[[nodiscard]] Vec2 collisionDimensions() const {
			const float scale = size.z;
			float height = size.y * scale;
			if (swimPose() || gliding || riptideTicks > 0) {
				height = size.x * scale;
			}

			return { size.x * 0.5f * scale, height };
		}

		[[nodiscard]] float slideYOffset(const bool useSlideOffset) const {
			return useSlideOffset ? slideOffset.y : 0.0f;
		}

		[[nodiscard]] AABB collisionBoxAt(const Vec3& feet, const bool useSlideOffset) const {
			const auto dimensions = collisionDimensions();
			const float width = dimensions.x;
			const float height = dimensions.y;
			const float offset = slideYOffset(useSlideOffset);
			return AABB::ordered(
				{ feet.x - width, feet.y + offset, feet.z - width },
				{ feet.x + width, feet.y + height + offset, feet.z + width }
			);
		}

		[[nodiscard]] AABB boundingBox(const bool useSlideOffset = false) const {
			if (retainedBox && retainedBox->position == position && retainedBox->dimensions == collisionDimensions() && retainedBox->yOffset == slideYOffset(useSlideOffset)) {
				return retainedBox->box;
			}

			return collisionBoxAt(position, useSlideOffset);
		}

		[[nodiscard]] AABB clientBoundingBox(const bool useSlideOffset = false) const {
			return collisionBoxAt(client.position, useSlideOffset);
		}

		void rememberCollisionBox(const AABB& box, const bool useSlideOffset) {
			retainedBox = RetainedBox{
				.box = box,
				.position = position,
				.dimensions = collisionDimensions(),
				.yOffset = slideYOffset(useSlideOffset)
			};
		}
	};
}
