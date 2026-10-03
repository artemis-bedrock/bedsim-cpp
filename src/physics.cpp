#include "bedsim/physics.h"

#include "bedsim/constants.h"
#include "bedsim/vec.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>

namespace bedsim {
	static constexpr float kPi = std::numbers::pi_v<float>;
	static constexpr int64_t kMaximumChunkSpan = 256;
	static constexpr int64_t kMaximumBlockSpan = kMaximumChunkSpan << 4;
	static constexpr std::string_view kSprintModifierId = "d208fc00-42aa-4aad-9276-d5446530de43";

	static float tableSin(const int64_t index) {
		const auto entry = static_cast<double>(index & 65535);
		return static_cast<float>(std::sin(entry * std::numbers::pi * 2.0 / 65536.0));
	}

	static int64_t tableIndex(const float scaled) {
		constexpr float limit = 9223372036854775808.0f;
		if (!(scaled >= -limit && scaled < limit)) {
			return std::numeric_limits<int64_t>::min();
		}

		return static_cast<int64_t>(scaled);
	}

	float mcSin(const float radians) {
		return tableSin(tableIndex(radians * 10430.378f));
	}

	float mcCos(const float radians) {
		return tableSin(tableIndex(radians * 10430.378f + 16384.0f));
	}

	bool isFinite(const float value) {
		return std::isfinite(value);
	}

	bool isFinite(const Vec2& value) {
		return isFinite(value.x) && isFinite(value.y);
	}

	bool isFinite(const Vec3& value) {
		return isFinite(value.x) && isFinite(value.y) && isFinite(value.z);
	}

	bool isFinite(const Body& body) {
		const std::array vectors{
			body.client.position, body.client.lastPosition, body.client.velocity, body.client.lastVelocity,
			body.client.movement, body.client.lastMovement, body.position, body.lastPosition, body.velocity,
			body.lastVelocity, body.movement, body.lastMovement, body.rotation, body.lastRotation,
			body.knockback, body.teleportPosition, body.pendingTeleportPosition, body.size,
			body.stuckSpeedMultiplier
		};

		const std::array scalars{
			body.standingHeight, body.sneakingHeight, body.crawlingHeight,
			body.gravity, body.jumpHeight, body.jumpStrength, body.fallDistance,
			body.movementSpeed, body.defaultMovementSpeed, body.airSpeed,
			body.underwaterMovementSpeed, body.lavaMovementSpeed, body.swimSpeedMultiplier,
			body.swimAmount
		};

		const bool vectorsFinite = std::ranges::all_of(vectors, [](const Vec3& value) {
			return isFinite(value);
		});

		const bool scalarsFinite = std::ranges::all_of(scalars, [](const float value) {
			return isFinite(value);
		});

		return vectorsFinite && scalarsFinite && isFinite(body.slideOffset) && isFinite(body.impulse);
	}

	bool isFinite(const Input& input) {
		if (const auto modifier = input.itemUseMovementModifier; modifier && (!isFinite(*modifier) || *modifier < 0.0f || *modifier > 1.0f)) {
			return false;
		}

		return isFinite(input.moveVector) && isFinite(input.clientPosition) && isFinite(input.clientVelocity) &&
			isFinite(input.pitch) && isFinite(input.yaw) && isFinite(input.headYaw);
	}

	Clip clipCollide(const AABB& stationary, const AABB& moving, const Vec3& velocity, const bool oneWay, const Vec3& penetration) {
		Clip clip{ .velocity = velocity, .penetration = penetration };
		if (stationary.hasZeroVolume()) {
			return clip;
		}

		Vec3 axisPenetration{};
		Vec3 signedPenetration{};
		Vec3 normal{};
		int separatingAxes = 0;
		int separatingAxis = 0;

		for (int axis = 0; axis < 3; ++axis) {
			float minPenetration = moving.max[axis] - stationary.min[axis];
			float maxPenetration = stationary.max[axis] - moving.min[axis];
			if (std::abs(minPenetration) <= kContactEpsilon) {
				minPenetration = 0.0f;
			}

			if (std::abs(maxPenetration) <= kContactEpsilon) {
				maxPenetration = 0.0f;
			}

			const float minPositive = std::max(0.0f, minPenetration);
			const float maxPositive = std::max(0.0f, maxPenetration);
			if (minPositive == 0.0f) {
				signedPenetration[axis] = minPenetration;
				normal[axis] = -1.0f;
				++separatingAxes;
				separatingAxis = axis;
			} else if (maxPositive == 0.0f) {
				signedPenetration[axis] = maxPenetration;
				normal[axis] = 1.0f;
				++separatingAxes;
				separatingAxis = axis;
			} else if (minPositive < maxPositive) {
				axisPenetration[axis] = minPositive;
				signedPenetration[axis] = minPositive;
				normal[axis] = -1.0f;
			} else {
				axisPenetration[axis] = maxPositive;
				signedPenetration[axis] = maxPositive;
				normal[axis] = 1.0f;
			}

			if (separatingAxes > 1) {
				return clip;
			}
		}

		if (separatingAxes == 0) {
			int bestAxis = 0;
			for (int axis = 1; axis < 3; ++axis) {
				if (axisPenetration[axis] < axisPenetration[bestAxis]) {
					bestAxis = axis;
				}
			}

			clip.penetration[bestAxis] = std::max(clip.penetration[bestAxis], axisPenetration[bestAxis]);
			if (oneWay) {
				return clip;
			}

			const float desired = axisPenetration[bestAxis] * normal[bestAxis];
			clip.velocity[bestAxis] = desired > 0.0f ? std::max(desired, velocity[bestAxis]) : std::min(desired, velocity[bestAxis]);
			return clip;
		}

		const float swept = signedPenetration[separatingAxis] - normal[separatingAxis] * velocity[separatingAxis];
		if (swept <= 0.0f) {
			return clip;
		}

		clip.velocity[separatingAxis] = signedPenetration[separatingAxis] * normal[separatingAxis];
		return clip;
	}

	Clip clipAll(const std::span<const AABB> boxes, const AABB& moving, const Vec3& velocity, const bool oneWay, const Vec3& penetration) {
		Clip clip{ .velocity = velocity, .penetration = penetration };
		for (auto box = boxes.rbegin(); box != boxes.rend(); ++box) {
			clip = clipCollide(*box, moving, clip.velocity, oneWay, clip.penetration);
		}

		return clip;
	}

	StepResult autoStep(const AABB& box, const Vec3& velocity, const std::span<const AABB> boxes, const bool oneWay) {
		int boxCount = 0;
		const auto clipStepBoxes = [&](const AABB& moving, const Vec3& movement, const bool count) {
			Vec3 result = movement;
			for (auto other = boxes.rbegin(); other != boxes.rend(); ++other) {
				if (other->min.y >= box.max.y) {
					continue;
				}

				if (count) {
					++boxCount;
				}

				result = clipCollide(*other, moving, result, oneWay).velocity;
			}

			return result;
		};

		AABB stepped = box;
		const auto up = clipStepBoxes(stepped, { 0.0f, kStepHeight, 0.0f }, true);
		stepped = stepped.translate(up);
		const auto x = clipStepBoxes(stepped, { velocity.x, 0.0f, 0.0f }, false);
		stepped = stepped.translate(x);
		const auto z = clipStepBoxes(stepped, { 0.0f, 0.0f, velocity.z }, false);
		stepped = stepped.translate(z);
		const auto down = clipStepBoxes(stepped, up * -1.0f, false);
		stepped = stepped.translate(down);

		return {
			.box = stepped,
			.velocity = up + x + z + down,
			.up = up,
			.x = x,
			.z = z,
			.down = down,
			.boxCount = boxCount
		};
	}

	AABB recoverRoundedContacts(const AABB& box, const Vec3& position, const std::span<const AABB> boxes) {
		const auto originalMin = box.min;
		const auto originalMax = box.max;
		auto low = originalMin;
		auto high = originalMax;
		constexpr std::array horizontal{ 0, 2 };
		for (const auto& other : boxes) {
			if (!box.intersects(other)) {
				continue;
			}

			for (const int axis : horizontal) {
				const float maxFace = other.max[axis];
				if (maxFace > originalMin[axis] && maxFace < originalMax[axis] && (maxFace + originalMax[axis]) * 0.5f == position[axis]) {
					low[axis] = std::max(low[axis], maxFace);
				}

				const float minFace = other.min[axis];
				if (minFace < originalMax[axis] && minFace > originalMin[axis] && (originalMin[axis] + minFace) * 0.5f == position[axis]) {
					high[axis] = std::min(high[axis], minFace);
				}
			}
		}

		for (const int axis : horizontal) {
			if (low[axis] >= high[axis] || (low[axis] + high[axis]) * 0.5f != position[axis]) {
				low[axis] = originalMin[axis];
				high[axis] = originalMax[axis];
			}
		}

		return AABB::ordered(low, high);
	}

	AABB movementProbeArea(const AABB& box) {
		const auto grown = box.grow(1.0f);
		return AABB::ordered(floor(grown.min), ceil(grown.max) + Vec3{ 1.0f });
	}

	bool isMovementRangeValid(const AABB& area) {
		const auto minimum = floor(area.min);
		const auto maximum = ceil(area.max) - Vec3{ 1.0f };
		for (const float value : { minimum.x, minimum.y, minimum.z, maximum.x, maximum.y, maximum.z }) {
			if (!isFinite(value) || value < kMinimumBlockCoordinate || value > kMaximumBlockCoordinate) {
				return false;
			}
		}

		const auto minimumChunkX = static_cast<int64_t>(static_cast<int32_t>(minimum.x) >> 4);
		const auto minimumChunkZ = static_cast<int64_t>(static_cast<int32_t>(minimum.z) >> 4);
		const auto maximumChunkX = static_cast<int64_t>(static_cast<int32_t>(maximum.x) >> 4);
		const auto maximumChunkZ = static_cast<int64_t>(static_cast<int32_t>(maximum.z) >> 4);
		const int64_t spanX = maximumChunkX - minimumChunkX + 1;
		const int64_t spanZ = maximumChunkZ - minimumChunkZ + 1;
		const int64_t spanY = static_cast<int64_t>(static_cast<int32_t>(maximum.y)) - static_cast<int64_t>(static_cast<int32_t>(minimum.y)) + 1;
		return spanX > 0 && spanZ > 0 && spanY > 0 && spanX <= kMaximumChunkSpan && spanZ <= kMaximumChunkSpan && spanY <= kMaximumBlockSpan;
	}

	Vec3 jumpImpulse(Vec3 velocity, const float jumpHeight, const float yaw, const bool sprinting) {
		velocity.y = std::max(jumpHeight, velocity.y);
		if (sprinting) {
			const float radians = yaw * 0.017453292f;
			velocity.x -= mcSin(radians) * kSprintJumpBoost;
			velocity.z += mcCos(radians) * kSprintJumpBoost;
		}

		return velocity;
	}

	Vec3 moveRelative(Vec3 velocity, const Vec2& impulse, const float yaw, const float speed) {
		const float lengthSquared = impulse.y * impulse.y + impulse.x * impulse.x;
		if (lengthSquared < 1e-4f) {
			return velocity;
		}

		const float force = speed / std::max(std::sqrt(lengthSquared), 1.0f);
		const float forward = impulse.y * force;
		const float strafe = impulse.x * force;
		const float radians = yaw * (kPi / 180.0f);
		const float sine = std::sin(radians);
		const float cosine = std::cos(radians);

		velocity.x += strafe * cosine - sine * impulse.y * force;
		velocity.z += strafe * sine + forward * cosine;
		return velocity;
	}

	void moveRelative(Body& body, const float speed) {
		const auto& impulse = body.impulse;
		if (impulse.y * impulse.y + impulse.x * impulse.x < 1e-4f) {
			return;
		}

		body.setVelocity(moveRelative(body.velocity, impulse, body.rotation.z, speed));
	}

	float groundAcceleration(const float movementSpeed, const float groundFriction, const float accelerationFriction) {
		const float friction = groundFriction * accelerationFriction * kAirFriction;
		const float baseFriction = kAirFriction * kBlockFriction;
		const float ratio = baseFriction / friction;
		return movementSpeed * ratio * ratio * ratio;
	}

	float effectiveAirSpeed(const Body& body) {
		return body.sprinting ? kSprintAirSpeed : kWalkAirSpeed;
	}

	float effectiveGravity(const Body& body, const Vec3& velocity) {
		return body.slowFalling && velocity.y < 0.0f ? kSlowFallingGravity : body.gravity;
	}

	bool sprintMovementBlocked(const Vec3& requested, const Vec3& actual) {
		constexpr float minimumMovement = 0.00005f;
		const float x = std::abs(requested.x);
		const float z = std::abs(requested.z);
		return (x > z && std::abs(actual.x) < minimumMovement) || (z > x && std::abs(actual.z) < minimumMovement);
	}

	void updateFallDistance(Body& body, const float oldY) {
		const float delta = body.position.y - oldY;
		if (delta < 0.0f && !body.onGround) {
			body.fallDistance -= delta;
		} else if (delta > 0.0f) {
			body.fallDistance = 0.0f;
		}

		if (body.onGround && body.fallDistance > 0.0f) {
			body.fallDistance = 0.0f;
		}
	}

	bool attemptKnockback(Body& body) {
		if (!body.hasKnockback()) {
			return false;
		}

		body.setVelocity(body.knockback);
		return true;
	}

	void simulateGlide(Body& body) {
		if (body.velocity.y > kGlideFallDistanceVelocity) {
			body.fallDistance = 1.0f;
		}

		constexpr float radians = kPi / 180.0f;
		const float yaw = body.rotation.z * radians;
		const float pitch = body.rotation.x * radians;
		const float yawCos = mcCos(-yaw - kPi);
		const float yawSin = mcSin(-yaw - kPi);
		const float pitchCos = mcCos(pitch);
		const float pitchSin = mcSin(pitch);

		const float lookX = yawSin * -pitchCos;
		const float lookY = -pitchSin;
		const float lookZ = yawCos * -pitchCos;

		auto velocity = body.velocity;
		const float horizontalSpeed = std::sqrt(velocity.x * velocity.x + velocity.z * velocity.z);
		const float lookHorizontal = pitchCos;
		const float pitchCosSquared = pitchCos * pitchCos;

		const float gravity = body.slowFalling ? kSlowFallingGravity : body.gravity;
		velocity.y += -gravity + pitchCosSquared * (gravity * 0.75f);
		if (velocity.y < 0.0f && lookHorizontal > kGlideLookEpsilon) {
			const float lift = velocity.y * -0.1f * pitchCosSquared;
			velocity.y += lift;
			velocity.x += lookX * lift / lookHorizontal;
			velocity.z += lookZ * lift / lookHorizontal;
		}

		if (pitch < 0.0f && lookHorizontal > kGlideLookEpsilon) {
			const float climb = horizontalSpeed * -pitchSin * 0.04f;
			velocity.y += climb * 3.2f;
			velocity.x -= lookX * climb / lookHorizontal;
			velocity.z -= lookZ * climb / lookHorizontal;
		}

		if (lookHorizontal > kGlideLookEpsilon) {
			velocity.x += (lookX / lookHorizontal * horizontalSpeed - velocity.x) * 0.1f;
			velocity.z += (lookZ / lookHorizontal * horizontalSpeed - velocity.z) * 0.1f;
		}

		if (body.glideBoostTicks > 0) {
			velocity.x += (lookX * 0.1f) + (((lookX * 1.5f) - velocity.x) * 0.5f);
			velocity.y += (lookY * 0.1f) + (((lookY * 1.5f) - velocity.y) * 0.5f);
			velocity.z += (lookZ * 0.1f) + (((lookZ * 1.5f) - velocity.z) * 0.5f);
		}

		velocity.x *= 0.99f;
		velocity.y *= 0.98f;
		velocity.z *= 0.99f;
		body.setVelocity(velocity);
	}

	Vec3 riptideImpulse(const Body& body, const int level, const bool wasInWater, const bool headInWater) {
		const float force = 0.75f * static_cast<float>(level + 1);
		const float pitch = body.rotation.x * kPi / 180.0f;
		const float yaw = body.rotation.z * kPi / 180.0f;
		Vec3 direction{ -mcSin(yaw) * mcCos(pitch), -mcSin(pitch), mcCos(yaw) * mcCos(pitch) };
		if (const float magnitude = length(direction); magnitude > 0.0f) {
			direction *= force / magnitude;
		}

		if (body.onGround && body.hasGravity) {
			if (wasInWater && !headInWater) {
				direction.y = direction.y / kWaterDrag * kGravityDrag;
			} else {
				direction.y += body.gravity;
			}
		}

		return direction;
	}

	void queueStuckSpeedMultiplier(Body& body, const Vec3& multiplier) {
		if (dot(body.stuckSpeedMultiplier, body.stuckSpeedMultiplier) <= 1e-7f) {
			body.stuckSpeedMultiplier = multiplier;
			return;
		}

		body.stuckSpeedMultiplier = bedsim::min(body.stuckSpeedMultiplier, multiplier);
	}

	bool applyStuckSpeedMultiplier(Body& body) {
		const auto multiplier = body.stuckSpeedMultiplier;
		if (dot(multiplier, multiplier) <= 1e-7f) {
			return false;
		}

		if (body.noClip) {
			body.stuckSpeedMultiplier = {};
			return false;
		}

		body.setVelocity(body.velocity * multiplier);
		body.stuckSpeedMultiplier = {};
		return true;
	}

	void applyInsideBlockMovement(Body& body, const Inside inside) {
		if (inside == Inside::SweetBerryBush) {
			queueStuckSpeedMultiplier(body, { 0.8f, 0.75f, 0.8f });
		} else if (inside == Inside::PowderSnow) {
			queueStuckSpeedMultiplier(body, { 0.9f, 1.5f, 0.9f });
		}
	}

	bool applyAscendableMovement(Body& body, const Traversal traversal, const bool leatherBoots) {
		auto velocity = body.velocity;
		if (traversal == Traversal::Scaffolding) {
			if (body.pressingDescend) {
				velocity.y = -0.15f;
				body.setVelocity(velocity);
				return true;
			}

			if (body.pressingAscend) {
				velocity.y = 0.15f;
			}
		} else if (traversal == Traversal::PowderSnow) {
			if (body.pressingDescend) {
				velocity.y = -0.15f;
			} else if (body.pressingAscend && leatherBoots) {
				velocity.y = 0.2f;
			}
		}

		body.setVelocity(velocity);
		return false;
	}

	bool honeySlideResetsFallDistance(const Body& body, const BlockPos& position) {
		if (body.velocity.y >= 0.0f || body.position.y > static_cast<float>(position.y) + 0.9375f) {
			return false;
		}

		const float radius = body.size.x * body.size.z * 0.5f + 0.43125f;
		const float centerX = static_cast<float>(position.x) + 0.5f;
		const float centerZ = static_cast<float>(position.z) + 0.5f;
		return std::abs(centerX - body.position.x) > radius || std::abs(centerZ - body.position.z) > radius;
	}

	void walkOnBlock(Body& body, const BlockMovement& under) {
		if (!body.onGround || body.sneaking) {
			return;
		}

		auto velocity = body.velocity;
		if (under.bounce == Bounce::Slime || under.honey) {
			const float vertical = std::abs(velocity.y);
			if (vertical < 0.1f && !body.pressingSneak) {
				const float factor = 0.4f + vertical * 0.2f;
				velocity.x *= factor;
				velocity.z *= factor;
			}
		}

		body.setVelocity(velocity);
	}

	void landOnBlock(Body& body, const Vec3& previousVelocity, const BlockMovement& under) {
		auto velocity = body.velocity;
		if (previousVelocity.y >= 0.0f || body.pressingSneak) {
			velocity.y = 0.0f;
			body.setVelocity(velocity);
			return;
		}

		switch (under.bounce) {
		case Bounce::Slime:
			velocity.y = kSlimeBounce * previousVelocity.y;
			if (std::abs(velocity.y) < 1e-4f) {
				velocity.y = 0.0f;
			}
			break;
		case Bounce::Bed:
			velocity.y = kBedBounce * previousVelocity.y;
			break;
		case Bounce::None:
		default:
			velocity.y = 0.0f;
			break;
		}

		body.setVelocity(velocity);
	}

	void setPostCollisionMotion(Body& body, const Vec3& previousVelocity, const bool previousOnGround, const BlockMovement& under) {
		if (!previousOnGround && body.collideY) {
			landOnBlock(body, previousVelocity, under);
		} else if (body.collideY) {
			auto velocity = body.velocity;
			velocity.y = 0.0f;
			body.setVelocity(velocity);
		}

		auto velocity = body.velocity;
		if (body.collideX) {
			velocity.x = 0.0f;
		}

		if (body.collideZ) {
			velocity.z = 0.0f;
		}

		body.setVelocity(velocity);
	}

	void setSwimmingPoseFlags(Body& body) {
		body.sneaking = false;
		body.crawling = false;
		body.size.y = body.standingHeight;
	}

	void applyLegacySprint(Body& body, const Input& input) {
		bool adjustSpeed = false;
		if (input.startSprinting && input.stopSprinting) {
			body.sprinting = false;
			adjustSpeed = true;
		} else if (input.startSprinting) {
			body.sprinting = true;
			adjustSpeed = true;
		} else if (input.stopSprinting) {
			body.sprinting = false;
			adjustSpeed = !body.serverUpdatedSpeed;
		}

		if (adjustSpeed) {
			body.serverUpdatedSpeed = false;
			body.movementSpeed = body.defaultMovementSpeed;
			if (body.sprinting) {
				body.movementSpeed *= kSprintSpeedMultiplier;
			}
		}

		body.airSpeed = effectiveAirSpeed(body);
	}

	void tickState(Body& body, const bool advanceTeleport) {
		if (body.glideBoostTicks > 0) {
			--body.glideBoostTicks;
		}

		if (body.dolphinBoostTicks > 0) {
			--body.dolphinBoostTicks;
			if (body.dolphinBoostTicks <= 0) {
				body.dolphinBoostTicks = 0;
				body.swimSpeedMultiplier = kSwimSpeedMultiplier;
			}
		}

		++body.ticksSinceKnockback;
		body.knockbackPending = false;
		if (advanceTeleport && body.ticksSinceTeleport < std::numeric_limits<uint64_t>::max()) {
			++body.ticksSinceTeleport;
		}

		if (body.jumpDelay > 0) {
			--body.jumpDelay;
		}

		if (body.riptideTicks > 0) {
			--body.riptideTicks;
			if (body.riptideTicks == 0) {
				body.riptideCollision = false;
			}
		}

		body.justDisabledFlight = false;
		body.stoppedSwimmingThisTick = false;
	}

	void completeTeleport(Body& body) {
		if (body.teleportCompletionTicks == std::numeric_limits<uint64_t>::max()) {
			body.teleportCompletionTicks = 0;
			body.ticksSinceTeleport = 1;
			return;
		}

		body.ticksSinceTeleport = body.teleportCompletionTicks + 1;
	}

	float liquidHeight(const Liquid& liquid) {
		return liquid.falling ? 1.0f : static_cast<float>(liquid.depth + 1) / 9.0f;
	}

	int liquidDecay(const Liquid& liquid) {
		return liquid.falling ? 0 : 8 - liquid.depth;
	}

	float liquidGravity(const bool swimming, const bool water) {
		if (!water) {
			return 0.02f;
		}

		return swimming ? 0.0f : 0.005f;
	}

	bool liquidIntersects(const AABB& box, const BlockPos& position, const Liquid& liquid) {
		const float surface = static_cast<float>(position.y) + liquidHeight(liquid);
		return box.max.y > static_cast<float>(position.y) && box.min.y < surface;
	}

	AABB shrinkLiquidBox(const AABB& box, const Vec3& offset) {
		auto minimum = box.min + offset;
		auto maximum = box.max - offset;
		for (int axis = 0; axis < 3; ++axis) {
			if (minimum[axis] > maximum[axis]) {
				const float middle = (box.min[axis] + box.max[axis]) * 0.5f;
				minimum[axis] = middle;
				maximum[axis] = middle;
			}
		}

		return AABB::ordered(minimum, maximum);
	}

	void applyBubbleColumn(Body& body, const BubbleDirection direction, const bool surface) {
		auto velocity = body.velocity;
		if (direction == BubbleDirection::Down) {
			const float cap = surface ? -0.9f : -0.3f;
			velocity.y = std::max(cap, velocity.y - 0.03f);
		} else {
			const float change = surface ? 0.1f : 0.06f;
			const float cap = surface ? 1.8f : 0.7f;
			velocity.y = std::min(cap, velocity.y + change);
		}

		body.setVelocity(velocity);
	}

	[[nodiscard]] static bool equalsIgnoringCase(const std::string_view left, const std::string_view right) {
		return std::ranges::equal(left, right, [](const char first, const char second) {
			return std::tolower(static_cast<unsigned char>(first)) == std::tolower(static_cast<unsigned char>(second));
		});
	}

	float movementSpeedWithoutSprint(const float value, const std::span<const AttributeModifier> modifiers) {
		for (const auto& modifier : modifiers) {
			if (equalsIgnoringCase(modifier.id, kSprintModifierId) && modifier.operation == ModifierOperation::MultiplyTotal && modifier.operand == ModifierOperand::Current) {
				return value / (1.0f + modifier.amount);
			}
		}

		return value;
	}

	float blockSupportHeight(const std::span<const AABB> localBoxes) {
		float top = -1.0f;
		for (const auto& box : localBoxes) {
			if (box.min.x <= 0.5f && box.max.x >= 0.5f && box.min.z <= 0.5f && box.max.z >= 0.5f) {
				top = std::max(top, box.max.y);
			}
		}

		return top >= 0.0f ? top : 1.0f;
	}
}
