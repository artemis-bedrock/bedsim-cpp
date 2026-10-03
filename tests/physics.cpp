#include "helpers.h"

#include "bedsim/constants.h"
#include "bedsim/physics.h"
#include "bedsim/semantics.h"

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>
#include <gtest/gtest.h>

using namespace bedsim;
using bedsim::test::near;

TEST(Physics, CheckSineTable) {
	EXPECT_TRUE(mcSin(0.0f) == 0.0f) << "sin(0)";
	EXPECT_TRUE(mcCos(0.0f) == 1.0f) << "cos(0)";
	EXPECT_TRUE(near(mcSin(std::numbers::pi_v<float> * 0.5f), 1.0f)) << "sin(pi/2)";
	EXPECT_TRUE(near(mcSin(-std::numbers::pi_v<float> * 0.5f), -1.0f)) << "sin(-pi/2) wraps through the table";
}

TEST(Physics, CheckJumpImpulse) {
	struct Case {
		Vec3 velocity;
		float yaw;
		bool sprint;
		Vec3 expected;
	};

	const std::vector<Case> cases{
		{ { 0.1f, 0.3f, -0.2f }, 0.0f, false, { 0.1f, 0.42f, -0.2f } },
		{ { 0.0f, 0.3f, -0.2f }, 0.0f, true, { 0.0f, 0.42f, 0.0f } },
		{ { 0.2f, 0.3f, 0.0f }, 90.0f, true, { 0.0f, 0.42f, 0.0f } },
		{ { 0.0f, 0.7f, -0.2f }, 0.0f, true, { 0.0f, 0.7f, 0.0f } }
	};

	for (const auto& [velocity, yaw, sprint, expected] : cases) {
		const auto result = jumpImpulse(velocity, kJumpHeight, yaw, sprint);
		EXPECT_TRUE(length(result - expected) <= 1e-5f) << "jump impulse";
	}
}

TEST(Physics, CheckAirSteering) {
	struct Case {
		float yaw;
		Vec3 before;
		Vec3 after;
	};

	const std::vector<Case> cases{
		{ -81.40101f, { 0.3311201f, 0.0f, 0.04030281f }, { 0.32424545f, 0.0f, 0.0401424f } },
		{ -73.75747f, { 0.32312074f, 0.0f, 0.050257172f }, { 0.31630123f, 0.0f, 0.052219465f } }
	};

	for (const auto& [yaw, before, after] : cases) {
		const auto result = moveRelative(before, { 0.0f, 0.98f }, yaw, kSprintAirSpeed) * kAirFriction;
		EXPECT_TRUE(length(result - after) <= 4e-8f) << "vanilla air steering";
	}
}

TEST(Physics, CheckGroundAcceleration) {
	EXPECT_TRUE(near(groundAcceleration(0.1f, kBlockFriction, 1.0f), 0.1f, 1e-6f)) << "grass keeps the attribute speed";
	EXPECT_TRUE(groundAcceleration(0.1f, 0.98f, 1.0f) < 0.1f) << "ice accelerates slower";
}

TEST(Physics, CheckClipCollide) {
	const AABB floor{ { -1.0f, -1.0f, -1.0f }, { 1.0f, 0.0f, 1.0f } };
	const AABB player{ { -0.3f, 0.5f, -0.3f }, { 0.3f, 2.3f, 0.3f } };
	const auto clipped = clipCollide(floor, player, { 0.0f, -2.0f, 0.0f }, false);
	EXPECT_TRUE(near(clipped.velocity.y, -0.5f)) << "falling stops on the floor top";

	const auto apart = clipCollide(floor, player.translate({ 5.0f, 0.0f, 0.0f }), { 0.0f, -2.0f, 0.0f }, false);
	EXPECT_TRUE(apart.velocity.y == -2.0f) << "boxes apart horizontally do not clip";
}

TEST(Physics, CheckFallDistance) {
	Body body{};
	body.position = { 0.0f, 10.0f, 0.0f };
	body.setPosition({ 0.0f, 7.0f, 0.0f });
	updateFallDistance(body, 10.0f);
	EXPECT_TRUE(body.fallDistance == 3.0f) << "falling accumulates";

	body.setPosition({ 0.0f, 8.0f, 0.0f });
	updateFallDistance(body, 7.0f);
	EXPECT_TRUE(body.fallDistance == 0.0f) << "rising resets";

	body.fallDistance = 4.0f;
	body.onGround = true;
	body.setPosition({ 0.0f, 6.0f, 0.0f });
	updateFallDistance(body, 8.0f);
	EXPECT_TRUE(body.fallDistance == 0.0f) << "ground resets";
}

TEST(Physics, CheckSemantics) {
	EXPECT_TRUE(vanillaMovement("minecraft:air").air) << "air";
	EXPECT_TRUE(vanillaMovement("minecraft:ice").groundFriction == 0.98f) << "ice friction";
	EXPECT_TRUE(vanillaMovement("minecraft:blue_ice").groundFriction == 0.989f) << "blue ice friction";
	EXPECT_TRUE(vanillaMovement("minecraft:web").cobweb && !vanillaMovement("minecraft:cobweb").cobweb) << "only the Bedrock web identifier is a cobweb";
	EXPECT_TRUE(vanillaMovement("minecraft:ladder").climbable) << "ladder climbable";
	EXPECT_TRUE(vanillaMovement("minecraft:slime").bounce == Bounce::Slime) << "slime bounce";
	EXPECT_TRUE(vanillaMovement("minecraft:soul_sand").accelerationFriction == kSoulSandAccelerationFriction) << "soul sand";
	EXPECT_TRUE(vanillaMovement("minecraft:oak_fence").fenceLike && vanillaMovement("minecraft:cobblestone_wall").fenceLike) << "fence-like";
	EXPECT_TRUE(vanillaMovement("minecraft:stone").groundFriction == kBlockFriction) << "default friction";
}

TEST(Physics, CheckHugeAnglesStayDefined) {
	EXPECT_TRUE(mcSin(1e30f) == 0.0f) << "an angle whose table index overflows selects entry zero instead of an undefined conversion";
	EXPECT_TRUE(mcCos(-1e30f) == 0.0f) << "the cosine lookup is bounded the same way";
	EXPECT_TRUE(near(mcSin(0.5f), std::sin(0.5f), 1e-4f)) << "in-range angles are unchanged";
}

TEST(Physics, CheckBlockConversionClampsToBlockRange) {
	constexpr int maximum = static_cast<int>(kMaximumBlockCoordinate);
	constexpr int minimum = std::numeric_limits<int>::min();
	EXPECT_TRUE((blockAt({ 1e30f, -1e30f, 0.5f }) == BlockPos{ maximum, minimum, 0 })) << "a huge position clamps to the block range before conversion";
	EXPECT_TRUE((blockCeiling({ 1e30f, 0.25f, -0.5f }) == BlockPos{ maximum, 1, 0 })) << "the ceiling conversion clamps the same way";
	EXPECT_TRUE((blockAt({ 0.5f, -0.5f, 1.0f }) == BlockPos{ 0, -1, 1 })) << "in-range positions floor as before";
}
