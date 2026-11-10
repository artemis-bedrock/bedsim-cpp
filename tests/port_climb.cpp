#include "helpers.h"
#include "fake_world.h"

#include "bedsim/constants.h"
#include "bedsim/simulator.h"

#include <map>
#include <set>
#include <string>
#include <vector>
#include <gtest/gtest.h>

using namespace bedsim;
using bedsim::test::BlockPosLess;
using bedsim::test::baseBody;
using bedsim::test::kFullBlock;
using bedsim::test::near;

struct ClimbWorld {
	std::map<BlockPos, std::string, BlockPosLess> blocks;
	std::set<BlockPos, BlockPosLess> solids;
	std::vector<CollisionContext>* contexts{};
	bool leatherBoots{};
	float loadedFloor{ -1.0e9f };

	void blockCollisions(const BlockPos& position, std::vector<AABB>& out) const {
		if (solids.contains(position)) {
			out.insert(out.end(), kFullBlock.begin(), kFullBlock.end());
		}
	}

	void movementCollisions(const AABB& area, const CollisionContext& context, std::vector<AABB>& out) const {
		if (contexts) {
			contexts->push_back(context);
		}

		for (const auto& position : solids) {
			if (const auto box = AABB::unitAt(position); area.intersects(box)) {
				out.push_back(box);
			}
		}
	}

	[[nodiscard]] BlockMovement movement(const BlockPos& position) const {
		if (const auto block = blocks.find(position); block != blocks.end()) {
			return vanillaMovement(block->second);
		}

		return vanillaMovement(solids.contains(position) ? "minecraft:stone" : "minecraft:air");
	}

	[[nodiscard]] bool isLoaded(const AABB& area) const {
		return area.min.y >= loadedFloor;
	}

	[[nodiscard]] int enchantmentLevel(const Enchantment) const {
		return 0;
	}

	[[nodiscard]] bool wearingLeatherBoots() const {
		return leatherBoots;
	}
};

[[nodiscard]] static Body climbingBody(const Vec3& position) {
	auto body = baseBody();
	body.position = position;
	body.client.position = position;
	body.gravity = kGravity;
	return body;
}

TEST(PortClimb, CheckLadderJumpReplacesGroundJump) {
	const ClimbWorld world{
		.blocks = { { BlockPos{ 0, 0, 0 }, "minecraft:ladder" } },
		.solids = { BlockPos{ 0, -1, 0 } }
	};

	Simulator simulator{ world };
	auto body = climbingBody({ 0.5f, 0.0f, 0.5f });
	body.onGround = true;
	body.sprinting = true;
	body.jumping = true;
	body.pressingJump = true;
	body.effectiveJumping = true;

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.y, (kClimbSpeed - kGravity) * kGravityDrag)) << "TestLadderJumpReplacesGroundJump: climb velocity";
	EXPECT_TRUE(body.velocity.x == 0.0f && body.velocity.z == 0.0f) << "TestLadderJumpReplacesGroundJump: no sprint impulse";
	EXPECT_TRUE(body.jumpDelay == 0) << "TestLadderJumpReplacesGroundJump: no jump delay";
}

TEST(PortClimb, CheckScaffoldingJumpReplacesGroundJump) {
	const ClimbWorld world{
		.blocks = { { BlockPos{ 0, 0, 0 }, "minecraft:scaffolding" } },
		.solids = { BlockPos{ 0, -1, 0 } }
	};

	Simulator simulator{ world };
	auto body = climbingBody({ 0.5f, 0.0f, 0.5f });
	body.onGround = true;
	body.sprinting = true;
	body.jumping = true;
	body.pressingJump = true;
	body.effectiveJumping = true;

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.y, (kScaffoldingSpeed - kGravity) * kGravityDrag)) << "TestScaffoldingJumpReplacesGroundJump: scaffolding ascent";
	EXPECT_TRUE(body.velocity.x == 0.0f && body.velocity.z == 0.0f) << "TestScaffoldingJumpReplacesGroundJump: no sprint impulse";
	EXPECT_TRUE(body.jumpDelay == kJumpDelayTicks) << "TestScaffoldingJumpReplacesGroundJump: jump delay";
}

TEST(PortClimb, CheckAutoClimbFollowsThisTicksWallCollision) {
	const ClimbWorld world{
		.blocks = { { BlockPos{ 0, 0, 0 }, "minecraft:ladder" } },
		.solids = { BlockPos{ 1, 0, 0 }, BlockPos{ 1, 1, 0 } }
	};

	Simulator simulator{ world };
	auto body = climbingBody({ 0.65f, 0.0f, 0.5f });
	body.velocity = { 0.1f, 0.0f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(body.collideX) << "TestAutoClimbFollowsThisTicksWallCollision: wall collision";
	EXPECT_TRUE(body.velocity.y == kClimbSpeed) << "TestAutoClimbFollowsThisTicksWallCollision: auto-climb velocity";
}

TEST(PortClimb, CheckPreviousWallCollisionDoesNotLiftClimber) {
	const ClimbWorld world{ .blocks = { { BlockPos{ 0, 0, 0 }, "minecraft:ladder" } } };
	Simulator simulator{ world };
	auto body = climbingBody({ 0.5f, 0.0f, 0.5f });
	body.collideX = true;

	simulator.simulateState(body);
	EXPECT_TRUE(body.position.y == 0.0f) << "TestPreviousWallCollisionDoesNotLiftClimber: stale collision does not move the climber";
	EXPECT_TRUE(near(body.velocity.y, -kGravity * kGravityDrag)) << "TestPreviousWallCollisionDoesNotLiftClimber: vertical velocity";
}

TEST(PortClimb, CheckScaffoldingBridgeOverAirDoesNotDescend) {
	std::vector<CollisionContext> contexts;
	const ClimbWorld world{
		.blocks = { { BlockPos{ 0, 0, 0 }, "minecraft:scaffolding" } },
		.contexts = &contexts
	};

	Simulator simulator{ world };
	auto body = climbingBody({ 0.5f, 1.0f, 0.5f });
	body.onGround = true;
	body.supportingBlock = BlockPos{ 0, 0, 0 };
	body.pressingDescend = true;

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.y, -kGravity * kGravityDrag)) << "TestScaffoldingBridgeOverAirDoesNotDescend: vertical velocity";
	for (const auto& context : contexts) {
		EXPECT_TRUE(!context.descendThroughBlock) << "TestScaffoldingBridgeOverAirDoesNotDescend: no descent through the block";
	}
}

TEST(PortClimb, CheckScaffoldingColumnDescentKeepsDrag) {
	std::vector<CollisionContext> contexts;
	const ClimbWorld world{
		.blocks = { { BlockPos{ 0, 0, 0 }, "minecraft:scaffolding" }, { BlockPos{ 0, 1, 0 }, "minecraft:scaffolding" } },
		.solids = { BlockPos{ 0, -1, 0 } },
		.contexts = &contexts
	};

	Simulator simulator{ world };
	auto body = climbingBody({ 0.5f, 1.5f, 0.5f });
	body.pressingDescend = true;
	body.fallDistance = 4.0f;

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.position.y, 1.35f)) << "TestScaffoldingColumnDescentKeepsDrag: position";
	EXPECT_TRUE(body.velocity.y == -kScaffoldingSpeed * kGravityDrag) << "TestScaffoldingColumnDescentKeepsDrag: descent velocity";
	EXPECT_TRUE(body.fallDistance == 0.0f) << "TestScaffoldingColumnDescentKeepsDrag: fall distance reset";
	EXPECT_TRUE(!contexts.empty() && contexts.back().descendThroughBlock) << "TestScaffoldingColumnDescentKeepsDrag: collision context reports the descent";
}

TEST(PortClimb, CheckPowderSnowDescentRequiresLeatherBoots) {
	const ClimbWorld world{ .blocks = { { BlockPos{ 0, 0, 0 }, "minecraft:powder_snow" } } };
	Simulator simulator{ world };
	auto body = climbingBody({ 0.5f, 0.5f, 0.5f });
	body.pressingDescend = true;

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.y, -kGravity * kGravityDrag)) << "TestPowderSnowDescentRequiresLeatherBoots: vertical velocity";
}

TEST(PortClimb, CheckLeatherBootsDescendIntoPowderSnow) {
	const ClimbWorld world{ .blocks = { { BlockPos{ 0, 0, 0 }, "minecraft:powder_snow" } }, .leatherBoots = true };
	Simulator simulator{ world };
	auto body = climbingBody({ 0.5f, 1.0f, 0.5f });
	body.pressingDescend = true;

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.y, (-kScaffoldingSpeed - kGravity) * kGravityDrag)) << "TestLeatherBootsDescendIntoPowderSnow: vertical velocity";
}

TEST(PortClimb, CheckLeatherBootsPowderSnowJumpAscends) {
	const ClimbWorld world{ .blocks = { { BlockPos{ 0, 0, 0 }, "minecraft:powder_snow" } }, .leatherBoots = true };
	Simulator simulator{ world };
	auto body = climbingBody({ 0.5f, 0.5f, 0.5f });
	body.pressingJump = true;
	body.effectiveJumping = true;

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.y, (kScaffoldingSpeed - kGravity) * kGravityDrag)) << "TestLeatherBootsPowderSnowJumpAscends: vertical velocity";
	EXPECT_TRUE(body.jumpDelay == kJumpDelayTicks) << "TestLeatherBootsPowderSnowJumpAscends: jump delay";
}

TEST(PortClimb, CheckScaffoldingSupportProbeRequiresLoadedArea) {
	const ClimbWorld world{
		.blocks = { { BlockPos{ 0, 0, 0 }, "minecraft:scaffolding" } },
		.loadedFloor = 0.0f
	};

	Simulator simulator{ world };
	auto body = climbingBody({ 0.5f, 1.5f, 0.5f });

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestScaffoldingSupportProbeRequiresLoadedArea: unloaded scaffold support";
}
