#include "helpers.h"
#include "fake_world.h"

#include "bedsim/constants.h"
#include "bedsim/simulator.h"

#include <array>
#include <format>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>
#include <gtest/gtest.h>

using namespace bedsim;
using bedsim::test::BlockPosLess;
using bedsim::test::FakeWorld;
using bedsim::test::baseBody;

struct GroundWorld {
	std::map<BlockPos, std::string, BlockPosLess> blocks;
	std::map<BlockPos, std::vector<AABB>, BlockPosLess> boxes;
	std::vector<BlockPos> order;
	std::optional<BlockMovement> semantics;

	void place(const BlockPos& position, const std::string& name, const float height) {
		blocks[position] = name;
		boxes[position] = { AABB{ {}, { 1.0f, height, 1.0f } } };
	}

	void blockCollisions(const BlockPos& position, std::vector<AABB>& out) const {
		if (const auto found = boxes.find(position); found != boxes.end()) {
			out.insert(out.end(), found->second.begin(), found->second.end());
		}
	}

	void collisions(const AABB& area, std::vector<AABB>& out) const {
		const auto gather = [&](const BlockPos& position, const std::vector<AABB>& local) {
			for (const auto& shape : local) {
				const auto box = shape.translate(Vec3{ position });
				if (strictlyIntersects(area, box)) {
					out.push_back(box);
				}
			}
		};

		if (order.empty()) {
			for (const auto& [position, local] : boxes) {
				gather(position, local);
			}

			return;
		}

		for (const auto& position : order) {
			gather(position, boxes.at(position));
		}
	}

	[[nodiscard]] BlockMovement movement(const BlockPos& position) const {
		if (semantics) {
			return *semantics;
		}

		const auto found = blocks.find(position);
		return vanillaMovement(found != blocks.end() ? found->second : "minecraft:air");
	}

	[[nodiscard]] bool isLoaded(const AABB&) const {
		return true;
	}

	[[nodiscard]] static bool strictlyIntersects(const AABB& first, const AABB& second) {
		for (int axis = 0; axis < 3; ++axis) {
			if (first.max[axis] <= second.min[axis] || second.max[axis] <= first.min[axis]) {
				return false;
			}
		}

		return true;
	}
};

[[nodiscard]] static Body groundBody(const Vec3& position) {
	auto body = baseBody();
	body.position = position;
	body.client.position = position;
	body.onGround = true;
	body.gravity = kGravity;
	return body;
}

TEST(PortGroundProbes, CheckGroundFrictionProbesJustBelowFeet) {
	GroundWorld world;
	world.place({ 0, 0, 0 }, "minecraft:packed_ice", 1.0f);
	world.place({ 0, 1, 0 }, "minecraft:snow_layer", 0.25f);
	Simulator simulator{ world };
	auto body = groundBody({ 0.5f, 1.25f, 0.5f });
	body.velocity = { 0.1f, 0.0f, 0.0f };

	simulator.simulateState(body);
	const float want = 0.1f * (kAirFriction * kBlockFriction);
	EXPECT_TRUE(body.velocity.x == want) << std::format("TestGroundFrictionProbesJustBelowFeet: drag over snow layer on ice {} want {}", body.velocity.x, want);
}

TEST(PortGroundProbes, CheckGroundFrictionIgnoresAirSemantics) {
	const GroundWorld world{ .semantics = BlockMovement{ .groundFriction = 0.98f, .air = true } };
	Simulator simulator{ world };
	auto body = groundBody({ 0.5f, 1.0f, 0.5f });
	body.hasGravity = false;
	body.velocity = { 0.1f, 0.0f, 0.0f };

	simulator.simulateState(body);
	const float want = 0.1f * (kAirFriction * kBlockFriction);
	EXPECT_TRUE(body.velocity.x == want) << std::format("TestGroundFrictionIgnoresAirSemantics: drag over air {} want {}", body.velocity.x, want);
}

TEST(PortGroundProbes, CheckHorizontalDragClearsEpsilonVelocity) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.hasGravity = false;
	body.velocity = { 1e-7f, 0.0f, -1e-7f };

	simulator.simulateState(body);
	EXPECT_TRUE(body.velocity.x == 0.0f && body.velocity.z == 0.0f) << std::format("TestHorizontalDragClearsEpsilonVelocity: epsilon velocity cleared, got {}", body.velocity);
}

TEST(PortGroundProbes, CheckSlimeIgnoresSlowLandings) {
	GroundWorld world;
	world.place({ 0, 0, 0 }, "minecraft:slime", 1.0f);
	Simulator simulator{ world };
	auto body = groundBody({ 0.5f, 1.05f, 0.5f });
	body.onGround = false;
	body.velocity = { 0.0f, -0.07f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(body.velocity.y == -kGravity * kGravityDrag && body.onGround) << std::format("TestSlimeIgnoresSlowLandings: vy={} onGround={}", body.velocity.y, body.onGround);
}

TEST(PortGroundProbes, CheckSneakingPoseSuppressesBounce) {
	GroundWorld world;
	world.place({ 0, 0, 0 }, "minecraft:slime", 1.0f);
	Simulator simulator{ world };
	auto body = groundBody({ 0.5f, 1.2f, 0.5f });
	body.onGround = false;
	body.sneaking = true;
	body.size.y = 1.49f;
	body.velocity = { 0.0f, -0.5f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(body.velocity.y == -kGravity * kGravityDrag) << std::format("TestSneakingPoseSuppressesBounce: vy={}", body.velocity.y);
}

TEST(PortGroundProbes, CheckCarpetOnSlimeBounces) {
	GroundWorld world;
	world.place({ 0, 0, 0 }, "minecraft:slime", 1.0f);
	world.place({ 0, 1, 0 }, "minecraft:white_carpet", 0.0625f);
	Simulator simulator{ world };
	auto body = groundBody({ 0.5f, 1.4f, 0.5f });
	body.onGround = false;
	body.hasGravity = false;
	body.velocity = { 0.0f, -0.5f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(body.velocity.y == 0.5f) << std::format("TestCarpetOnSlimeBounces: vy={}", body.velocity.y);
}

TEST(PortGroundProbes, CheckStandOnDampingFollowsGravityAndDrag) {
	for (const std::string name : { "minecraft:slime", "minecraft:honey_block" }) {
		GroundWorld world;
		world.place({ 0, 0, 0 }, name, 1.0f);
		Simulator simulator{ world };
		auto body = groundBody({ 0.5f, 1.0f, 0.5f });
		body.velocity = { 0.2f, 0.0f, 0.0f };

		simulator.simulateState(body);
		const float vertical = -kGravity * kGravityDrag;
		const float factor = (-vertical * 0.2f) + 0.4f;
		const float want = 0.2f * (kAirFriction * 0.8f) * factor;
		EXPECT_TRUE(body.velocity.x == want && body.velocity.y == vertical) << std::format("TestStandOnDampingFollowsGravityAndDrag/{}: velocity {} want x={} y={}", name, body.velocity, want, vertical);
	}
}

TEST(PortGroundProbes, CheckHoneyJumpFactorBlock) {
	struct Case {
		std::string_view name;
		std::string_view feet;
		float feetY;
		float height;
	};

	constexpr std::array cases{
		Case{ "thin block over honey", "minecraft:test_plate", 1.05f, 1.0f },
		Case{ "carpet over honey", "minecraft:red_carpet", 1.0625f, 0.6f },
		Case{ "honey boundary", "minecraft:air", 1.0f, 0.6f }
	};

	for (const auto& [name, feet, feetY, height] : cases) {
		GroundWorld world;
		world.blocks[{ 0, 0, 0 }] = "minecraft:honey_block";
		world.blocks[{ 0, 1, 0 }] = std::string{ feet };
		Simulator simulator{ world };
		auto body = groundBody({ 0.5f, feetY, 0.5f });
		body.hasGravity = false;
		body.jumping = true;
		body.jumpHeight = kJumpHeight;

		simulator.simulateState(body);
		EXPECT_TRUE(body.velocity.y == kJumpHeight * height) << std::format("TestHoneyJumpFactorBlock/{}: jump velocity {} want {}", name, body.velocity.y, kJumpHeight * height);
	}
}

TEST(PortGroundProbes, CheckSneakEdgeClipsDisplacementOnly) {
	GroundWorld world;
	world.place({ 0, 0, 0 }, "minecraft:stone", 1.0f);
	Simulator simulator{ world };
	auto body = groundBody({ 0.5f, 1.0f, 0.5f });
	body.sneaking = true;
	body.size.y = 1.49f;
	body.hasGravity = false;
	body.velocity = { 1.0f, 0.42f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(body.position.x < 1.3f) << std::format("TestSneakEdgeClipsDisplacementOnly: jump-tick sneak move left supported ground x={}", body.position.x);
	const float want = 1.0f * (kAirFriction * kBlockFriction);
	EXPECT_TRUE(body.velocity.x == want) << std::format("TestSneakEdgeClipsDisplacementOnly: retained velocity {} want {}", body.velocity.x, want);
}

TEST(PortGroundProbes, CheckSmallVerticalClipGrounds) {
	GroundWorld world;
	world.place({ 0, 0, 0 }, "minecraft:stone", 1.0f);
	Simulator simulator{ world };
	auto body = groundBody({ 0.5f, 1.01f, 0.5f });
	body.onGround = false;
	body.hasGravity = false;
	body.velocity = { 0.0f, -0.010003f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(body.onGround && body.collideY) << std::format("TestSmallVerticalClipGrounds: onGround={} collideY={} position={}", body.onGround, body.collideY, body.position);
}

TEST(PortGroundProbes, CheckLandingTieUsesVanillaGatheringOrder) {
	const BlockPos slime{ 0, 0, 0 };
	const BlockPos stone{ 1, 0, 0 };
	for (const auto& order : { std::vector{ slime, stone }, std::vector{ stone, slime } }) {
		GroundWorld world;
		world.order = order;
		world.place(slime, "minecraft:slime", 1.0f);
		world.place(stone, "minecraft:stone", 1.0f);
		Simulator simulator{ world };
		auto body = groundBody({ 1.0f, 1.2f, 0.5f });
		body.onGround = false;
		body.hasGravity = false;
		body.velocity = { 0.0f, -0.5f, 0.0f };

		simulator.simulateState(body);
		EXPECT_TRUE(body.velocity.y == 0.5f) << std::format("TestLandingTieUsesVanillaGatheringOrder: boundary landing vy={} want slime bounce", body.velocity.y);
	}
}
