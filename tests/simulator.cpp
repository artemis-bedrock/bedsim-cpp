#include "helpers.h"
#include "fake_world.h"

#include "bedsim/constants.h"
#include "bedsim/simulator.h"

#include <array>
#include <cmath>
#include <type_traits>
#include <gtest/gtest.h>

using namespace bedsim;
using bedsim::test::BoxWorld;
using bedsim::test::FakeWorld;
using bedsim::test::baseBody;
using bedsim::test::near;

TEST(Simulator, CheckGrassAcceleration) {
	const FakeWorld world{ .fill = "minecraft:grass_block" };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 1.0f, 0.5f };
	body.onGround = true;
	body.hasGravity = false;

	constexpr std::array expected{ 0.05350801f, 0.08272339f, 0.09867499f, 0.10738456f, 0.112139985f };
	for (const float velocity : expected) {
		const auto result = simulator.simulate(body, { .moveVector = { 0.0f, 1.0f } });
		EXPECT_TRUE(result.velocity.z == velocity) << "grass acceleration matches the client exactly";
	}
}

TEST(Simulator, CheckSoulSandAcceleration) {
	const FakeWorld world{ .fill = "minecraft:soul_sand" };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 0.875f, 0.5f };
	body.onGround = true;
	body.hasGravity = false;
	const auto result = simulator.simulate(body, { .moveVector = { 0.0f, 1.0f } });
	EXPECT_TRUE(result.velocity.z == 0.029107885f) << "soul sand acceleration matches the client exactly";
}

TEST(Simulator, CheckAutoStepGroundContact) {
	const BoxWorld world{ .boxes = {
		AABB{ { -2.0f, -1.0f, -2.0f }, { 2.0f, 0.0f, 2.0f } },
		AABB{ { 1.0f, 0.0f, -2.0f }, { 2.0f, 0.5f, 2.0f } }
	} };

	struct Case {
		float velocityY;
		bool grounded;
	};

	for (const auto [velocityY, grounded] : std::array{ Case{ -0.0784f, true }, Case{ 0.0f, false }, Case{ kJumpHeight, false } }) {
		Simulator simulator{ world, predictionOptions() };
		auto body = baseBody();
		body.setPosition({ 0.65f, 0.0f, 0.5f });
		body.setVelocity({ 0.3f, velocityY, 0.0f });
		body.onGround = true;
		body.hasGravity = false;
		simulator.simulateState(body);
		EXPECT_TRUE(near(body.position.y, 0.5f, 1e-5f)) << "half-block auto-step";
		EXPECT_TRUE(body.onGround == grounded) << "ground state after the step follows the requested Y movement";

		body.setVelocity({ 0.1f, -0.0784f, 0.0f });
		simulator.simulateState(body);
		EXPECT_TRUE(body.onGround) << "the following downward collision lands";
	}
}

TEST(Simulator, CheckFallLanding) {
	FakeWorld world;
	world.floor(-1, 2);
	Simulator simulator{ world, predictionOptions() };
	auto body = baseBody();
	body.position = { 0.5f, 10.0f, 0.5f };

	const auto frames = simulator.run(body, {}, 100, true);
	const auto& landing = frames.back();
	EXPECT_TRUE(landing.result.landed && landing.body.onGround) << "lands on the floor";
	EXPECT_TRUE(landing.body.position.y == 0.0f) << "rests on the floor top";

	const float lastAirborneY = frames[frames.size() - 2].body.position.y;
	EXPECT_TRUE(near(landing.result.landingFallDistance, 10.0f - lastAirborneY, 1e-4f)) << "landing fall distance excludes the impact tick";
}

TEST(Simulator, CheckLadder) {
	FakeWorld world;
	world.passable({ 0, 10, 0 }, "minecraft:ladder");
	Simulator simulator{ world, predictionOptions() };
	auto body = baseBody();
	body.position = { 0.5f, 10.2f, 0.5f };
	body.velocity = { 0.0f, -1.5f, 0.0f };
	body.fallDistance = 12.0f;
	simulator.simulate(body, {});
	EXPECT_TRUE(near(body.position.y, 10.2f - kClimbSpeed)) << "ladder caps descent at the climb speed";
	EXPECT_TRUE(body.fallDistance == 0.0f) << "ladder resets fall distance";
}

TEST(Simulator, CheckWaterSlowsFall) {
	FakeWorld world;
	for (int y = 0; y < 5; ++y) {
		world.water({ 0, y, 0 });
	}

	world.solid({ 0, -1, 0 });
	Simulator simulator{ world, predictionOptions() };
	auto body = baseBody();
	body.position = { 0.5f, 3.0f, 0.5f };
	body.velocity = { 0.0f, -1.0f, 0.0f };
	body.fallDistance = 20.0f;
	simulator.simulate(body, {});
	EXPECT_TRUE(near(body.velocity.y, -1.0f * 0.8f - 0.005f)) << "water drag and buoyancy";
	EXPECT_TRUE(body.fallDistance == 0.0f) << "water resets fall distance";
}

TEST(Simulator, CheckUnloaded) {
	FakeWorld world;
	world.loaded = [](const AABB&) {
		return false;
	};

	Simulator simulator{ world, predictionOptions() };
	auto body = baseBody();
	body.position = { 0.0f, 64.0f, 0.0f };
	body.velocity = { 0.3f, -1.0f, 0.0f };
	const auto result = simulator.simulate(body, {});
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "unloaded outcome";
	EXPECT_TRUE((body.position == Vec3{ 0.0f, 64.0f, 0.0f } && body.velocity == Vec3{})) << "unloaded freezes the body";
}

TEST(Simulator, CheckHostileExtremesStayFinite) {
	FakeWorld world;
	world.floor(-1, 2);
	Simulator simulator{ world, predictionOptions() };
	auto body = baseBody();
	body.position = { 1e30f, 1e30f, -1e30f };
	const auto head = simulator.observeHeadLiquid(body);
	EXPECT_TRUE(!head.water && !head.lava) << "a head position beyond the block range observes no liquid instead of converting out of range";

	const Input hostile{ .moveVector = { 0.0f, 1.0f }, .pitch = -1e30f, .yaw = 1e30f, .headYaw = 1e30f };
	const auto unloaded = simulator.simulate(body, hostile);
	EXPECT_TRUE(unloaded.outcome == Outcome::UnloadedChunk && isFinite(body)) << "a position beyond the block range is treated as unloaded";

	body.position = { 0.5f, 0.0f, 0.5f };
	body.onGround = true;
	const auto moved = simulator.simulate(body, hostile);
	EXPECT_TRUE(moved.outcome == Outcome::Normal && isFinite(body)) << "huge finite angles still produce a finite tick";
}

static_assert(std::is_constructible_v<Simulator<FakeWorld>, const FakeWorld&, Options>);
static_assert(!std::is_constructible_v<Simulator<FakeWorld>, FakeWorld, Options>, "a temporary world would dangle, so the simulator rejects it");
