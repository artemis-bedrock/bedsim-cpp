#include "helpers.h"
#include "fake_world.h"

#include "bedsim/constants.h"
#include "bedsim/physics.h"
#include "bedsim/simulator.h"
#include "bedsim/systems/systems.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <gtest/gtest.h>

namespace bedsim::test {
	struct PlainWorld {
		FakeWorld blocks;

		void blockCollisions(const BlockPos& position, std::vector<AABB>& out) const {
			blocks.blockCollisions(position, out);
		}

		[[nodiscard]] BlockMovement movement(const BlockPos& position) const {
			return blocks.movement(position);
		}

		[[nodiscard]] bool isLoaded(const AABB& area) const {
			return blocks.isLoaded(area);
		}
	};

	struct LayeredWorld {
		FakeWorld blocks;
		std::map<BlockPos, Liquid, BlockPosLess> layer;

		LayeredWorld& waterlog(const BlockPos& position, FakeBlock block, const Liquid& liquid) {
			blocks.set(position, std::move(block));
			layer[position] = liquid;
			return *this;
		}

		void blockCollisions(const BlockPos& position, std::vector<AABB>& out) const {
			blocks.blockCollisions(position, out);
		}

		[[nodiscard]] BlockMovement movement(const BlockPos& position) const {
			return blocks.movement(position);
		}

		[[nodiscard]] bool isLoaded(const AABB& area) const {
			return blocks.isLoaded(area);
		}

		[[nodiscard]] std::optional<Liquid> liquid(const BlockPos& position) const {
			if (const auto found = layer.find(position); found != layer.end()) {
				return found->second;
			}

			return blocks.liquid(position);
		}
	};

	struct FlowSemanticsWorld : FakeWorld {
		[[nodiscard]] bool liquidFaceClosed(const BlockPos& position, const Face) const {
			const auto* block = find(position);
			return block && std::ranges::any_of(block->shapes, [](const AABB& box) {
				return box == kFullBlock.front();
			});
		}

		[[nodiscard]] bool liquidFlowBarrier(const BlockPos&) const {
			return false;
		}
	};

	struct ExactBubbleSurfaceWorld : FakeWorld {
		bool surface{};

		[[nodiscard]] std::optional<bool> bubbleColumnSurface(const BlockPos&) const {
			return surface;
		}
	};

	static_assert(World<PlainWorld> && !HasLiquids<PlainWorld>);
	static_assert(World<LayeredWorld> && HasLiquids<LayeredWorld> && !HasLiquidFlow<LayeredWorld>);
	static_assert(World<FlowSemanticsWorld> && HasLiquidFlow<FlowSemanticsWorld>);
	static_assert(World<ExactBubbleSurfaceWorld> && HasBubbleColumnSurface<ExactBubbleSurfaceWorld>);
}

using namespace bedsim;
using bedsim::test::BoxWorld;
using bedsim::test::ExactBubbleSurfaceWorld;
using bedsim::test::FakeBlock;
using bedsim::test::FakeWorld;
using bedsim::test::FlowSemanticsWorld;
using bedsim::test::LayeredWorld;
using bedsim::test::PlainWorld;
using bedsim::test::baseBody;
using bedsim::test::kFullBlock;
using bedsim::test::near;

[[nodiscard]] static FakeBlock waterBlock(const int depth = 8, const bool falling = false) {
	return {
		.name = "minecraft:water",
		.liquid = Liquid{ .kind = LiquidKind::Water, .depth = depth, .falling = falling }
	};
}

[[nodiscard]] static FakeBlock lavaBlock() {
	return {
		.name = "minecraft:lava",
		.liquid = Liquid{ .kind = LiquidKind::Lava }
	};
}

[[nodiscard]] static FakeBlock stoneBlock() {
	return { .name = "minecraft:stone", .shapes = kFullBlock };
}

[[nodiscard]] static FakeBlock stairsBlock(const Face facing) {
	const AABB upper = facing == Face::East ? AABB{ { 0.5f, 0.5f, 0.0f }, { 1.0f, 1.0f, 1.0f } } : AABB{ { 0.0f, 0.5f, 0.0f }, { 0.5f, 1.0f, 1.0f } };
	return {
		.name = "minecraft:oak_stairs",
		.shapes = { AABB{ {}, { 1.0f, 0.5f, 1.0f } }, upper }
	};
}

static void fill(FakeWorld& world, const BlockPos& minimum, const BlockPos& maximum, const FakeBlock& block) {
	for (int x = minimum.x; x <= maximum.x; ++x) {
		for (int y = minimum.y; y <= maximum.y; ++y) {
			for (int z = minimum.z; z <= maximum.z; ++z) {
				world.set({ x, y, z }, block);
			}
		}
	}
}

[[nodiscard]] static FakeWorld filledColumn(const FakeBlock& block) {
	FakeWorld world;
	fill(world, { -2, 0, -2 }, { 2, 3, 2 }, block);
	return world;
}

[[nodiscard]] static Options liquidOptions() {
	return { .positionCorrectionThreshold = 0.3f };
}

[[nodiscard]] static Body submergedBody() {
	auto body = baseBody();
	body.position = { 0.5f, 0.5f, 0.5f };
	body.client.position = body.position;
	return body;
}

[[nodiscard]] static Body dryBody() {
	auto body = submergedBody();
	body.gravity = kGravity;
	return body;
}

[[nodiscard]] static bool approxEqual(const float value, const float expected) {
	return std::abs(value - expected) < 1e-6f;
}

[[nodiscard]] static bool approxVec(const Vec3& value, const Vec3& expected) {
	return approxEqual(value.x, expected.x) && approxEqual(value.y, expected.y) && approxEqual(value.z, expected.z);
}

[[nodiscard]] static Vec3 normalize(const Vec3& value) {
	return value * (1.0f / length(value));
}

template <typename W>
[[nodiscard]] static Vec3 liquidFlow(const W& world, const BlockPos& position) {
	const Options options{};
	Scratch scratch;
	systems::Systems<W> systems{ world, options, scratch };
	auto body = baseBody();
	const std::array positions{ position };
	const bool loaded = systems.liquids.applyFlow(body, positions, LiquidKind::Water);
	EXPECT_TRUE(loaded) << "liquid flow area is loaded";
	return body.velocity / 0.014f;
}

template <typename W>
[[nodiscard]] static size_t touchingWater(const W& world, const Body& body) {
	const Options options{};
	Scratch scratch;
	const systems::Systems<W> systems{ world, options, scratch };
	std::vector<BlockPos> positions;
	systems.liquids.touchingBlocks(body, LiquidKind::Water, positions);
	return positions.size();
}

TEST(PortLiquidHardening, CheckSpoofedSwimmingInAirStopsHovering) {
	const FakeWorld world;
	Simulator simulator{ world, liquidOptions() };
	auto body = dryBody();
	body.swimming = true;

	const float startY = body.position.y;
	for (int tick = 0; tick < 60; ++tick) {
		simulator.simulate(body, {});
	}

	EXPECT_TRUE(body.velocity.y <= -0.5f) << "TestSpoofedSwimmingInAirStopsHovering: clear fall after the grace expires";
	EXPECT_TRUE(body.position.y < startY) << "TestSpoofedSwimmingInAirStopsHovering: position falls below the start";
}

TEST(PortLiquidHardening, CheckSwimmingPreservesWaterTravelAcrossSurfaceTransition) {
	const FakeWorld world;
	Simulator simulator{ world, liquidOptions() };
	auto body = dryBody();
	body.swimming = true;
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;

	simulator.simulateState(body);
	EXPECT_TRUE(approxEqual(body.velocity.y, 0.0f)) << "TestSwimmingPreservesWaterTravelAcrossSurfaceTransition: water travel inside the grace window";
}

TEST(PortLiquidHardening, CheckSwimWaterGraceRefillsOnWaterContact) {
	const auto world = filledColumn(waterBlock());
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = true;
	body.swimWaterGraceTicks = 1;

	simulator.simulateState(body);
	EXPECT_TRUE(body.swimWaterGraceTicks == kSwimWaterGraceTicks) << "TestSwimWaterGraceRefillsOnWaterContact: grace refilled";
}

TEST(PortLiquidHardening, CheckSwimWaterGraceExpiresDeterministically) {
	const FakeWorld world;
	auto options = liquidOptions();
	options.swimWaterGraceTicks = 3;
	Simulator simulator{ world, options };
	auto body = dryBody();
	body.swimming = true;
	body.swimWaterGraceTicks = 3;

	for (int tick = 1; tick <= 3; ++tick) {
		simulator.simulateState(body);
		EXPECT_TRUE(approxEqual(body.velocity.y, 0.0f)) << "TestSwimWaterGraceExpiresDeterministically: water travel while grace remains";
		EXPECT_TRUE(body.swimWaterGraceTicks == 3 - tick) << "TestSwimWaterGraceExpiresDeterministically: grace decays one tick per dry tick";
	}

	simulator.simulateState(body);
	EXPECT_TRUE(!approxEqual(body.velocity.y, 0.0f)) << "TestSwimWaterGraceExpiresDeterministically: water travel stops once the grace is exhausted";
}

TEST(PortLiquidHardening, CheckSwimWaterGraceOptionOverride) {
	const auto world = filledColumn(waterBlock());
	auto options = liquidOptions();
	options.swimWaterGraceTicks = 25;
	Simulator simulator{ world, options };
	auto body = submergedBody();
	body.swimming = true;

	simulator.simulateState(body);
	EXPECT_TRUE(body.swimWaterGraceTicks == 25) << "TestSwimWaterGraceOptionOverride: grace widened to 25";
}

TEST(PortLiquidHardening, CheckSwimWaterGraceCanBeDisabled) {
	auto options = liquidOptions();
	options.swimWaterGraceTicks = -1;

	const auto wet = filledColumn(waterBlock());
	Simulator simulator{ wet, options };
	auto body = submergedBody();
	body.swimming = true;
	simulator.simulateState(body);
	EXPECT_TRUE(body.swimWaterGraceTicks == 0) << "TestSwimWaterGraceCanBeDisabled: grace stays 0 when disabled";

	const FakeWorld dryWorld;
	Simulator dry{ dryWorld, options };
	auto dryState = dryBody();
	dryState.swimming = true;
	dryState.swimWaterGraceTicks = 5;
	dry.simulateState(dryState);
	EXPECT_TRUE(!approxEqual(dryState.velocity.y, 0.0f)) << "TestSwimWaterGraceCanBeDisabled: disabled grace must not permit water travel out of water";
}

TEST(PortLiquidHardening, CheckSwimWaterGraceResetOnUnreliableFrame) {
	const FakeWorld world;
	Simulator simulator{ world, liquidOptions() };
	auto body = dryBody();
	body.swimming = true;
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;
	body.alive = false;

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Unreliable) << "TestSwimWaterGraceResetOnUnreliableFrame: outcome unreliable";
	EXPECT_TRUE(body.swimWaterGraceTicks == 0) << "TestSwimWaterGraceResetOnUnreliableFrame: grace reset";
}

TEST(PortLiquidHardening, CheckSwimWaterGraceResetOnUnloadedChunk) {
	FakeWorld world;
	world.loaded = [](const AABB&) {
		return false;
	};

	Simulator simulator{ world, liquidOptions() };
	auto body = dryBody();
	body.swimming = true;
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;

	simulator.simulateState(body);
	EXPECT_TRUE(body.swimWaterGraceTicks == 0) << "TestSwimWaterGraceResetOnUnloadedChunk: grace reset";
}

TEST(PortLiquidHardening, CheckSwimWaterGraceIsBounded) {
	const auto world = filledColumn(waterBlock());
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = true;

	bool bounded = true;
	for (int tick = 0; tick < 100; ++tick) {
		simulator.simulateState(body);
		bounded = bounded && body.swimWaterGraceTicks <= kSwimWaterGraceTicks;
	}

	EXPECT_TRUE(bounded) << "TestSwimWaterGraceIsBounded: grace never exceeds the default";
}

TEST(PortLiquidHardening, CheckRealWaterContactDoesNotNeedSwimmingFlag) {
	const auto world = filledColumn(waterBlock());
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = false;

	simulator.simulateState(body);
	EXPECT_TRUE(approxVec(body.velocity, { 0.0f, -0.005f, 0.0f })) << "TestRealWaterContactDoesNotNeedSwimmingFlag: water travel velocity";
}

TEST(PortLiquidHardening, CheckDefaultSwimWaterGraceTicksValue) {
	EXPECT_TRUE(kSwimWaterGraceTicks == 10) << "TestDefaultSwimWaterGraceTicksValue: default grace is 10";
}

TEST(PortLiquidHardening, CheckSwimWaterGraceResetOnTeleport) {
	const FakeWorld world;
	Simulator simulator{ world, liquidOptions() };
	auto body = dryBody();
	body.swimming = true;
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;
	body.teleportPosition = { 50.0f, 50.0f, 50.0f };
	body.teleportCompletionTicks = 3;
	body.ticksSinceTeleport = 0;

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Teleport) << "TestSwimWaterGraceResetOnTeleport: outcome teleport";
	EXPECT_TRUE(body.swimWaterGraceTicks == 0) << "TestSwimWaterGraceResetOnTeleport: grace reset across a teleport";
}

TEST(PortLiquidHardening, CheckSwimWaterGraceResetWhenImmobile) {
	const FakeWorld world;
	Simulator simulator{ world, liquidOptions() };
	auto body = dryBody();
	body.swimming = true;
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;
	body.immobile = true;

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::ImmobileOrNotReady) << "TestSwimWaterGraceResetWhenImmobile: outcome immobile";
	EXPECT_TRUE(body.swimWaterGraceTicks == 0) << "TestSwimWaterGraceResetWhenImmobile: grace reset while immobile";
}

TEST(PortLiquidHardening, CheckLavaWinsOverStaleWaterGrace) {
	const auto world = filledColumn(lavaBlock());
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = true;
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;

	simulator.simulateState(body);
	EXPECT_TRUE(approxVec(body.velocity, { 0.0f, -0.02f, 0.0f })) << "TestLavaWinsOverStaleWaterGrace: lava gravity applies";
}

TEST(PortLiquidHardening, CheckSwimSpeedMultiplierDepthStriderScaling) {
	const auto run = [](const int level) {
		auto world = filledColumn(waterBlock());
		world.enchantments[Enchantment::DepthStrider] = level;
		Simulator simulator{ world, liquidOptions() };
		auto body = submergedBody();
		body.swimming = true;
		body.swimWaterGraceTicks = kSwimWaterGraceTicks;
		body.swimSpeedMultiplier = 2.0f;
		body.onGround = true;
		body.impulse = { 0.0f, 0.98f };
		simulator.simulateState(body);
		return body.velocity.z;
	};

	const float none = run(0);
	const float full = run(3);
	EXPECT_TRUE(!approxEqual(none, 0.0f)) << "TestSwimSpeedMultiplierDepthStriderScaling: baseline velocity is non-zero";
	EXPECT_TRUE(std::abs(full / none - 1.0f / 0.7f) <= 1e-6f) << "TestSwimSpeedMultiplierDepthStriderScaling: full/none ratio is 1/0.7";
}

TEST(PortLiquidHardening, CheckHasLiquidLayerReportsExplicitProvider) {
	EXPECT_TRUE(!Simulator<PlainWorld>::hasLiquidLayer()) << "TestHasLiquidLayerReportsExplicitProvider: a plain world reports no liquid layer";
	EXPECT_TRUE(Simulator<LayeredWorld>::hasLiquidLayer()) << "TestHasLiquidLayerReportsExplicitProvider: an explicit provider reports support";
}

TEST(PortLiquidHardening, CheckHasLiquidLayerAcceptsWorldProvider) {
	const LayeredWorld world;
	const Simulator simulator{ world, liquidOptions() };
	EXPECT_TRUE(simulator.hasLiquidLayer()) << "TestHasLiquidLayerAcceptsWorldProvider: a world implementing the liquid provider reports support";
}

TEST(PortLiquidHardening, CheckExplicitLiquidsFieldTakesPrecedence) {
	LayeredWorld layered;
	layered.waterlog({ 0, 0, 0 }, {}, Liquid{ .kind = LiquidKind::Water });

	const LayeredWorld world{ .blocks = layered.blocks };
	EXPECT_TRUE(touchingWater(world, submergedBody()) == 0) << "TestExplicitLiquidsFieldTakesPrecedence: explicit empty provider hides the world's layer";
}

TEST(PortLiquidHardening, CheckExplicitLiquidsProviderDetectsWaterlogged) {
	LayeredWorld world;
	for (int y = 0; y < 4; ++y) {
		world.layer[{ 0, y, 0 }] = Liquid{ .kind = LiquidKind::Water };
	}

	auto body = submergedBody();
	EXPECT_TRUE(touchingWater(world, body) != 0) << "TestExplicitLiquidsProviderDetectsWaterlogged: waterlogged blocks detected";

	Simulator simulator{ world, liquidOptions() };
	simulator.simulateState(body);
	EXPECT_TRUE(approxVec(body.velocity, { 0.0f, -0.005f, 0.0f })) << "TestExplicitLiquidsProviderDetectsWaterlogged: water travel velocity";
}

TEST(PortLiquidHardening, CheckRequireLiquidLayerFailsClosed) {
	const PlainWorld world;
	auto options = liquidOptions();
	options.requireLiquidLayer = true;
	Simulator simulator{ world, options };
	auto body = submergedBody();
	body.velocity = { 0.5f, 0.5f, 0.5f };

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Unreliable) << "TestRequireLiquidLayerFailsClosed: unreliable without a liquid layer";
}

TEST(PortLiquidHardening, CheckRequireLiquidLayerPassesWithProvider) {
	const LayeredWorld world;
	auto options = liquidOptions();
	options.requireLiquidLayer = true;
	Simulator simulator{ world, options };
	auto body = submergedBody();

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome != Outcome::Unreliable) << "TestRequireLiquidLayerPassesWithProvider: a liquid layer does not fail closed";
}

TEST(PortLiquidHardening, CheckLiquidLayerFallbackStillSimulatesByDefault) {
	const PlainWorld world{ .blocks = filledColumn(waterBlock()) };
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Normal) << "TestLiquidLayerFallbackStillSimulatesByDefault: outcome normal";
}

TEST(PortLiquidHardening, CheckUpstreamImpulseClampingOptIn) {
	struct Case {
		const char* name;
		bool upstream;
		Input input;
		float want;
	};

	const std::array cases{
		Case{ "TestUpstreamImpulseClampingOptIn/sneak default", false, { .moveVector = { 0.0f, 1.0f }, .sneakDown = true }, kMaxSneakImpulse * 0.98f },
		Case{ "TestUpstreamImpulseClampingOptIn/sneak upstream", true, { .moveVector = { 0.0f, 1.0f }, .sneakDown = true }, 0.98f },
		Case{ "TestUpstreamImpulseClampingOptIn/consumable default", false, { .moveVector = { 0.0f, 1.0f }, .usingConsumable = true }, kMaxConsumingImpulse * 0.98f },
		Case{ "TestUpstreamImpulseClampingOptIn/consumable upstream", true, { .moveVector = { 0.0f, 1.0f }, .usingConsumable = true }, 0.98f }
	};

	for (const auto& [name, upstream, input, want] : cases) {
		const FakeWorld world;
		auto options = liquidOptions();
		options.upstreamImpulseClamping = upstream;
		Scratch scratch;
		const systems::Systems<FakeWorld> systems{ world, options, scratch };
		auto body = baseBody();

		std::ignore = systems.inputs.apply(body, input);
		EXPECT_TRUE(approxEqual(body.impulse.y, want)) << name;
	}
}

TEST(PortLiquidHardening, CheckUpstreamImpulseClampingStillBoundsMoveVector) {
	const FakeWorld world;
	auto options = liquidOptions();
	options.upstreamImpulseClamping = true;
	Scratch scratch;
	const systems::Systems<FakeWorld> systems{ world, options, scratch };
	auto body = baseBody();

	std::ignore = systems.inputs.apply(body, { .moveVector = { 5.0f, -5.0f } });
	EXPECT_TRUE(approxEqual(body.impulse.x, 0.98f) && approxEqual(body.impulse.y, -0.98f)) << "TestUpstreamImpulseClampingStillBoundsMoveVector: move vector clamped to [-1, 1] then scaled";
}

TEST(PortLiquidHardening, CheckFlyingIsUnreliableBeforePhysics) {
	const auto world = filledColumn(waterBlock());
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.flying = true;
	body.velocity = { 0.25f, 0.25f, 0.25f };
	body.client.velocity = { 1.0f, 2.0f, 3.0f };

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Unreliable) << "TestFlyingIsUnreliableBeforePhysics: outcome unreliable";
	EXPECT_TRUE(approxVec(body.velocity, { 1.0f, 2.0f, 3.0f })) << "TestFlyingIsUnreliableBeforePhysics: velocity reset to the client";
}

TEST(PortLiquidHardening, CheckFlowDropWeightIsEight) {
	FakeWorld world;
	world.set({ 0, 0, 0 }, waterBlock(8));
	world.set({ -1, 0, 0 }, waterBlock(7));
	world.set({ 0, 0, 1 }, waterBlock(4));
	world.set({ 0, 0, -1 }, waterBlock(8));
	world.set({ 1, -1, 0 }, waterBlock(8));

	const auto flow = liquidFlow(world, { 0, 0, 0 });
	EXPECT_TRUE(approxVec(flow, normalize(Vec3{ 7.0f, 0.0f, 4.0f }))) << "TestFlowDropWeightIsEight: drop weight is 8";
}

TEST(PortLiquidHardening, CheckPartialNeighbourFaceAllowsFlowIntoDrop) {
	FakeWorld world;
	world.set({ 0, 0, 0 }, waterBlock(8));
	world.set({ 1, 0, 0 }, { .name = "minecraft:stone_slab", .shapes = { AABB{ {}, { 1.0f, 0.5f, 1.0f } } } });
	world.set({ 1, -1, 0 }, waterBlock(8));

	const auto flow = liquidFlow(world, { 0, 0, 0 });
	EXPECT_TRUE(flow.x > 0.0f) << "TestPartialNeighbourFaceAllowsFlowIntoDrop: flow through the slab's open side";
}

TEST(PortLiquidHardening, CheckCurrentBlockClosedFaceBlocksFlowIntoDrop) {
	LayeredWorld world;
	world.waterlog({ 0, 0, 0 }, stairsBlock(Face::East), Liquid{ .kind = LiquidKind::Water });
	world.blocks.set({ 1, -1, 0 }, waterBlock(8));

	const auto flow = liquidFlow(world, { 0, 0, 0 });
	EXPECT_TRUE(flow.x == 0.0f) << "TestCurrentBlockClosedFaceBlocksFlowIntoDrop: no flow through the current block's closed face";
}

TEST(PortLiquidHardening, CheckFallingFlowUsesProviderBarrierSemantics) {
	FlowSemanticsWorld world;
	world.set({ 0, 0, 0 }, waterBlock(8, true));
	world.set({ 1, 0, 0 }, stoneBlock());

	const auto flow = liquidFlow(world, { 0, 0, 0 });
	EXPECT_TRUE(length(flow) == 0.0f) << "TestFallingFlowUsesProviderBarrierSemantics: no falling-current barrier";
}

TEST(PortLiquidHardening, CheckFallingFlowDownwardWeightIsSix) {
	FakeWorld world;
	world.set({ 0, 0, 0 }, waterBlock(8, true));
	world.set({ -1, 0, 0 }, waterBlock(4));
	world.set({ 1, 0, 0 }, stoneBlock());

	const auto flow = liquidFlow(world, { 0, 0, 0 });
	EXPECT_TRUE(approxVec(flow, normalize(Vec3{ -1.0f, -6.0f, 0.0f }))) << "TestFallingFlowDownwardWeightIsSix: downward weight is 6";
}

TEST(PortLiquidHardening, CheckStairsSolidFaceBlocksFlow) {
	const auto build = [](const Face facing) {
		LayeredWorld world;
		world.waterlog({ 0, 0, 0 }, stairsBlock(facing), Liquid{ .kind = LiquidKind::Water });
		world.blocks.set({ 1, 0, 0 }, waterBlock(4));
		return liquidFlow(world, { 0, 0, 0 });
	};

	EXPECT_TRUE(approxEqual(build(Face::East).x, 0.0f)) << "TestStairsSolidFaceBlocksFlow: east-facing stairs close the +X side";
	EXPECT_TRUE(build(Face::West).x > 0.0f) << "TestStairsSolidFaceBlocksFlow: west-facing stairs leave the +X side open";
}

TEST(PortLiquidHardening, CheckWaterloggedTrapdoorSolidFaceBlocksFlow) {
	LayeredWorld world;
	const FakeBlock trapdoor{ .name = "minecraft:trapdoor", .shapes = { AABB{ { 0.8125f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } } } };
	world.waterlog({ 0, 0, 0 }, trapdoor, Liquid{ .kind = LiquidKind::Water });
	world.blocks.set({ 1, 0, 0 }, waterBlock(4));

	const auto flow = liquidFlow(world, { 0, 0, 0 });
	EXPECT_TRUE(approxEqual(flow.x, 0.0f)) << "TestWaterloggedTrapdoorSolidFaceBlocksFlow: trapdoor solid face blocks flow";
}

TEST(PortLiquidHardening, CheckNilWorldIsSafe) {
	const PlainWorld world;
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = true;
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;

	simulator.simulateState(body);

	const Options options{};
	Scratch scratch;
	const systems::Systems<PlainWorld> systems{ world, options, scratch };
	EXPECT_TRUE(touchingWater(world, body) == 0) << "TestNilWorldIsSafe: no water blocks without a world";
	EXPECT_TRUE(!systems.liquids.containsAnyLiquid(body.boundingBox(false))) << "TestNilWorldIsSafe: no liquid without a world";
	EXPECT_TRUE(length(liquidFlow(world, { 0, 0, 0 })) == 0.0f) << "TestNilWorldIsSafe: zero flow without a world";
	EXPECT_TRUE(!Simulator<PlainWorld>::hasLiquidLayer()) << "TestNilWorldIsSafe: no liquid layer without a world";
}

TEST(PortLiquidHardening, CheckSwimHitboxChangesCeilingCollision) {
	FakeWorld world;
	fill(world, { -1, 0, -1 }, { 1, 1, 1 }, waterBlock());
	world.solid({ 0, 2, 0 });

	const auto ceilingBody = []() {
		auto body = submergedBody();
		body.position = { 0.5f, 0.0f, 0.5f };
		body.client.position = body.position;
		body.velocity = { 0.0f, 0.5f, 0.0f };
		return body;
	};

	Simulator standingSimulator{ world, liquidOptions() };
	auto standing = ceilingBody();
	standingSimulator.simulateState(standing);

	Simulator swimmingSimulator{ world, liquidOptions() };
	auto swimming = ceilingBody();
	swimming.swimming = true;
	swimming.swimWaterGraceTicks = kSwimWaterGraceTicks;
	swimmingSimulator.simulateState(swimming);

	EXPECT_TRUE(standing.collideY) << "TestSwimHitboxChangesCeilingCollision: a standing hitbox hits the ceiling";
	EXPECT_TRUE(!swimming.collideY) << "TestSwimHitboxChangesCeilingCollision: a swimming hitbox fits under the ceiling";
}

[[nodiscard]] static FakeWorld goldenWorld() {
	FakeWorld world;
	fill(world, { -8, 0, -8 }, { 8, 8, 8 }, waterBlock(8));
	world.set({ 1, 0, 0 }, waterBlock(6));
	world.set({ 0, 0, 1 }, waterBlock(4));
	world.set({ -1, 1, 0 }, waterBlock(8, true));
	world.set({ 2, 0, 2 }, stoneBlock());
	world.enchantments[Enchantment::DepthStrider] = 2;
	return world;
}

[[nodiscard]] static Body runGoldenScenario(const FakeWorld& world) {
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = true;
	body.swimAmount = 0.5f;
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;

	const Input input{
		.moveVector = { 0.5f, 1.0f },
		.pitch = 25.0f,
		.yaw = 40.0f,
		.headYaw = 40.0f,
		.jumping = true
	};

	for (int tick = 0; tick < 20; ++tick) {
		simulator.simulate(body, input);
	}

	return body;
}

TEST(PortLiquidHardening, CheckLiquidSimulationIsRepeatable) {
	const auto world = goldenWorld();
	const auto expected = runGoldenScenario(world);
	for (int run = 0; run < 5; ++run) {
		const auto repeated = runGoldenScenario(goldenWorld());
		EXPECT_TRUE(repeated.position == expected.position && repeated.velocity == expected.velocity) << "TestLiquidSimulationIsRepeatable: runs agree exactly";
	}
}

TEST(PortLiquidHardening, CheckLiquidGoldenScenario) {
	const auto world = goldenWorld();
	const auto body = runGoldenScenario(world);

	const Vec3 wantPosition{ -0.012862622737884521f, 2.922518253326416f, 3.5954177379608154f };
	const Vec3 wantVelocity{ -0.027029925957322121f, 0.15549643337726593f, 0.11428361386060715f };
	EXPECT_TRUE(near(body.position.x, wantPosition.x)) << "TestLiquidGoldenScenario: Pos.X";
	EXPECT_TRUE(near(body.position.y, wantPosition.y)) << "TestLiquidGoldenScenario: Pos.Y";
	EXPECT_TRUE(near(body.position.z, wantPosition.z)) << "TestLiquidGoldenScenario: Pos.Z";
	EXPECT_TRUE(near(body.velocity.x, wantVelocity.x)) << "TestLiquidGoldenScenario: Vel.X";
	EXPECT_TRUE(near(body.velocity.y, wantVelocity.y)) << "TestLiquidGoldenScenario: Vel.Y";
	EXPECT_TRUE(near(body.velocity.z, wantVelocity.z)) << "TestLiquidGoldenScenario: Vel.Z";
}

TEST(PortLiquidHardening, CheckBubbleColumnUsesBoarImpulsesAndCaps) {
	struct Case {
		const char* name;
		BubbleDirection direction;
		bool surface;
		float initial;
		float want;
	};

	constexpr std::array cases{
		Case{ "TestBubbleColumnUsesBoarImpulsesAndCaps/submerged up", BubbleDirection::Up, false, 0.0f, 0.06f },
		Case{ "TestBubbleColumnUsesBoarImpulsesAndCaps/submerged up cap", BubbleDirection::Up, false, 0.69f, 0.70f },
		Case{ "TestBubbleColumnUsesBoarImpulsesAndCaps/surface up", BubbleDirection::Up, true, 0.0f, 0.10f },
		Case{ "TestBubbleColumnUsesBoarImpulsesAndCaps/surface up cap", BubbleDirection::Up, true, 1.79f, 1.80f },
		Case{ "TestBubbleColumnUsesBoarImpulsesAndCaps/submerged down", BubbleDirection::Down, false, 0.0f, -0.03f },
		Case{ "TestBubbleColumnUsesBoarImpulsesAndCaps/submerged down cap", BubbleDirection::Down, false, -0.29f, -0.30f },
		Case{ "TestBubbleColumnUsesBoarImpulsesAndCaps/surface down", BubbleDirection::Down, true, 0.0f, -0.03f },
		Case{ "TestBubbleColumnUsesBoarImpulsesAndCaps/surface down cap", BubbleDirection::Down, true, -0.89f, -0.90f }
	};

	for (const auto& [name, direction, surface, initial, want] : cases) {
		auto body = baseBody();
		body.velocity.y = initial;
		applyBubbleColumn(body, direction, surface);
		EXPECT_TRUE(near(body.velocity.y, want)) << name;
	}
}

template <typename W>
static void applyBubbleColumns(const W& world, Body& body) {
	const Options options{};
	Scratch scratch;
	const systems::Systems<W> systems{ world, options, scratch };
	systems.liquids.applyBubbleColumns(body);
}

TEST(PortLiquidHardening, CheckBubbleColumnSurfaceAcceptsRegistryBackedAir) {
	FakeWorld world;
	world.set({ 0, 0, 0 }, { .bubble = BubbleDirection::Up });
	world.passable({ 0, 1, 0 }, "minecraft:air");
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };

	applyBubbleColumns(world, body);
	EXPECT_TRUE(body.velocity.y == 0.1f) << "TestBubbleColumnSurfaceAcceptsRegistryBackedAir: surface impulse above registry-backed air";
}

TEST(PortLiquidHardening, CheckBubbleColumnUsesExactSurfaceProvider) {
	ExactBubbleSurfaceWorld world;
	world.surface = true;
	world.set({ 0, 0, 0 }, { .bubble = BubbleDirection::Up });
	world.set({ 0, 1, 0 }, waterBlock(8));
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };

	applyBubbleColumns(world, body);
	EXPECT_TRUE(body.velocity.y == 0.1f) << "TestBubbleColumnUsesExactSurfaceProvider: exact surface provider is honoured";
}

TEST(PortLiquidHardening, CheckBubbleColumnAppliesForEachOccupiedCell) {
	FakeWorld world;
	auto bubbleWater = waterBlock(8);
	bubbleWater.bubble = BubbleDirection::Up;
	world.set({ 0, 0, 0 }, bubbleWater);
	world.set({ 0, 1, 0 }, bubbleWater);
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.fallDistance = 4.0f;

	applyBubbleColumns(world, body);
	EXPECT_TRUE(near(body.velocity.y, 0.16f)) << "TestBubbleColumnAppliesForEachOccupiedCell: per-cell impulses total 0.16";
	EXPECT_TRUE(body.fallDistance == 0.0f) << "TestBubbleColumnAppliesForEachOccupiedCell: bubble contact clears fall distance";
}

TEST(PortLiquidHardening, CheckBubbleColumnAppliesOutsideLiquidTravel) {
	FakeWorld world;
	world.set({ 0, 0, 0 }, { .bubble = BubbleDirection::Up });
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.hasGravity = false;

	simulator.simulateState(body);
	EXPECT_TRUE(body.velocity.y == 0.1f) << "TestBubbleColumnAppliesOutsideLiquidTravel: normal movement applies the surface bubble impulse";
}

[[nodiscard]] static FakeWorld riptideWorld(const std::optional<FakeBlock>& liquid) {
	FakeWorld world;
	world.enchantments[Enchantment::Riptide] = 2;
	if (liquid) {
		world.set({ 0, 0, 0 }, *liquid);
	}

	return world;
}

[[nodiscard]] static Body riptideBody() {
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.gravity = kGravity;
	body.riptideReady = true;
	return body;
}

TEST(PortLiquidHardening, CheckRiptideLaunchesInWaterAndStartsSpinAttack) {
	const auto world = riptideWorld(waterBlock(8));
	Simulator simulator{ world };
	auto body = riptideBody();

	simulator.simulate(body, { .startSpinAttack = true });
	EXPECT_TRUE(near(body.velocity.z, 1.8f)) << "TestRiptideLaunchesInWaterAndStartsSpinAttack: launch velocity";
	EXPECT_TRUE(body.riptideTicks == 19) << "TestRiptideLaunchesInWaterAndStartsSpinAttack: 19 riptide ticks after the launch tick";
}

TEST(PortLiquidHardening, CheckRiptideHeadWaterUsesSneakingOffset) {
	FakeWorld world;
	world.set({ 0, 1, 0 }, waterBlock(2));
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.sneaking = true;

	EXPECT_TRUE(simulator.observeHeadLiquid(body).water) << "TestRiptideHeadWaterUsesSneakingOffset: sneaking head below the partial water surface is wet";
}

TEST(PortLiquidHardening, CheckRiptideDoesNotCompensateDisabledGravity) {
	auto body = baseBody();
	body.onGround = true;
	body.hasGravity = false;

	const auto impulse = riptideImpulse(body, 2, false, false);
	EXPECT_TRUE(impulse.y == 0.0f) << "TestRiptideDoesNotCompensateDisabledGravity: no vertical motion without gravity";
}

TEST(PortLiquidHardening, CheckRiptideUsesConfiguredGravityForDryCompensation) {
	auto body = baseBody();
	body.onGround = true;
	body.hasGravity = true;
	body.gravity = 0.04f;

	const auto impulse = riptideImpulse(body, 2, false, false);
	EXPECT_TRUE(near(impulse.y, body.gravity)) << "TestRiptideUsesConfiguredGravityForDryCompensation: vertical compensation equals configured gravity";
}

TEST(PortLiquidHardening, CheckRiptideDoesNotLaunchInLava) {
	const auto world = riptideWorld(lavaBlock());
	Simulator simulator{ world };
	auto body = riptideBody();

	simulator.simulate(body, { .startSpinAttack = true });
	EXPECT_TRUE(body.riptideTicks == 0) << "TestRiptideDoesNotLaunchInLava: lava does not start riptide";
}

TEST(PortLiquidHardening, CheckRiptideDoesNotLaunchWhileFlying) {
	const auto world = riptideWorld(waterBlock(8));
	Simulator simulator{ world };
	auto body = riptideBody();
	body.flying = true;

	simulator.simulate(body, { .startSpinAttack = true });
	EXPECT_TRUE(body.riptideTicks == 0) << "TestRiptideDoesNotLaunchWhileFlying: flying does not start riptide";
}

TEST(PortLiquidHardening, CheckRiptideDoesNotLaunchWhileMounted) {
	const auto world = riptideWorld(waterBlock(8));
	Simulator simulator{ world };
	auto body = riptideBody();
	body.inVehicle = true;

	simulator.simulate(body, { .startSpinAttack = true });
	EXPECT_TRUE(body.riptideTicks == 0) << "TestRiptideDoesNotLaunchWhileMounted: mounted player does not start riptide";
}

TEST(PortLiquidHardening, CheckRiptideRequiresValidatedTridentRelease) {
	const auto world = riptideWorld(waterBlock(8));
	Simulator simulator{ world };
	auto body = riptideBody();
	body.riptideReady = false;

	simulator.simulate(body, { .startSpinAttack = true });
	EXPECT_TRUE(body.riptideTicks == 0) << "TestRiptideRequiresValidatedTridentRelease: unvalidated start flag does not launch";
}

TEST(PortLiquidHardening, CheckRiptideLaunchesInRainWithoutBlockWater) {
	const auto world = riptideWorld(std::nullopt);
	Simulator simulator{ world };
	auto body = baseBody();
	body.gravity = kGravity;
	body.riptideReady = true;
	body.riptideInRain = true;

	simulator.simulate(body, { .startSpinAttack = true });
	EXPECT_TRUE(body.riptideTicks == 19) << "TestRiptideLaunchesInRainWithoutBlockWater: rain-authorized launch";
}

TEST(PortLiquidHardening, CheckRiptideStopsOnNormalMovementWallCollision) {
	const BoxWorld world{ .boxes = { AABB{ { 0.5f, -1.0f, -1.0f }, { 1.5f, 2.0f, 1.0f } } } };
	Simulator simulator{ world };
	auto body = baseBody();
	body.velocity = { 1.0f, 0.0f, 0.0f };
	body.riptideTicks = 10;
	body.hasGravity = false;

	simulator.simulateState(body);
	EXPECT_TRUE(body.riptideTicks == 0) << "TestRiptideStopsOnNormalMovementWallCollision: wall collision stops riptide";
}

TEST(PortLiquidHardening, CheckRiptideCollisionClearsActiveAttack) {
	const BoxWorld world;
	const Options options{};
	Scratch scratch;
	const systems::Systems<BoxWorld> systems{ world, options, scratch };
	auto body = baseBody();
	body.riptideTicks = 10;
	body.collideX = true;

	std::ignore = systems.poses.stopRiptideOnBlockCollision(body);
	EXPECT_TRUE(body.riptideTicks == 0) << "TestRiptideCollisionClearsActiveAttack: collision clears the active attack";
}

TEST(PortLiquidHardening, CheckRiptideStopRequiresValidatedEntityCollision) {
	const BoxWorld world;
	const Options options{};
	Scratch scratch;
	const systems::Systems<BoxWorld> systems{ world, options, scratch };
	auto body = baseBody();
	body.riptideTicks = 10;
	body.velocity = { 1.0f, 0.0f, 0.0f };

	std::ignore = systems.inputs.apply(body, { .stopSpinAttack = true });
	EXPECT_TRUE(body.riptideTicks == 10 && body.velocity.x == 1.0f) << "TestRiptideStopRequiresValidatedEntityCollision: spoofed stop is ignored";

	body.riptideCollision = true;
	std::ignore = systems.inputs.apply(body, { .stopSpinAttack = true });
	EXPECT_TRUE(body.riptideTicks == 0 && body.velocity.x == -0.2f) << "TestRiptideStopRequiresValidatedEntityCollision: validated stop applies";
}
