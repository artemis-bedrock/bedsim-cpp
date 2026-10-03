#include "helpers.h"
#include "fake_world.h"

#include "bedsim/constants.h"
#include "bedsim/physics.h"
#include "bedsim/simulator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>
#include <gtest/gtest.h>

using namespace bedsim;
using bedsim::test::FakeBlock;
using bedsim::test::FakeWorld;
using bedsim::test::baseBody;
using bedsim::test::kFullBlock;
using bedsim::test::near;

struct BareLiquidWorld {
	const FakeWorld& source;

	void blockCollisions(const BlockPos& position, std::vector<AABB>& out) const {
		source.blockCollisions(position, out);
	}

	[[nodiscard]] BlockMovement movement(const BlockPos& position) const {
		return source.movement(position);
	}

	[[nodiscard]] bool isLoaded(const AABB& area) const {
		return source.isLoaded(area);
	}

	[[nodiscard]] std::optional<Liquid> liquid(const BlockPos& position) const {
		return source.liquid(position);
	}
};

struct ElytraLiquidWorld : BareLiquidWorld {
	[[nodiscard]] bool hasElytra() const {
		return false;
	}
};

struct ClimbableWorld {
	void blockCollisions(const BlockPos&, std::vector<AABB>&) const { }

	[[nodiscard]] BlockMovement movement(const BlockPos&) const {
		return { .air = true, .climbable = true };
	}

	[[nodiscard]] bool isLoaded(const AABB&) const {
		return true;
	}
};

static_assert(!HasEffects<BareLiquidWorld> && !HasEquipment<BareLiquidWorld> && HasLiquids<BareLiquidWorld>);
static_assert(HasElytra<ElytraLiquidWorld> && !HasEquipment<ElytraLiquidWorld>);
static_assert(!HasLiquids<ClimbableWorld> && !HasClimbableContact<ClimbableWorld>);

[[nodiscard]] static Options liquidOptions() {
	return { .positionCorrectionThreshold = 0.3f };
}

[[nodiscard]] static Body submergedBody() {
	auto body = baseBody();
	body.position = { 0.5f, 0.5f, 0.5f };
	body.client.position = body.position;
	return body;
}

static void fillLiquid(FakeWorld& world, const BlockPos& minimum, const BlockPos& maximum, const LiquidKind kind) {
	for (int x = minimum.x; x <= maximum.x; ++x) {
		for (int y = minimum.y; y <= maximum.y; ++y) {
			for (int z = minimum.z; z <= maximum.z; ++z) {
				if (kind == LiquidKind::Water) {
					world.water({ x, y, z });
				} else {
					world.lava({ x, y, z });
				}
			}
		}
	}
}

[[nodiscard]] static FakeWorld filledColumn(const LiquidKind kind) {
	FakeWorld world;
	fillLiquid(world, { -2, 0, -2 }, { 2, 3, 2 }, kind);
	return world;
}

[[nodiscard]] static FakeWorld surfaceWorld() {
	FakeWorld world;
	fillLiquid(world, { -2, -4, -2 }, { 2, 0, 2 }, LiquidKind::Water);
	return world;
}

[[nodiscard]] static bool nearVec(const Vec3& value, const Vec3& expected) {
	return near(value.x, expected.x) && near(value.y, expected.y) && near(value.z, expected.z);
}

[[nodiscard]] static float height(const AABB& box) {
	return box.max.y - box.min.y;
}

[[nodiscard]] static float width(const AABB& box) {
	return box.max.x - box.min.x;
}

template <World W>
static systems::AppliedInput applyInput(const W& world, Body& body, const Input& input) {
	const auto options = liquidOptions();
	Scratch scratch;
	const systems::Systems<W> systems{ world, options, scratch };
	return systems.inputs.apply(body, input);
}

template <World W>
[[nodiscard]] static std::vector<BlockPos> touchingLiquid(const W& world, const Body& body, const LiquidKind kind) {
	const auto options = liquidOptions();
	Scratch scratch;
	const systems::Systems<W> systems{ world, options, scratch };
	std::vector<BlockPos> positions;
	systems.liquids.touchingBlocks(body, kind, positions);
	return positions;
}

template <World W>
[[nodiscard]] static bool applyFlow(const W& world, Body& body, const std::span<const BlockPos> positions, const LiquidKind kind) {
	const auto options = liquidOptions();
	Scratch scratch;
	const systems::Systems<W> systems{ world, options, scratch };
	return systems.liquids.applyFlow(body, positions, kind);
}

template <World W>
[[nodiscard]] static bool containsAnyLiquid(const W& world, const AABB& box) {
	const auto options = liquidOptions();
	Scratch scratch;
	const systems::Systems<W> systems{ world, options, scratch };
	return systems.liquids.containsAnyLiquid(box);
}

[[nodiscard]] static Vec3 waterFlowAt(const FakeWorld& world, const BlockPos& position) {
	auto body = submergedBody();
	const std::array positions{ position };
	EXPECT_TRUE(applyFlow(world, body, positions, LiquidKind::Water)) << "liquid flow probe area is loaded";
	return body.velocity * (1.0f / 0.014f);
}

TEST(PortLiquid, SwimmingBoundingBoxUsesWidthAsHeight) {
	auto body = baseBody();
	body.position = { 0.5f, 10.0f, 0.5f };

	const auto standing = body.boundingBox(false);
	EXPECT_TRUE(near(height(standing), 1.8f)) << "TestSwimmingBoundingBoxUsesWidthAsHeight: standing height is 1.8";

	body.swimming = true;
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;
	const auto swimming = body.boundingBox(false);
	EXPECT_TRUE(near(height(swimming), 0.6f)) << "TestSwimmingBoundingBoxUsesWidthAsHeight: swimming height is 0.6";
	EXPECT_TRUE(near(width(swimming), width(standing))) << "TestSwimmingBoundingBoxUsesWidthAsHeight: swimming width is unchanged";
}

TEST(PortLiquid, SwimmingFlagAloneDoesNotShrinkHitbox) {
	auto body = baseBody();
	body.position = { 0.5f, 10.0f, 0.5f };
	body.swimming = true;
	body.swimWaterGraceTicks = 0;

	EXPECT_TRUE(!body.swimPose()) << "TestSwimmingFlagAloneDoesNotShrinkHitbox: swim pose requires water evidence";
	EXPECT_TRUE(near(height(body.boundingBox(false)), 1.8f)) << "TestSwimmingFlagAloneDoesNotShrinkHitbox: full standing hitbox";
	EXPECT_TRUE(near(height(body.clientBoundingBox(false)), 1.8f)) << "TestSwimmingFlagAloneDoesNotShrinkHitbox: full standing client hitbox";
}

TEST(PortLiquid, SpoofedSwimmingCannotFitThroughCeilingGap) {
	FakeWorld world;
	world.solid({ 0, 2, 0 });
	Simulator simulator{ world, liquidOptions() };
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.client.position = body.position;
	body.swimming = true;
	body.velocity = { 0.0f, 0.5f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(body.collideY) << "TestSpoofedSwimmingCannotFitThroughCeilingGap: a spoofed swim pose still collides with the ceiling";
}

TEST(PortLiquid, SwimmingClientBoundingBoxUsesWidthAsHeight) {
	auto body = baseBody();
	body.client.position = { 0.5f, 10.0f, 0.5f };

	EXPECT_TRUE(near(height(body.clientBoundingBox(false)), 1.8f)) << "TestSwimmingClientBoundingBoxUsesWidthAsHeight: standing client height is 1.8";

	body.swimming = true;
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;
	EXPECT_TRUE(near(height(body.clientBoundingBox(false)), 0.6f)) << "TestSwimmingClientBoundingBoxUsesWidthAsHeight: swimming client height is 0.6";
}

TEST(PortLiquid, SwimmingBoundingBoxRespectsScale) {
	auto body = baseBody();
	body.position = { 0.5f, 10.0f, 0.5f };
	body.size = { 0.6f, 1.8f, 2.0f };
	body.swimming = true;
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;

	EXPECT_TRUE(near(height(body.boundingBox(false)), 1.2f)) << "TestSwimmingBoundingBoxRespectsScale: scaled swimming height is 1.2";
}

TEST(PortLiquid, JumpingIsEdgeTriggeredByStartJumping) {
	const FakeWorld world;
	auto body = baseBody();

	applyInput(world, body, { .jumping = true });
	EXPECT_TRUE(!body.jumping) << "TestJumpingIsEdgeTriggeredByStartJumping: held jump does not set jumping";
	EXPECT_TRUE(body.pressingJump) << "TestJumpingIsEdgeTriggeredByStartJumping: held jump sets pressingJump";
	EXPECT_TRUE(body.effectiveJumping) << "TestJumpingIsEdgeTriggeredByStartJumping: held jump sets effectiveJumping";

	applyInput(world, body, { .startJumping = true });
	EXPECT_TRUE(body.jumping) << "TestJumpingIsEdgeTriggeredByStartJumping: startJumping sets jumping";
}

TEST(PortLiquid, EffectiveJumpingSources) {
	const FakeWorld world;

	struct Case {
		std::string_view name;
		Input input;
	};

	const std::array cases{
		Case{ .name = "jumping", .input = { .jumping = true } },
		Case{ .name = "autoJumpingInWater", .input = { .autoJumpingInWater = true } },
		Case{ .name = "ascendBlock", .input = { .ascendBlock = true } }
	};

	for (const auto& [name, input] : cases) {
		auto body = baseBody();
		applyInput(world, body, input);
		EXPECT_TRUE(body.effectiveJumping) << std::string{ "TestEffectiveJumpingSources: " } + std::string{ name } + " sets effectiveJumping";
	}

	auto body = baseBody();
	applyInput(world, body, { .startJumping = true });
	EXPECT_TRUE(!body.effectiveJumping) << "TestEffectiveJumpingSources: startJumping alone does not set effectiveJumping";
}

TEST(PortLiquid, SwimAmountInterpolation) {
	const FakeWorld world;
	auto body = baseBody();

	applyInput(world, body, { .startSwimming = true });
	EXPECT_TRUE(body.swimming) << "TestSwimAmountInterpolation: startSwimming sets swimming";
	EXPECT_TRUE(near(body.swimAmount, 0.0f)) << "TestSwimAmountInterpolation: first startSwimming tick still decays to 0";

	for (int tick = 1; tick <= 3; ++tick) {
		applyInput(world, body, {});
		EXPECT_TRUE(near(body.swimAmount, static_cast<float>(tick) * 0.1f)) << "TestSwimAmountInterpolation: swimAmount rises by 0.1 per tick";
	}

	applyInput(world, body, { .stopSwimming = true });
	EXPECT_TRUE(!body.swimming) << "TestSwimAmountInterpolation: stopSwimming clears swimming";
	EXPECT_TRUE(near(body.swimAmount, 0.4f)) << "TestSwimAmountInterpolation: swimAmount still rises to 0.4 on the stop tick";

	applyInput(world, body, {});
	EXPECT_TRUE(near(body.swimAmount, 0.3f)) << "TestSwimAmountInterpolation: swimAmount decays to 0.3";
}

TEST(PortLiquid, SwimAmountClampedToUnitRange) {
	const FakeWorld world;
	auto body = baseBody();
	body.swimming = true;
	for (int tick = 0; tick < 30; ++tick) {
		applyInput(world, body, {});
	}

	EXPECT_TRUE(near(body.swimAmount, 1.0f)) << "TestSwimAmountClampedToUnitRange: swimAmount clamps to 1";

	body.swimming = false;
	for (int tick = 0; tick < 30; ++tick) {
		applyInput(world, body, {});
	}

	EXPECT_TRUE(near(body.swimAmount, 0.0f)) << "TestSwimAmountClampedToUnitRange: swimAmount clamps to 0";
}

TEST(PortLiquid, StartSwimmingClearsSneaking) {
	const FakeWorld world;
	auto body = baseBody();
	body.crawling = true;

	applyInput(world, body, { .startSneaking = true, .sneakDown = true, .startSwimming = true });
	EXPECT_TRUE(!body.sneaking) << "TestStartSwimmingClearsSneaking: startSwimming clears sneaking";
	EXPECT_TRUE(body.size.y == body.standingHeight) << "TestStartSwimmingClearsSneaking: startSwimming clears the crouched height";
	EXPECT_TRUE(!body.crawling) << "TestStartSwimmingClearsSneaking: startSwimming clears crawling";
}

TEST(PortLiquid, SimulateStateInitializesPoseHeightBeforeSwimming) {
	FakeWorld world;
	world.water({ 0, 0, 0 });
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.swimming = true;
	body.standingHeight = 0.0f;
	body.sneakingHeight = 0.0f;
	body.crawlingHeight = 0.0f;

	Simulator simulator{ world };
	simulator.simulateState(body);

	EXPECT_TRUE(body.standingHeight == 1.8f && body.size.y == 1.8f) << "TestSimulateStateInitializesPoseHeightBeforeSwimming: standing pose initialized";
}

TEST(PortLiquid, StopSwimmingTakesPriority) {
	const FakeWorld world;
	auto body = baseBody();
	body.swimming = true;

	applyInput(world, body, { .startSwimming = true, .stopSwimming = true });
	EXPECT_TRUE(!body.swimming) << "TestStopSwimmingTakesPriority: stopSwimming wins over startSwimming";
}

TEST(PortLiquid, WaterDragAndGravity) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, { 0.0f, -0.005f, 0.0f })) << "TestWaterDragAndGravity: first tick sinks at water gravity";

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, { 0.0f, -0.005f * 0.8f - 0.005f, 0.0f })) << "TestWaterDragAndGravity: drag applies before gravity";
}

TEST(PortLiquid, WaterSprintDrag) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };

	auto normal = submergedBody();
	normal.velocity = { 0.5f, 0.0f, 0.0f };
	simulator.simulateState(normal);

	auto sprinting = submergedBody();
	sprinting.velocity = { 0.5f, 0.0f, 0.0f };
	sprinting.sprinting = true;
	simulator.simulateState(sprinting);

	EXPECT_TRUE(near(normal.velocity.x, 0.5f * 0.8f)) << "TestWaterSprintDrag: walking drag is 0.8";
	EXPECT_TRUE(near(sprinting.velocity.x, 0.5f * 0.9f)) << "TestWaterSprintDrag: sprinting drag is 0.9";
}

TEST(PortLiquid, WaterVerticalDragIndependentOfSprint) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.velocity = { 0.0f, 0.5f, 0.0f };
	body.sprinting = true;
	simulator.simulateState(body);

	EXPECT_TRUE(nearVec(body.velocity, { 0.0f, 0.5f * 0.8f - 0.005f, 0.0f })) << "TestWaterVerticalDragIndependentOfSprint: vertical drag stays 0.8";
}

TEST(PortLiquid, LavaDragAndGravity) {
	const auto world = filledColumn(LiquidKind::Lava);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.velocity = { 0.4f, 0.4f, 0.4f };

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, { 0.2f, 0.4f * 0.5f - 0.02f, 0.2f })) << "TestLavaDragAndGravity: flat 0.5 drag and 0.02 gravity";
}

TEST(PortLiquid, SwimmingCancelsWaterGravity) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = true;
	body.rotation = {};

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.y, 0.0f)) << "TestSwimmingCancelsWaterGravity: swimming vertical velocity is 0";
}

TEST(PortLiquid, StopSwimmingUsesFastWaterDragForOneTick) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = false;
	body.stoppedSwimmingThisTick = true;
	body.velocity = { 0.5f, 0.0f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.x, 0.45f)) << "TestStopSwimmingUsesFastWaterDragForOneTick: stop-swimming drag is 0.9";
}

TEST(PortLiquid, StopSwimmingFlagWithoutTransitionUsesNormalDrag) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = false;
	body.velocity = { 0.5f, 0.0f, 0.0f };

	simulator.simulate(body, { .stopSwimming = true });
	EXPECT_TRUE(near(body.velocity.x, 0.4f)) << "TestStopSwimmingFlagWithoutTransitionUsesNormalDrag: false stop-swimming uses 0.8 drag";
}

TEST(PortLiquid, NoGravityInLiquid) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.hasGravity = false;

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, {})) << "TestNoGravityInLiquid: no gravity leaves velocity at zero";
}

TEST(PortLiquid, LevitationOverridesLiquidGravity) {
	auto world = filledColumn(LiquidKind::Water);
	world.effects[Effect::Levitation] = 0;
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, { 0.0f, 0.05f * 0.2f, 0.0f })) << "TestLevitationOverridesLiquidGravity: levitation pull replaces gravity";
}

TEST(PortLiquid, LevitationAmplifierScales) {
	auto world = filledColumn(LiquidKind::Water);
	world.effects[Effect::Levitation] = 3;
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, { 0.0f, kLevitationSpeed * 4.0f * 0.2f, 0.0f })) << "TestLevitationAmplifierScales: amplifier 3 scales the target by 4";
}

TEST(PortLiquid, NilEffectsProviderFallsBackToGravity) {
	const auto fake = filledColumn(LiquidKind::Water);
	const BareLiquidWorld world{ .source = fake };
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, { 0.0f, -0.005f, 0.0f })) << "TestNilEffectsProviderFallsBackToGravity: world without effects uses water gravity";
}

TEST(PortLiquid, LiquidResetsFallDistance) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.fallDistance = 12.0f;

	simulator.simulateState(body);
	EXPECT_TRUE(body.fallDistance == 0.0f) << "TestLiquidResetsFallDistance: liquid clears fall distance";
}

TEST(PortLiquid, EffectiveJumpingAscendsInWater) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.effectiveJumping = true;

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, { 0.0f, 0.04f * 0.8f - 0.005f, 0.0f })) << "TestEffectiveJumpingAscendsInWater: jump adds a 0.04 ascent";
}

TEST(PortLiquid, SwimTransitionZeroesJumpAscent) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.effectiveJumping = true;
	body.swimAmount = 0.5f;

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, { 0.0f, -0.005f, 0.0f })) << "TestSwimTransitionZeroesJumpAscent: mid-transition zeroes the ascent";
}

TEST(PortLiquid, FullSwimAmountStillAscends) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.effectiveJumping = true;
	body.swimAmount = 1.0f;

	simulator.simulateState(body);
	EXPECT_TRUE(body.velocity.y > 0.0f) << "TestFullSwimAmountStillAscends: full swim amount still ascends";
}

TEST(PortLiquid, WantDownSinksInWater) {
	struct Case {
		std::string_view name;
		void (*apply)(Body&);
	};

	const std::array cases{
		Case{ .name = "wantDown", .apply = [](Body& body) {
			body.wantDown = true;
		} },
		Case{ .name = "wantDownSlow", .apply = [](Body& body) {
			body.wantDownSlow = true;
		} }
	};

	for (const auto& [name, apply] : cases) {
		const auto world = filledColumn(LiquidKind::Water);
		Simulator simulator{ world, liquidOptions() };
		auto body = submergedBody();
		apply(body);

		simulator.simulateState(body);
		EXPECT_TRUE(nearVec(body.velocity, { 0.0f, -0.04f * 0.8f - 0.005f, 0.0f })) << std::string{ "TestWantDownSinksInWater/" } + std::string{ name } + ": sinks by 0.04 before drag";
	}
}

TEST(PortLiquid, WantDownIgnoredInLava) {
	const auto world = filledColumn(LiquidKind::Lava);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.wantDown = true;

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, { 0.0f, -0.02f, 0.0f })) << "TestWantDownIgnoredInLava: lava ignores the descend input";
}

TEST(PortLiquid, DescendInputsDoNotChangeSneakImpulseClamp) {
	const auto world = filledColumn(LiquidKind::Water);

	auto sneaking = baseBody();
	applyInput(world, sneaking, { .moveVector = { 0.0f, 1.0f }, .sneakDown = true });

	auto descending = baseBody();
	applyInput(world, descending, { .moveVector = { 0.0f, 1.0f }, .sneakDown = true, .wantDown = true });

	EXPECT_TRUE(near(descending.impulse.y, sneaking.impulse.y)) << "TestDescendInputsDoNotChangeSneakImpulseClamp: descending impulse matches sneaking impulse";
	EXPECT_TRUE(near(sneaking.impulse.y, kMaxSneakImpulse * 0.98f)) << "TestDescendInputsDoNotChangeSneakImpulseClamp: sneaking impulse is clamped";
}

TEST(PortLiquid, SwimTravelFollowsPitch) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = true;
	body.rotation = { -90.0f, 0.0f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, { 0.0f, 0.06f * 0.8f, 0.0f })) << "TestSwimTravelFollowsPitch: looking up steers upward at 0.06";
}

TEST(PortLiquid, SwimTravelUsesFasterRateWhenDivingSteeply) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = true;
	body.rotation = { 90.0f, 0.0f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, { 0.0f, -0.085f * 0.8f, 0.0f })) << "TestSwimTravelUsesFasterRateWhenDivingSteeply: steep dive uses 0.085";
}

TEST(PortLiquid, SwimTravelSkippedWhileJumping) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = true;
	body.effectiveJumping = true;
	body.swimAmount = 1.0f;
	body.rotation = { 90.0f, 0.0f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, { 0.0f, 0.04f * 0.8f, 0.0f })) << "TestSwimTravelSkippedWhileJumping: only the jump impulse applies";
}

TEST(PortLiquid, SwimTravelStopsAtSurface) {
	const auto world = surfaceWorld();
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.position = { 0.5f, 1.5f, 0.5f };
	body.swimming = true;
	body.rotation = { -90.0f, 0.0f, 0.0f };
	body.velocity = { 0.0f, 0.5f, 0.0f };
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.y, 0.0f)) << "TestSwimTravelStopsAtSurface: vertical velocity is zeroed at the surface";
}

TEST(PortLiquid, SwimTravelContinuesWhileHeadSubmerged) {
	const auto world = surfaceWorld();
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = true;
	body.rotation = { -90.0f, 0.0f, 0.0f };
	body.velocity = { 0.0f, 0.5f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(!near(body.velocity.y, 0.0f)) << "TestSwimTravelContinuesWhileHeadSubmerged: a submerged head does not trigger the surface clamp";
}

TEST(PortLiquid, SwimTravelSurfaceClampSkippedWhenWantDownSlow) {
	const auto world = surfaceWorld();
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.position = { 0.5f, 1.5f, 0.5f };
	body.swimming = true;
	body.rotation = { -90.0f, 0.0f, 0.0f };
	body.wantDownSlow = true;
	body.velocity = { 0.0f, 0.5f, 0.0f };
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;

	simulator.simulateState(body);
	EXPECT_TRUE(!near(body.velocity.y, 0.0f)) << "TestSwimTravelSurfaceClampSkippedWhenWantDownSlow: wantDownSlow skips the surface clamp";
}

TEST(PortLiquid, SwimTravelSurfaceClampSkippedWhenPressingDescend) {
	const auto world = surfaceWorld();
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.position = { 0.5f, 1.5f, 0.5f };
	body.swimming = true;
	body.rotation = { -90.0f, 0.0f, 0.0f };
	body.pressingDescend = true;
	body.velocity = { 0.0f, 0.5f, 0.0f };
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;

	simulator.simulateState(body);
	EXPECT_TRUE(!near(body.velocity.y, 0.0f)) << "TestSwimTravelSurfaceClampSkippedWhenPressingDescend: pressingDescend skips the surface clamp";
}

TEST(PortLiquid, DepthStriderLowersDragCoefficient) {
	const auto baseWorld = filledColumn(LiquidKind::Water);
	Simulator base{ baseWorld, liquidOptions() };
	auto baseState = submergedBody();
	baseState.velocity = { 0.5f, 0.0f, 0.0f };
	base.simulateState(baseState);

	auto striderWorld = filledColumn(LiquidKind::Water);
	striderWorld.enchantments[Enchantment::DepthStrider] = 3;
	Simulator strider{ striderWorld, liquidOptions() };
	auto striderState = submergedBody();
	striderState.velocity = { 0.5f, 0.0f, 0.0f };
	striderState.onGround = true;
	strider.simulateState(striderState);

	EXPECT_TRUE(near(baseState.velocity.x, 0.5f * 0.8f)) << "TestDepthStriderLowersDragCoefficient: base drag is 0.8";
	EXPECT_TRUE(near(striderState.velocity.x, 0.5f * 0.54600006f)) << "TestDepthStriderLowersDragCoefficient: level 3 grounded drag is 0.546";
	EXPECT_TRUE(striderState.velocity.x < baseState.velocity.x) << "TestDepthStriderLowersDragCoefficient: depth strider decays faster";
}

TEST(PortLiquid, DepthStriderIncreasesAcceleration) {
	const auto baseWorld = filledColumn(LiquidKind::Water);
	Simulator base{ baseWorld, liquidOptions() };
	auto baseState = submergedBody();
	baseState.impulse = { 0.0f, 0.98f };
	base.simulateState(baseState);

	auto striderWorld = filledColumn(LiquidKind::Water);
	striderWorld.enchantments[Enchantment::DepthStrider] = 3;
	Simulator strider{ striderWorld, liquidOptions() };
	auto striderState = submergedBody();
	striderState.impulse = { 0.0f, 0.98f };
	striderState.onGround = true;
	strider.simulateState(striderState);

	EXPECT_TRUE(std::abs(striderState.velocity.z) > std::abs(baseState.velocity.z)) << "TestDepthStriderIncreasesAcceleration: depth strider accelerates faster";
}

TEST(PortLiquid, DepthStriderHalvedWhenAirborne) {
	auto world = filledColumn(LiquidKind::Water);
	world.enchantments[Enchantment::DepthStrider] = 3;
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.velocity = { 0.5f, 0.0f, 0.0f };
	body.onGround = false;

	simulator.simulateState(body);
	const float expected = 0.5f * (0.8f + (0.54600006f - 0.8f) * 0.5f);
	EXPECT_TRUE(near(body.velocity.x, expected)) << "TestDepthStriderHalvedWhenAirborne: airborne level is halved";
}

TEST(PortLiquid, DepthStriderClampedToMaxLevel) {
	auto world = filledColumn(LiquidKind::Water);
	world.enchantments[Enchantment::DepthStrider] = 99;
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.velocity = { 0.5f, 0.0f, 0.0f };
	body.onGround = true;

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.x, 0.5f * 0.54600006f)) << "TestDepthStriderClampedToMaxLevel: level 99 behaves as level 3";
}

TEST(PortLiquid, DepthStriderNegativeLevelIgnored) {
	auto world = filledColumn(LiquidKind::Water);
	world.enchantments[Enchantment::DepthStrider] = -5;
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.velocity = { 0.5f, 0.0f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.x, 0.5f * 0.8f)) << "TestDepthStriderNegativeLevelIgnored: negative level uses plain water drag";
}

TEST(PortLiquid, DepthStriderIgnoredInLava) {
	auto world = filledColumn(LiquidKind::Lava);
	world.enchantments[Enchantment::DepthStrider] = 3;
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.velocity = { 0.5f, 0.0f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.x, 0.5f * 0.5f)) << "TestDepthStriderIgnoredInLava: lava keeps flat 0.5 drag";
}

TEST(PortLiquid, InventoryWithoutDepthStriderProvider) {
	const auto fake = filledColumn(LiquidKind::Water);
	const ElytraLiquidWorld world{ { .source = fake } };
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.velocity = { 0.5f, 0.0f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.x, 0.5f * 0.8f)) << "TestInventoryWithoutDepthStriderProvider: world without equipment uses plain water drag";
}

TEST(PortLiquid, ZeroEquipmentDepthStriderFallsBackToLegacyInventory) {
	auto world = filledColumn(LiquidKind::Water);
	world.enchantments[Enchantment::DepthStrider] = 3;
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.velocity = { 0.5f, 0.0f, 0.0f };
	body.onGround = true;

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.x, 0.5f * 0.54600006f)) << "TestZeroEquipmentDepthStriderFallsBackToLegacyInventory: level 3 equipment gives level-3 drag";
}

TEST(PortLiquid, SwimSpeedMultiplierRequiresSwimming) {
	const auto world = filledColumn(LiquidKind::Water);

	Simulator boosted{ world, liquidOptions() };
	auto boostedState = submergedBody();
	boostedState.swimming = true;
	boostedState.swimSpeedMultiplier = 2.0f;
	boostedState.impulse = { 0.0f, 0.98f };
	boosted.simulateState(boostedState);

	Simulator plain{ world, liquidOptions() };
	auto plainState = submergedBody();
	plainState.swimming = true;
	plainState.swimSpeedMultiplier = 1.0f;
	plainState.impulse = { 0.0f, 0.98f };
	plain.simulateState(plainState);

	EXPECT_TRUE(std::abs(boostedState.velocity.z) > std::abs(plainState.velocity.z)) << "TestSwimSpeedMultiplierRequiresSwimming: boosted swimmer is faster";

	Simulator notSwimming{ world, liquidOptions() };
	auto notSwimmingState = submergedBody();
	notSwimmingState.swimSpeedMultiplier = 2.0f;
	notSwimmingState.impulse = { 0.0f, 0.98f };
	notSwimming.simulateState(notSwimmingState);

	EXPECT_TRUE(near(notSwimmingState.velocity.z, plainState.velocity.z)) << "TestSwimSpeedMultiplierRequiresSwimming: non-swimmer is unboosted";
}

TEST(PortLiquid, DolphinBoostTicksExpire) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = true;
	body.dolphinBoostTicks = 2;
	body.swimSpeedMultiplier = 2.0f;

	simulator.simulate(body, {});
	EXPECT_TRUE(body.dolphinBoostTicks == 1) << "TestDolphinBoostTicksExpire: boost ticks count down to 1";
	EXPECT_TRUE(near(body.swimSpeedMultiplier, 2.0f)) << "TestDolphinBoostTicksExpire: multiplier still boosted";

	simulator.simulate(body, {});
	EXPECT_TRUE(body.dolphinBoostTicks == 0) << "TestDolphinBoostTicksExpire: boost ticks reach 0";
	EXPECT_TRUE(near(body.swimSpeedMultiplier, kSwimSpeedMultiplier)) << "TestDolphinBoostTicksExpire: multiplier reset to the default";
}

TEST(PortLiquid, ZeroSwimSpeedMultiplierTreatedAsDefault) {
	const auto world = filledColumn(LiquidKind::Water);

	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = true;
	body.swimSpeedMultiplier = 0.0f;
	body.impulse = { 0.0f, 0.98f };
	simulator.simulateState(body);

	Simulator explicitSimulator{ world, liquidOptions() };
	auto explicitBody = submergedBody();
	explicitBody.swimming = true;
	explicitBody.swimSpeedMultiplier = kSwimSpeedMultiplier;
	explicitBody.impulse = { 0.0f, 0.98f };
	explicitSimulator.simulateState(explicitBody);

	EXPECT_TRUE(nearVec(body.velocity, explicitBody.velocity)) << "TestZeroSwimSpeedMultiplierTreatedAsDefault: zero multiplier matches the default";
}

TEST(PortLiquid, ZeroMovementSpeedsUseDefaults) {
	const auto world = filledColumn(LiquidKind::Water);

	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.underwaterMovementSpeed = 0.0f;
	body.impulse = { 0.0f, 0.98f };
	simulator.simulateState(body);

	Simulator explicitSimulator{ world, liquidOptions() };
	auto explicitBody = submergedBody();
	explicitBody.underwaterMovementSpeed = kUnderwaterMovementSpeed;
	explicitBody.impulse = { 0.0f, 0.98f };
	explicitSimulator.simulateState(explicitBody);

	EXPECT_TRUE(nearVec(body.velocity, explicitBody.velocity)) << "TestZeroMovementSpeedsUseDefaults: zero underwater speed matches the default";
}

TEST(PortLiquid, WaterDetectedAtFeet) {
	FakeWorld world;
	world.water({ 0, 0, 0 });
	const auto body = submergedBody();

	EXPECT_TRUE(touchingLiquid(world, body, LiquidKind::Water).size() == 1) << "TestWaterDetectedAtFeet: one water block touched";
}

TEST(PortLiquid, LavaUsesWiderHorizontalMargin) {
	FakeWorld world;
	world.water({ 0, 0, 0 });
	world.lava({ 1, 0, 0 });
	auto body = submergedBody();
	body.position = { 0.75f, 0.5f, 0.5f };

	const auto water = touchingLiquid(world, body, LiquidKind::Water);
	const auto lava = touchingLiquid(world, body, LiquidKind::Lava);
	EXPECT_TRUE(!water.empty()) << "TestLavaUsesWiderHorizontalMargin: water contact";
	EXPECT_TRUE(lava.empty()) << "TestLavaUsesWiderHorizontalMargin: no lava contact due to the wider margin";
}

TEST(PortLiquid, LiquidTypeFiltering) {
	const auto world = filledColumn(LiquidKind::Lava);
	const auto body = submergedBody();

	EXPECT_TRUE(touchingLiquid(world, body, LiquidKind::Water).empty()) << "TestLiquidTypeFiltering: no water in a lava column";
	EXPECT_TRUE(!touchingLiquid(world, body, LiquidKind::Lava).empty()) << "TestLiquidTypeFiltering: lava contact";
}

TEST(PortLiquid, WaterTakesPriorityOverLava) {
	auto world = filledColumn(LiquidKind::Water);
	world.lava({ 0, 0, 0 });
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, { 0.0f, -0.005f, 0.0f })) << "TestWaterTakesPriorityOverLava: water gravity wins";
}

TEST(PortLiquid, LiquidFallsBackToBlockProvider) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, { 0.0f, -0.005f, 0.0f })) << "TestLiquidFallsBackToBlockProvider: water gravity from the liquid provider";
}

TEST(PortLiquid, LiquidProviderDetectsWaterloggedBlocks) {
	FakeWorld world;
	for (int y = 0; y < 4; ++y) {
		world.set({ 0, y, 0 }, FakeBlock{ .liquid = Liquid{ .kind = LiquidKind::Water } });
	}

	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();

	EXPECT_TRUE(!touchingLiquid(world, body, LiquidKind::Water).empty()) << "TestLiquidProviderDetectsWaterloggedBlocks: waterlogged blocks register as water";

	simulator.simulateState(body);
	EXPECT_TRUE(nearVec(body.velocity, { 0.0f, -0.005f, 0.0f })) << "TestLiquidProviderDetectsWaterloggedBlocks: water gravity in waterlogged blocks";
}

TEST(PortLiquid, NoLiquidRunsNormalPhysics) {
	const FakeWorld world;
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.gravity = kGravity;

	simulator.simulateState(body);
	EXPECT_TRUE(!near(body.velocity.y, -0.005f)) << "TestNoLiquidRunsNormalPhysics: dry world does not use liquid gravity";
	EXPECT_TRUE(near(body.velocity.y, -kGravity * kGravityDrag)) << "TestNoLiquidRunsNormalPhysics: normal gravity applies";
}

TEST(PortLiquid, UnloadedChunkCancelsLiquidSimulation) {
	auto world = filledColumn(LiquidKind::Water);
	world.loaded = [](const AABB&) {
		return false;
	};

	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.velocity = { 0.5f, 0.5f, 0.5f };

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestUnloadedChunkCancelsLiquidSimulation: unloaded outcome";
	EXPECT_TRUE(nearVec(body.velocity, {})) << "TestUnloadedChunkCancelsLiquidSimulation: velocity cancelled";
}

TEST(PortLiquid, LiquidIsReliableScenario) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome != Outcome::Unreliable) << "TestLiquidIsReliableScenario: liquid contact is reliable";
}

TEST(PortLiquid, WaterCancelsGliding) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.gliding = true;
	body.glideBoostTicks = 15;

	simulator.simulateState(body);
	EXPECT_TRUE(!body.gliding) << "TestWaterCancelsGliding: water cancels gliding";
	EXPECT_TRUE(body.glideBoostTicks == 0) << "TestWaterCancelsGliding: glide boost cleared";
}

TEST(PortLiquid, WaterPreservesGlideBoostWhenNotGliding) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.gliding = false;
	body.glideBoostTicks = 15;

	simulator.simulateState(body);
	EXPECT_TRUE(body.glideBoostTicks == 15) << "TestWaterPreservesGlideBoostWhenNotGliding: glide boost preserved";
}

TEST(PortLiquid, LavaDoesNotCancelGliding) {
	const auto world = filledColumn(LiquidKind::Lava);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.gliding = true;
	body.glideBoostTicks = 15;

	simulator.simulateState(body);
	EXPECT_TRUE(body.gliding) << "TestLavaDoesNotCancelGliding: lava keeps gliding";
	EXPECT_TRUE(body.glideBoostTicks == 15) << "TestLavaDoesNotCancelGliding: glide boost preserved";
}

TEST(PortLiquid, SwimmingPreservesWaterTravelOutsideWater) {
	const FakeWorld world;
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.swimming = true;
	body.rotation = {};
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;
	body.gravity = kGravity;

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.y, 0.0f)) << "TestSwimmingPreservesWaterTravelOutsideWater: water travel keeps zero gravity";
}

TEST(PortLiquid, SwimmingOutsideWaterSuppressesJump) {
	const FakeWorld dryWorld;
	Simulator simulator{ dryWorld, liquidOptions() };
	auto body = submergedBody();
	body.swimming = true;
	body.swimAmount = 1.0f;
	body.effectiveJumping = true;
	body.swimWaterGraceTicks = kSwimWaterGraceTicks;
	body.gravity = kGravity;

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.y, 0.0f)) << "TestSwimmingOutsideWaterSuppressesJump: jump suppressed outside water";

	const auto wetWorld = filledColumn(LiquidKind::Water);
	Simulator wet{ wetWorld, liquidOptions() };
	auto wetBody = submergedBody();
	wetBody.swimming = true;
	wetBody.swimAmount = 1.0f;
	wetBody.effectiveJumping = true;
	wet.simulateState(wetBody);
	EXPECT_TRUE(wetBody.velocity.y > 0.0f) << "TestSwimmingOutsideWaterSuppressesJump: in-water swimmer ascends";
}

TEST(PortLiquid, LiquidFlowPushesTowardLowerDepth) {
	FakeWorld world;
	world.water({ 0, 0, 0 });
	world.water({ 1, 0, 0 }, 7);
	world.water({ -1, 0, 0 });
	world.water({ 0, 0, 1 });
	world.water({ 0, 0, -1 });
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();

	simulator.simulateState(body);
	EXPECT_TRUE(body.velocity.x > 0.0f) << "TestLiquidFlowPushesTowardLowerDepth: pushed toward the shallower neighbour";
}

TEST(PortLiquid, WaterFlowStrength) {
	FakeWorld world;
	world.water({ 0, 0, 0 });
	world.water({ 1, 0, 0 }, 7);
	auto body = submergedBody();

	const auto positions = touchingLiquid(world, body, LiquidKind::Water);
	EXPECT_TRUE(applyFlow(world, body, positions, LiquidKind::Water)) << "TestWaterFlowStrength: flow area loaded";
	EXPECT_TRUE(near(body.velocity.x, 0.014f)) << "TestWaterFlowStrength: water flow strength is 0.014";
}

TEST(PortLiquid, LavaFlowStrength) {
	FakeWorld world;
	world.lava({ 0, 0, 0 });
	world.lava({ 1, 0, 0 }, 7);
	auto body = submergedBody();

	const auto positions = touchingLiquid(world, body, LiquidKind::Lava);
	EXPECT_TRUE(applyFlow(world, body, positions, LiquidKind::Lava)) << "TestLavaFlowStrength: flow area loaded";
	EXPECT_TRUE(near(body.velocity.x, 0.0035f)) << "TestLavaFlowStrength: lava flow strength is 0.0035";
}

TEST(PortLiquid, UniformLiquidHasNoFlow) {
	const auto world = filledColumn(LiquidKind::Water);
	auto body = submergedBody();

	const auto positions = touchingLiquid(world, body, LiquidKind::Water);
	EXPECT_TRUE(applyFlow(world, body, positions, LiquidKind::Water)) << "TestUniformLiquidHasNoFlow: flow area loaded";
	EXPECT_TRUE(nearVec(body.velocity, {})) << "TestUniformLiquidHasNoFlow: uniform liquid has no flow";
}

TEST(PortLiquid, FallingLiquidFlowsDownwardAlongSolids) {
	FakeWorld world;
	world.water({ 0, 0, 0 }, 8, true);
	world.solid({ 1, 0, 0 });

	const auto flow = waterFlowAt(world, { 0, 0, 0 });
	EXPECT_TRUE(flow.y < 0.0f) << "TestFallingLiquidFlowsDownwardAlongSolids: falling liquid flows downward";
}

TEST(PortLiquid, NonFallingLiquidHasNoDownwardFlow) {
	FakeWorld world;
	world.water({ 0, 0, 0 });
	world.solid({ 1, 0, 0 });

	const auto flow = waterFlowAt(world, { 0, 0, 0 });
	EXPECT_TRUE(!(flow.y < 0.0f)) << "TestNonFallingLiquidHasNoDownwardFlow: no downward push";
}

TEST(PortLiquid, SolidNeighbourBlocksFlow) {
	FakeWorld world;
	world.water({ 0, 0, 0 });
	world.solid({ 1, 0, 0 });
	world.water({ 1, -1, 0 });

	const auto flow = waterFlowAt(world, { 0, 0, 0 });
	EXPECT_TRUE(near(flow.x, 0.0f)) << "TestSolidNeighbourBlocksFlow: solid neighbour blocks flow";
}

TEST(PortLiquid, FlowFallsIntoOpenDrop) {
	FakeWorld world;
	world.water({ 0, 0, 0 });
	world.water({ 1, -1, 0 });

	const auto flow = waterFlowAt(world, { 0, 0, 0 });
	EXPECT_TRUE(flow.x > 0.0f) << "TestFlowFallsIntoOpenDrop: flow pulls into the drop";
}

TEST(PortLiquid, NegligibleFlowIgnored) {
	const auto world = filledColumn(LiquidKind::Water);
	auto body = submergedBody();
	const auto before = body.velocity;

	EXPECT_TRUE(applyFlow(world, body, {}, LiquidKind::Water)) << "TestNegligibleFlowIgnored: empty flow succeeds";
	EXPECT_TRUE(nearVec(body.velocity, before)) << "TestNegligibleFlowIgnored: velocity unchanged";
}

TEST(PortLiquid, FallingLiquidDecayAndHeight) {
	const Liquid falling{ .kind = LiquidKind::Water, .depth = 3, .falling = true };
	EXPECT_TRUE(liquidDecay(falling) == 0) << "TestFallingLiquidDecayAndHeight: falling decay is 0";
	EXPECT_TRUE(near(liquidHeight(falling), 1.0f)) << "TestFallingLiquidDecayAndHeight: falling height is 1";

	const Liquid still{ .kind = LiquidKind::Water, .depth = 8 };
	EXPECT_TRUE(liquidDecay(still) == 0) << "TestFallingLiquidDecayAndHeight: source decay is 0";
	EXPECT_TRUE(near(liquidHeight(still), 1.0f)) << "TestFallingLiquidDecayAndHeight: source height is 1";

	const Liquid shallow{ .kind = LiquidKind::Water, .depth = 1 };
	EXPECT_TRUE(liquidDecay(shallow) == 7) << "TestFallingLiquidDecayAndHeight: shallow decay is 7";
	EXPECT_TRUE(near(liquidHeight(shallow), 2.0f / 9.0f)) << "TestFallingLiquidDecayAndHeight: shallow height is 2/9";
}

TEST(PortLiquid, LiquidExitProbeBoostsOverLedge) {
	FakeWorld world;
	fillLiquid(world, { -1, 0, -1 }, { 0, 0, 1 }, LiquidKind::Water);
	world.solid({ 1, 0, 0 });
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.velocity = { 0.5f, 0.0f, 0.0f };
	body.impulse = { 0.0f, 0.98f };

	simulator.simulateState(body);
	EXPECT_TRUE(body.collideX) << "TestLiquidExitProbeBoostsOverLedge: horizontal collision against the ledge";
	EXPECT_TRUE(near(body.velocity.y, 0.3f)) << "TestLiquidExitProbeBoostsOverLedge: exit boost is 0.3";
}

TEST(PortLiquid, LiquidExitProbeBlockedByCollisionAlone) {
	for (const bool overhang : { false, true }) {
		FakeWorld world;
		fillLiquid(world, { -1, 0, -1 }, { 0, 0, 1 }, LiquidKind::Water);
		world.solid({ 1, 0, 0 });
		if (overhang) {
			world.solid({ 0, 1, 0 });
		}

		auto body = submergedBody();
		body.position = { 0.5f, 0.4f, 0.5f };
		body.client.position = body.position;
		body.swimming = true;
		body.swimWaterGraceTicks = kSwimWaterGraceTicks;
		body.velocity = { 0.5f, 0.0f, 0.0f };
		Simulator simulator{ world, liquidOptions() };

		const std::string label = std::string{ "TestLiquidExitProbeBlockedByCollisionAlone overhang=" } + (overhang ? "true" : "false");
		const auto raised = body.boundingBox(false).translate({ 0.0f, 0.6f, 0.0f });
		EXPECT_TRUE(!containsAnyLiquid(world, raised)) << label + ": probe box contains no liquid";

		simulator.simulateState(body);
		EXPECT_TRUE(body.collideX) << label + ": horizontal collision against the ledge";
		EXPECT_TRUE(!body.collideY) << label + ": hitbox does not touch the overhang";

		const bool boosted = near(body.velocity.y, 0.3f);
		if (overhang) {
			EXPECT_TRUE(!boosted) << label + ": a block above the probe denies the exit boost";
		} else {
			EXPECT_TRUE(boosted) << label + ": a clear probe grants the exit boost";
		}
	}
}

TEST(PortLiquid, LiquidExitProbeBlockedByLiquidAbove) {
	FakeWorld world;
	fillLiquid(world, { -1, 0, -1 }, { 0, 4, 1 }, LiquidKind::Water);
	world.solid({ 1, 0, 0 });
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.velocity = { 0.5f, 0.0f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(!near(body.velocity.y, 0.3f)) << "TestLiquidExitProbeBlockedByLiquidAbove: liquid above blocks the exit boost";
}

TEST(PortLiquid, NoExitProbeWithoutHorizontalCollision) {
	const auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();

	simulator.simulateState(body);
	EXPECT_TRUE(!near(body.velocity.y, 0.3f)) << "TestNoExitProbeWithoutHorizontalCollision: exit boost requires a horizontal collision";
}

TEST(PortLiquid, ClimbUsesEffectiveJumping) {
	struct Case {
		std::string_view name;
		void (*apply)(Body&);
		bool climbs;
	};

	const std::array cases{
		Case{ .name = "pressingJump", .apply = [](Body& body) {
			body.pressingJump = true;
			body.effectiveJumping = true;
		}, .climbs = true },
		Case{ .name = "autoJumpingInWater", .apply = [](Body& body) {
			body.effectiveJumping = true;
		}, .climbs = true },
		Case{ .name = "none", .apply = [](Body&) { }, .climbs = false }
	};

	for (const auto& [name, apply, climbs] : cases) {
		const ClimbableWorld world;
		Simulator simulator{ world, liquidOptions() };
		auto body = submergedBody();
		apply(body);

		simulator.simulateState(body);
		const bool climbing = near(body.velocity.y, kClimbSpeed * kGravityDrag);
		EXPECT_TRUE(climbing == climbs) << std::string{ "TestClimbUsesEffectiveJumping/" } + std::string{ name } + ": climb follows effectiveJumping";
	}
}

TEST(PortLiquid, ShrinkLiquidBoxCollapsesToMidpoint) {
	const AABB box{ { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.2f, 1.0f } };
	const auto shrunk = shrinkLiquidBox(box, { 0.001f, 0.401f, 0.001f });

	EXPECT_TRUE(near(shrunk.min.y, 0.1f) && near(shrunk.max.y, 0.1f)) << "TestShrinkLiquidBoxCollapsesToMidpoint: Y collapses to 0.1";
	EXPECT_TRUE(near(shrunk.min.x, 0.001f) && near(shrunk.max.x, 0.999f)) << "TestShrinkLiquidBoxCollapsesToMidpoint: X shrinks to [0.001 0.999]";
}

TEST(PortLiquid, BoostedDepthStriderKeepsFullAirborneAcceleration) {
	for (const bool onGround : { false, true }) {
		for (const int level : { 0, 1, 2, 3, 5 }) {
			auto world = filledColumn(LiquidKind::Water);
			world.enchantments[Enchantment::DepthStrider] = level;
			Simulator simulator{ world, liquidOptions() };
			auto body = submergedBody();
			body.swimming = true;
			body.onGround = onGround;
			body.swimSpeedMultiplier = 2.0f;
			body.impulse = { 0.0f, 1.0f };
			simulator.simulateState(body);

			const float expected = 0.02f * (0.7f + static_cast<float>(std::min(level, 3)) / 3.0f * 0.3f) * 2.0f;
			const std::string label = "TestBoostedDepthStriderKeepsFullAirborneAcceleration ground=" + std::string{ onGround ? "true" : "false" } + " level=" + std::to_string(level);
			EXPECT_TRUE(near(body.movement.z, expected) && near(body.velocity.z, expected * 0.8f)) << label + ": boosted acceleration ignores the airborne halving";
		}
	}
}

TEST(PortLiquid, DisabledSwimGraceStillUsesObservedWaterPose) {
	auto world = filledColumn(LiquidKind::Water);
	Simulator simulator{ world, liquidOptions() };
	simulator.options().swimWaterGraceTicks = -1;
	auto body = submergedBody();
	body.swimming = true;
	body.hasGravity = false;

	for (int tick = 0; tick < 2; ++tick) {
		const auto result = simulator.simulateState(body);
		const auto box = body.boundingBox(false);
		const bool wet = result.outcome == Outcome::Normal && body.swimPose() && body.swimWaterGraceTicks == 0 && near(height(box), 0.6f);
		EXPECT_TRUE(wet) << "TestDisabledSwimGraceStillUsesObservedWaterPose: wet tick " + std::to_string(tick) + " keeps the observed swim pose";
	}

	world.blocks.clear();
	simulator.simulateState(body);
	EXPECT_TRUE(!body.swimPose() && near(height(body.boundingBox(false)), 1.8f)) << "TestDisabledSwimGraceStillUsesObservedWaterPose: dry tick drops water contact and compact pose";
}

TEST(PortLiquid, ObservedSwimContactClearsOnSkippedTicks) {
	for (const std::string_view skip : { "unloaded", "immobile", "unreliable", "teleport", "mounted" }) {
		const std::string label = "TestObservedSwimContactClearsOnSkippedTicks/" + std::string{ skip };
		auto world = filledColumn(LiquidKind::Water);
		Simulator simulator{ world, liquidOptions() };
		simulator.options().swimWaterGraceTicks = -1;
		auto body = submergedBody();
		body.swimming = true;
		simulator.simulateState(body);
		EXPECT_TRUE(body.swimPose()) << label + ": establishes contact";

		if (skip == "unloaded") {
			world.blocks.clear();
			world.loaded = [](const AABB&) {
				return false;
			};
		} else if (skip == "immobile") {
			body.immobile = true;
		} else if (skip == "unreliable") {
			body.alive = false;
		} else if (skip == "teleport") {
			body.queueTeleport({ 30.0f, 5.0f, 30.0f }, false, 0);
		} else if (skip == "mounted") {
			body.inVehicle = true;
		}

		const auto result = simulator.simulateState(body);
		EXPECT_TRUE(result.outcome != Outcome::Normal && !body.swimPose()) << label + ": skipped tick clears contact";
	}
}

TEST(PortLiquid, SwimContactLossRestoresPoseBelowCeiling) {
	for (const int64_t grace : { int64_t{ -1 }, int64_t{ 1 } }) {
		auto world = filledColumn(LiquidKind::Water);
		Simulator simulator{ world, liquidOptions() };
		simulator.options().swimWaterGraceTicks = grace;
		auto body = submergedBody();
		body.swimming = true;
		body.hasGravity = false;
		simulator.simulateState(body);

		world.blocks.clear();
		for (int x = -2; x < 2; ++x) {
			for (int z = -2; z < 2; ++z) {
				world.set({ x, 1, z }, FakeBlock{ .shapes = { AABB{ { 0.0f, 0.25f, 0.0f }, { 1.0f, 1.0f, 1.0f } } } });
				world.set({ x, 2, z }, FakeBlock{ .shapes = kFullBlock });
				world.set({ x, 3, z }, FakeBlock{ .shapes = kFullBlock });
			}
		}

		const auto result = simulator.simulateState(body);
		const bool restored = result.outcome == Outcome::Normal && !body.swimPose() && body.crawling && !body.stuckInCollider;
		EXPECT_TRUE(restored) << "TestSwimContactLossRestoresPoseBelowCeiling grace=" + std::to_string(grace) + ": contact loss crawls below the ceiling";
	}
}
