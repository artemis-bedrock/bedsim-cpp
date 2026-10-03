#include "helpers.h"
#include "fake_world.h"

#include "bedsim/constants.h"
#include "bedsim/simulator.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <format>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <vector>
#include <gtest/gtest.h>

using namespace bedsim;
using bedsim::test::BlockPosLess;
using bedsim::test::FakeWorld;
using bedsim::test::baseBody;
using bedsim::test::kFullBlock;
using bedsim::test::near;

struct EnvironmentWorld {
	std::map<BlockPos, std::string, BlockPosLess> blocks;
	std::set<BlockPos, BlockPosLess> solids;
	bool leatherBoots{};

	void blockCollisions(const BlockPos& position, std::vector<AABB>& out) const {
		if (solids.contains(position)) {
			out.insert(out.end(), kFullBlock.begin(), kFullBlock.end());
		}
	}

	void collisions(const AABB&, std::vector<AABB>&) const { }

	[[nodiscard]] BlockMovement movement(const BlockPos& position) const {
		if (const auto block = blocks.find(position); block != blocks.end()) {
			return vanillaMovement(block->second);
		}

		return vanillaMovement(solids.contains(position) ? "minecraft:stone" : "minecraft:air");
	}

	[[nodiscard]] bool isLoaded(const AABB&) const {
		return true;
	}

	[[nodiscard]] int enchantmentLevel(const Enchantment) const {
		return 0;
	}

	[[nodiscard]] bool wearingLeatherBoots() const {
		return leatherBoots;
	}
};

struct DynamicCollisionWorld : EnvironmentWorld {
	void movementCollisions(const AABB&, const CollisionContext& context, std::vector<AABB>& out) const {
		if (context.leatherBoots && !context.descending && !context.wantDown) {
			out.push_back(AABB{ { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } });
		}
	}
};

struct ListWorld {
	std::vector<AABB> boxes;

	void blockCollisions(const BlockPos&, std::vector<AABB>&) const { }

	void collisions(const AABB&, std::vector<AABB>& out) const {
		out.insert(out.end(), boxes.begin(), boxes.end());
	}

	[[nodiscard]] BlockMovement movement(const BlockPos&) const {
		return vanillaMovement("minecraft:air");
	}

	[[nodiscard]] bool isLoaded(const AABB&) const {
		return true;
	}
};

struct StaticProbeWorld {
	std::vector<AABB> boxes;
	std::function<bool(const AABB&)> loaded;
	mutable int unknownReads{};

	void blockCollisions(const BlockPos& position, std::vector<AABB>&) const {
		if (position.x < -2) {
			++unknownReads;
		}
	}

	void collisions(const AABB& area, std::vector<AABB>& out) const {
		for (const auto& box : boxes) {
			if (area.intersects(box)) {
				out.push_back(box);
			}
		}
	}

	[[nodiscard]] BlockMovement movement(const BlockPos&) const {
		return vanillaMovement("minecraft:air");
	}

	[[nodiscard]] bool isLoaded(const AABB& area) const {
		return loaded(area);
	}
};

struct AuxiliaryProbeWorld {
	mutable int blockReads{};

	void blockCollisions(const BlockPos&, std::vector<AABB>&) const { }

	void collisions(const AABB&, std::vector<AABB>&) const { }

	[[nodiscard]] BlockMovement movement(const BlockPos&) const {
		++blockReads;
		return vanillaMovement("minecraft:air");
	}

	[[nodiscard]] bool isLoaded(const AABB& area) const {
		return area.min.x >= -0.3f && area.min.y >= 0.0f && area.min.z >= -0.3f && area.max.x <= 0.3f && area.max.y <= 1.8f && area.max.z <= 0.3f;
	}
};

static constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
static constexpr float kInfinity = std::numeric_limits<float>::infinity();

[[nodiscard]] static bool originChunkLoaded(const AABB& area) {
	const auto minimum = floor(area.min);
	const auto maximum = ceil(area.max) - Vec3{ 1.0f };
	const int minimumChunkX = static_cast<int>(minimum.x) >> 4;
	const int minimumChunkZ = static_cast<int>(minimum.z) >> 4;
	const int maximumChunkX = static_cast<int>(maximum.x) >> 4;
	const int maximumChunkZ = static_cast<int>(maximum.z) >> 4;
	return minimumChunkX == 0 && minimumChunkZ == 0 && maximumChunkX == 0 && maximumChunkZ == 0;
}

[[nodiscard]] static Body submergedBody() {
	auto body = baseBody();
	body.position = { 0.5f, 0.5f, 0.5f };
	body.client.position = body.position;
	return body;
}

[[nodiscard]] static Options liquidOptions() {
	return { .positionCorrectionThreshold = 0.3f };
}

[[nodiscard]] static bool velocityFinite(const Vec3& velocity) {
	return std::isfinite(velocity.x) && std::isfinite(velocity.y) && std::isfinite(velocity.z);
}

TEST(PortParityRegressions, CheckZeroValueStateHasNoSyntheticEvents) {
	Body body{};
	EXPECT_TRUE(!body.hasKnockback()) << "TestZeroValueStateHasNoSyntheticEvents: default body must not report knockback";
	EXPECT_TRUE(!body.hasTeleport()) << "TestZeroValueStateHasNoSyntheticEvents: default body must not report teleport";

	body.ticksSinceKnockback = 0;
	body.ticksSinceTeleport = 0;
	EXPECT_TRUE(!body.hasKnockback()) << "TestZeroValueStateHasNoSyntheticEvents: zero state must not report knockback";
	EXPECT_TRUE(!body.hasTeleport()) << "TestZeroValueStateHasNoSyntheticEvents: zero state must not report teleport";
}

TEST(PortParityRegressions, CheckSimulationRejectsNonFiniteInputAndState) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 1.0f, 2.0f, 3.0f };
	auto result = simulator.simulate(body, { .pitch = kNaN });
	EXPECT_TRUE(result.outcome == Outcome::InvalidInput && result.needsCorrection) << "TestSimulationRejectsNonFiniteInputAndState: invalid input result";
	EXPECT_TRUE((body.position == Vec3{ 1.0f, 2.0f, 3.0f })) << "TestSimulationRejectsNonFiniteInputAndState: invalid input must not mutate the position";

	body = baseBody();
	body.velocity.x = kInfinity;
	result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::InvalidInput && result.needsCorrection) << "TestSimulationRejectsNonFiniteInputAndState: invalid state result";
}

TEST(PortParityRegressions, CheckInvalidInputResultPreservesAuthoritativeState) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 12.0f, 64.0f, 9.0f };
	body.velocity = { 0.1f, 0.2f, 0.3f };
	body.movement = { 0.2f, 0.0f, 0.0f };

	const auto result = simulator.simulate(body, { .pitch = kNaN });
	EXPECT_TRUE(result.outcome == Outcome::InvalidInput) << "TestInvalidInputResultPreservesAuthoritativeState: outcome is invalid input";
	EXPECT_TRUE(result.position == body.position && result.velocity == body.velocity && result.movement == body.movement) << "TestInvalidInputResultPreservesAuthoritativeState: invalid input keeps the authoritative state";
}

TEST(PortParityRegressions, CheckPassiveModeDoesNotRequestCorrectionForInvalidInput) {
	const FakeWorld world;
	Simulator simulator{ world, { .mode = SimulationMode::Passive } };
	Body body{};
	body.velocity = { kNaN, 0.0f, 0.0f };
	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::InvalidInput && !result.needsCorrection) << "TestPassiveModeDoesNotRequestCorrectionForInvalidInput: passive invalid result needs no correction";
}

TEST(PortParityRegressions, CheckMountedStateSkipsMovement) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.inVehicle = true;
	body.position = { 10.0f, 70.0f, 10.0f };
	body.velocity = { 1.0f, 2.0f, 3.0f };
	body.client.position = { 4.0f, 5.0f, 6.0f };
	body.client.velocity = { 0.1f, 0.2f, 0.3f };
	body.onGround = true;
	body.collideX = true;
	body.collideY = true;
	body.collideZ = true;

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Mounted) << "TestMountedStateSkipsMovement: outcome is mounted";
	EXPECT_TRUE(body.position == body.client.position && body.velocity == body.client.velocity) << "TestMountedStateSkipsMovement: mounted state is reset to the client";
	EXPECT_TRUE(!body.onGround && !body.collideX && !body.collideY && !body.collideZ) << "TestMountedStateSkipsMovement: mounted reset clears contact flags";
}

TEST(PortParityRegressions, CheckMountedStateIgnoresUnknownOriginArea) {
	const FakeWorld world{ .loaded = originChunkLoaded };
	Simulator simulator{ world };
	auto body = baseBody();
	body.inVehicle = true;
	body.position = { 16.5f, 0.0f, 0.5f };
	body.client.position = body.position;

	const auto result = simulator.simulate(body, { .clientPosition = body.client.position });
	EXPECT_TRUE(result.outcome == Outcome::Mounted) << "TestMountedStateIgnoresUnknownOriginArea: mounted despite unknown origin area";
}

TEST(PortParityRegressions, CheckMountedResetClearsStaleSupport) {
	const EnvironmentWorld world{ .blocks = { { BlockPos{ 0, 0, 0 }, "minecraft:scaffolding" } } };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 1.0f, 0.5f };
	body.client.position = { 100.5f, 1.0f, 0.5f };
	body.supportingBlock = BlockPos{ 0, 0, 0 };
	body.inVehicle = true;
	body.hasGravity = false;

	simulator.simulateState(body);
	EXPECT_TRUE(!body.supportingBlock) << "TestMountedResetClearsStaleSupport: mounted reset clears the stale support";

	body.inVehicle = false;
	body.pressingDescend = true;
	simulator.simulateState(body);
	EXPECT_TRUE(body.velocity.y == 0.0f) << "TestMountedResetClearsStaleSupport: stale support must not affect later movement";
}

TEST(PortParityRegressions, CheckSimulateStateLeavesTransientInputForCaller) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.riptideReady = true;
	simulator.simulateState(body);
	EXPECT_TRUE(body.riptideReady) << "TestSimulateStateLeavesTransientInputForCaller: simulateState keeps caller-managed transient input";
}

TEST(PortParityRegressions, CheckActiveRiptideRunsOrdinaryPhysics) {
	const FakeWorld world{ .enchantments = { { Enchantment::Riptide, 2 } } };
	Simulator simulator{ world };
	auto body = baseBody();
	body.riptideTicks = 5;
	body.velocity = { 0.0f, 0.8f, 0.0f };
	body.gravity = kGravity;
	body.hasGravity = true;

	simulator.simulateState(body);
	EXPECT_TRUE(std::abs(body.velocity.y - 0.8f) > 1e-6f) << "TestActiveRiptideRunsOrdinaryPhysics: riptide tick applies gravity";
	EXPECT_TRUE(body.velocity.z == 0.0f) << "TestActiveRiptideRunsOrdinaryPhysics: riptide tick does not re-apply its launch impulse";
}

TEST(PortParityRegressions, CheckRiptideLaunchAppliesImpulseOnce) {
	const FakeWorld world{ .enchantments = { { Enchantment::Riptide, 2 } } };
	Simulator simulator{ world };
	auto body = baseBody();
	body.riptideInRain = true;
	body.riptideReady = true;
	body.startingSpinAttack = true;

	simulator.simulateState(body);
	const float launched = body.velocity.z;
	EXPECT_TRUE(near(launched, 2.25f * kAirFriction)) << "TestRiptideLaunchAppliesImpulseOnce: launch velocity decays through air friction once";

	body.riptideReady = false;
	body.startingSpinAttack = false;
	simulator.simulateState(body);
	EXPECT_TRUE(body.velocity.z <= launched) << "TestRiptideLaunchAppliesImpulseOnce: riptide gains no speed after its launch tick";
}

TEST(PortParityRegressions, CheckActiveRiptideConsumesRetainedWaterGrace) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.riptideTicks = 5;
	body.swimWaterGraceTicks = 2;

	simulator.simulateState(body);
	EXPECT_TRUE(body.swimWaterGraceTicks == 1) << "TestActiveRiptideConsumesRetainedWaterGrace: active riptide consumes one grace tick";
}

TEST(PortParityRegressions, CheckMovementSpeedUsesEffectiveAttribute) {
	auto withoutEffect = baseBody();
	withoutEffect.movementSpeed = 0.12f;
	withoutEffect.defaultMovementSpeed = 0.12f;
	withoutEffect.impulse = { 0.0f, 1.0f };
	auto withEffect = withoutEffect;

	const FakeWorld plainWorld;
	const FakeWorld effectWorld{ .effects = { { Effect::JumpBoost, 0 }, { Effect::Weaving, 0 } } };
	Simulator plainSimulator{ plainWorld };
	Simulator effectSimulator{ effectWorld };
	const auto base = plainSimulator.simulateState(withoutEffect);
	const auto withActiveEffect = effectSimulator.simulateState(withEffect);
	EXPECT_TRUE(base.velocity == withActiveEffect.velocity) << "TestMovementSpeedUsesEffectiveAttribute: active effects must not modify the effective movement speed again";
}

TEST(PortParityRegressions, CheckAirSpeedIgnoresTheMovementAttribute) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.movementSpeed = 0.2f;
	body.defaultMovementSpeed = 0.2f;

	simulator.simulate(body, { .startSprinting = true });
	EXPECT_TRUE(near(body.movementSpeed, 0.26f)) << "TestAirSpeedIgnoresTheMovementAttribute: sprinting movement speed is 0.26";
	EXPECT_TRUE(body.airSpeed == kSprintAirSpeed) << "TestAirSpeedIgnoresTheMovementAttribute: sprinting air speed is the fixed sprint air speed";
}

TEST(PortParityRegressions, CheckTeleportDoesNotApplyJumpImpulse) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.onGround = true;
	body.jumping = true;
	body.supportingBlock = BlockPos{ 7, 8, 9 };
	body.queueTeleport({ 10.0f, 20.0f, 30.0f }, false, 0);

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Teleport) << "TestTeleportDoesNotApplyJumpImpulse: outcome is teleport";
	EXPECT_TRUE(body.velocity == Vec3{}) << "TestTeleportDoesNotApplyJumpImpulse: teleport applies no jump or other velocity";
	EXPECT_TRUE(!body.hasTeleport()) << "TestTeleportDoesNotApplyJumpImpulse: completed hard teleport is not active";
	EXPECT_TRUE(!body.supportingBlock) << "TestTeleportDoesNotApplyJumpImpulse: teleport clears the stale support block";
}

TEST(PortParityRegressions, CheckQueuedTeleportEscapesUnloadedOrigin) {
	const FakeWorld world{ .loaded = originChunkLoaded };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 16.5f, 0.0f, 0.5f };
	body.client.position = body.position;
	body.queueTeleport({ 0.5f, 0.0f, 0.5f }, false, 0);

	const auto result = simulator.simulate(body, { .clientPosition = body.client.position });
	EXPECT_TRUE(result.outcome == Outcome::Teleport) << "TestQueuedTeleportEscapesUnloadedOrigin: outcome is teleport from an unloaded origin";
	EXPECT_TRUE(body.position == body.teleportPosition && !body.hasTeleport()) << "TestQueuedTeleportEscapesUnloadedOrigin: queued teleport completes";
}

TEST(PortParityRegressions, CheckQueueTeleportCanTargetOrigin) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 10.0f, 20.0f, 30.0f };
	body.queueTeleport({}, false, 0);

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Teleport && body.position == Vec3{}) << "TestQueueTeleportCanTargetOrigin: teleport reaches the origin";
}

TEST(PortParityRegressions, CheckHardTeleportAtMaximumCompletionTickFinishes) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.queueTeleport({ 10.0f, 20.0f, 30.0f }, false, std::numeric_limits<uint64_t>::max());

	const auto first = simulator.simulateState(body);
	const auto second = simulator.simulateState(body);
	EXPECT_TRUE(first.outcome == Outcome::Teleport) << "TestHardTeleportAtMaximumCompletionTickFinishes: first outcome is teleport";
	EXPECT_TRUE(!body.hasTeleport() && second.outcome != Outcome::Teleport) << "TestHardTeleportAtMaximumCompletionTickFinishes: maximum-window teleport completes";
}

TEST(PortParityRegressions, CheckLegacyTeleportFieldsCanBeRearmed) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.teleportPosition = { 1.0f, 2.0f, 3.0f };
	body.ticksSinceTeleport = 0;
	body.teleportCompletionTicks = 0;
	const auto first = simulator.simulateState(body);
	EXPECT_TRUE(first.outcome == Outcome::Teleport) << "TestLegacyTeleportFieldsCanBeRearmed: first outcome is teleport";

	body.teleportPosition = { 4.0f, 5.0f, 6.0f };
	body.ticksSinceTeleport = 0;
	body.teleportCompletionTicks = 0;
	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Teleport && body.position == body.teleportPosition) << "TestLegacyTeleportFieldsCanBeRearmed: rearmed teleport reaches its target";
}

TEST(PortParityRegressions, CheckLegacyPendingTeleportKeepsExplicitTarget) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.pendingTeleports = 1;
	body.teleportPosition = { 10.0f, 20.0f, 30.0f };

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Teleport && body.position == body.teleportPosition) << "TestLegacyPendingTeleportKeepsExplicitTarget: legacy teleport keeps its explicit target";
}

TEST(PortParityRegressions, CheckLegacySmoothedPendingTeleportKeepsExplicitTarget) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.pendingTeleports = 1;
	body.teleportPosition = { 8.0f, 0.0f, 0.0f };
	body.teleportCompletionTicks = 2;
	body.ticksSinceTeleport = 0;
	body.teleportSmoothed = true;
	const auto target = body.teleportPosition;

	for (int tick = 0; tick < 3; ++tick) {
		const auto result = simulator.simulate(body, {});
		EXPECT_TRUE(result.outcome == Outcome::Teleport) << std::format("TestLegacySmoothedPendingTeleportKeepsExplicitTarget: tick {} outcome is teleport", tick);
		EXPECT_TRUE(body.teleportPosition == target) << std::format("TestLegacySmoothedPendingTeleportKeepsExplicitTarget: tick {} keeps the target", tick);
	}

	EXPECT_TRUE(body.position == target && !body.hasTeleport()) << "TestLegacySmoothedPendingTeleportKeepsExplicitTarget: smoothed teleport completes";
}

TEST(PortParityRegressions, CheckGlideAtVerticalPitchRemainsFinite) {
	const FakeWorld world{ .elytra = true };
	Simulator simulator{ world };
	auto body = baseBody();
	body.gliding = true;
	body.onGround = false;
	body.rotation = { -90.0f, 0.0f, 0.0f };
	body.velocity = { 1.0f, 0.0f, 0.0f };

	simulator.simulateState(body);
	EXPECT_TRUE(velocityFinite(body.velocity)) << "TestGlideAtVerticalPitchRemainsFinite: glide velocity stays finite";
}

TEST(PortParityRegressions, CheckGlideNearVerticalPitchDoesNotExplode) {
	const FakeWorld world{ .elytra = true };
	Simulator simulator{ world };
	auto body = baseBody();
	body.gliding = true;
	body.onGround = false;
	body.rotation = { -89.999f, 0.0f, 0.0f };
	body.velocity = { 1.0f, 0.0f, 0.0f };

	simulator.simulateState(body);
	for (int axis = 0; axis < 3; ++axis) {
		const float value = body.velocity[axis];
		EXPECT_TRUE(std::isfinite(value) && std::abs(value) <= 10.0f) << std::format("TestGlideNearVerticalPitchDoesNotExplode: velocity axis {} = {}", axis, value);
	}
}

TEST(PortParityRegressions, CheckShallowLiquidBelowPlayerIsNotContact) {
	FakeWorld world;
	world.water({ 0, 0, 0 }, 0);
	const auto options = liquidOptions();
	Scratch scratch;
	const systems::Systems<FakeWorld> systems{ world, options, scratch };
	const auto body = submergedBody();

	std::vector<BlockPos> touching;
	systems.liquids.touchingBlocks(body, LiquidKind::Water, touching);
	EXPECT_TRUE(touching.empty()) << "TestShallowLiquidBelowPlayerIsNotContact: shallow liquid below the box is not contact";
}

TEST(PortParityRegressions, CheckMovementChecksSweptChunks) {
	const FakeWorld world{ .loaded = originChunkLoaded };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 15.5f, 0.0f, 0.5f };
	body.velocity = { 1.0f, 0.0f, 0.0f };

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestMovementChecksSweptChunks: swept movement into an unloaded chunk";
}

TEST(PortParityRegressions, CheckMovementPreflightsAuxiliaryWorldProbes) {
	const AuxiliaryProbeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.hasGravity = false;

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestMovementPreflightsAuxiliaryWorldProbes: unknown auxiliary probes are unloaded";
	EXPECT_TRUE(world.blockReads == 0) << std::format("TestMovementPreflightsAuxiliaryWorldProbes: unknown auxiliary area was read {} times", world.blockReads);
}

TEST(PortParityRegressions, CheckMovementPreflightsTranslatedSupportFallback) {
	const StaticProbeWorld world{
		.boxes = { AABB{ { 1.0f, -1.0f, -1.0f }, { 2.0f, 2.0f, 1.0f } } },
		.loaded = [](const AABB& area) {
			return area.min.x >= -2.0f;
		}
	};

	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.client.position = body.position;
	body.velocity = { 20.0f, 0.0f, 0.0f };
	body.onGround = true;
	body.hasGravity = false;

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestMovementPreflightsTranslatedSupportFallback: unknown support fallback is unloaded";
	EXPECT_TRUE(world.unknownReads == 0) << std::format("TestMovementPreflightsTranslatedSupportFallback: translated support fallback made {} unknown reads", world.unknownReads);
}

TEST(PortParityRegressions, CheckMovementChecksStepProbeArea) {
	const StaticProbeWorld world{
		.boxes = { AABB{ { 1.0f, 0.0f, 0.0f }, { 2.0f, 0.5f, 1.0f } } },
		.loaded = [](const AABB& area) {
			return area.max.y <= 1.81f;
		}
	};

	Simulator simulator{ world, { .ignoreClientStepTiebreaker = true } };
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.client.position = body.position;
	body.velocity = { 1.0f, 0.0f, 0.0f };
	body.onGround = true;
	body.hasGravity = false;

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestMovementChecksStepProbeArea: unknown step probe is unloaded";
	EXPECT_TRUE((body.position == Vec3{ 0.5f, 0.0f, 0.5f })) << "TestMovementChecksStepProbeArea: unknown step probe does not move the body";
}

TEST(PortParityRegressions, CheckMovementChecksSneakEdgeProbeArea) {
	const FakeWorld world{
		.loaded = [](const AABB& area) {
			return area.min.y >= -0.1f;
		}
	};

	Simulator simulator{ world };
	auto body = baseBody();
	body.sneaking = true;
	body.onGround = true;
	body.hasGravity = false;
	body.velocity = { 0.2f, 0.0f, 0.0f };

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestMovementChecksSneakEdgeProbeArea: unknown sneak-edge probe is unloaded";
}

TEST(PortParityRegressions, CheckMovementChecksLiquidExitProbeArea) {
	FakeWorld world{
		.loaded = [](const AABB& area) {
			return area.max.y <= 2.5f;
		}
	};

	for (int x = -1; x <= 0; ++x) {
		for (int y = 0; y <= 2; ++y) {
			for (int z = -1; z <= 1; ++z) {
				world.water({ x, y, z });
			}
		}
	}

	for (int y = 0; y < 3; ++y) {
		world.solid({ 1, y, 0 });
	}

	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.hasGravity = false;
	body.velocity = { 1.0f, 0.0f, 0.0f };

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestMovementChecksLiquidExitProbeArea: unknown liquid-exit probe is unloaded";
}

TEST(PortParityRegressions, CheckMovementChecksLiquidFlowProbeArea) {
	FakeWorld world{
		.loaded = [](const AABB& area) {
			return area.max.x < 16.0f;
		}
	};

	world.water({ 15, 0, 0 });
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.position = { 15.5f, 0.5f, 0.5f };
	body.client.position = body.position;
	body.hasGravity = false;

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestMovementChecksLiquidFlowProbeArea: unknown liquid-flow probe is unloaded";
}

TEST(PortParityRegressions, CheckMovementChecksTargetPoseArea) {
	const StaticProbeWorld world{
		.boxes = { AABB{ { -1.0f, 0.7f, -1.0f }, { 1.0f, 1.8f, 1.0f } } },
		.loaded = [](const AABB& area) {
			return area.max.y <= 0.7f;
		}
	};

	Simulator simulator{ world };
	auto body = baseBody();
	body.crawlingHeight = 0.6f;
	body.crawling = true;
	body.size.y = body.crawlingHeight;

	const auto result = simulator.simulate(body, { .stopCrawling = true });
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestMovementChecksTargetPoseArea: unknown target pose is unloaded";
	EXPECT_TRUE(body.crawling && body.size.y == body.crawlingHeight) << "TestMovementChecksTargetPoseArea: unknown target pose is not committed";
}

TEST(PortParityRegressions, CheckImmobileMovementDoesNotCheckUnappliedSweep) {
	const FakeWorld world{ .loaded = originChunkLoaded };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 15.5f, 0.0f, 0.5f };
	body.client.position = body.position;
	body.velocity = { 1.0f, 0.0f, 0.0f };
	body.immobile = true;
	body.riptideReady = true;

	const auto result = simulator.simulate(body, { .clientPosition = body.position });
	EXPECT_TRUE(result.outcome == Outcome::ImmobileOrNotReady) << "TestImmobileMovementDoesNotCheckUnappliedSweep: outcome is immobile or not ready";
	EXPECT_TRUE(body.velocity == Vec3{}) << "TestImmobileMovementDoesNotCheckUnappliedSweep: immobile state drops stale velocity";
	EXPECT_TRUE(!body.riptideReady && body.ticksSinceKnockback == 2) << "TestImmobileMovementDoesNotCheckUnappliedSweep: immobile tick advances transient state";
}

TEST(PortParityRegressions, CheckLegacySprintTransitionUpdatesSpeedOnUnloadedTick) {
	const FakeWorld world{ .loaded = originChunkLoaded };
	Simulator simulator{ world, { .sprintTiming = SprintTiming::Legacy } };
	auto body = baseBody();
	body.position = { 16.5f, 0.0f, 0.5f };
	body.client.position = body.position;

	const auto result = simulator.simulate(body, { .clientPosition = body.position, .startSprinting = true });
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestLegacySprintTransitionUpdatesSpeedOnUnloadedTick: outcome is unloaded chunk";
	EXPECT_TRUE(body.sprinting && near(body.movementSpeed, 0.13f)) << "TestLegacySprintTransitionUpdatesSpeedOnUnloadedTick: legacy sprint transition keeps state synchronized";
}

TEST(PortParityRegressions, CheckQueuedKnockbackChecksSweptChunks) {
	const FakeWorld world{ .loaded = originChunkLoaded };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 15.5f, 0.0f, 0.5f };
	body.client.position = body.position;
	body.queueKnockback({ 1.0f, 0.0f, 0.0f });

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestQueuedKnockbackChecksSweptChunks: queued knockback into an unloaded chunk";
	EXPECT_TRUE(body.position == body.client.position) << "TestQueuedKnockbackChecksSweptChunks: queued knockback does not move into the unloaded area";
}

TEST(PortParityRegressions, CheckInputAccelerationChecksSweptChunks) {
	const FakeWorld world{ .loaded = originChunkLoaded };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 15.69f, 0.0f, 0.5f };
	body.client.position = body.position;
	body.hasGravity = false;

	const auto result = simulator.simulate(body, { .moveVector = { 1.0f, 0.0f } });
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestInputAccelerationChecksSweptChunks: same-tick acceleration into an unloaded chunk";
}

TEST(PortParityRegressions, CheckRiptideLaunchChecksSweptChunks) {
	const FakeWorld world{ .enchantments = { { Enchantment::Riptide, 2 } }, .loaded = originChunkLoaded };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 15.5f };
	body.client.position = body.position;
	body.riptideInRain = true;
	body.riptideReady = true;

	const auto result = simulator.simulate(body, { .startSpinAttack = true });
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestRiptideLaunchChecksSweptChunks: riptide launch into an unloaded chunk";
	EXPECT_TRUE(body.riptideTicks == 0 && body.riptideReady && body.startingSpinAttack) << "TestRiptideLaunchChecksSweptChunks: unloaded riptide keeps the launch state";
	EXPECT_TRUE(body.ticksSinceKnockback == 1) << "TestRiptideLaunchChecksSweptChunks: unloaded riptide does not advance tick counters";
}

TEST(PortParityRegressions, CheckRiptideLaunchRetriesAfterUnloadedTick) {
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 15.5f };
	body.client.position = body.position;
	body.riptideInRain = true;
	body.riptideReady = true;

	const FakeWorld unloadedWorld{ .enchantments = { { Enchantment::Riptide, 2 } }, .loaded = originChunkLoaded };
	Simulator unloadedSimulator{ unloadedWorld };
	const auto first = unloadedSimulator.simulate(body, { .clientPosition = body.client.position, .startSpinAttack = true });
	EXPECT_TRUE(first.outcome == Outcome::UnloadedChunk) << "TestRiptideLaunchRetriesAfterUnloadedTick: first outcome is unloaded chunk";

	const FakeWorld loadedWorld{ .enchantments = { { Enchantment::Riptide, 2 } } };
	Simulator loadedSimulator{ loadedWorld };
	const auto second = loadedSimulator.simulate(body, { .clientPosition = body.client.position });
	EXPECT_TRUE(second.outcome == Outcome::Normal && body.riptideTicks != 0) << "TestRiptideLaunchRetriesAfterUnloadedTick: retried launch succeeds";
	EXPECT_TRUE(!body.riptideReady && !body.startingSpinAttack) << "TestRiptideLaunchRetriesAfterUnloadedTick: successful retry clears the pending launch";
}

TEST(PortParityRegressions, CheckRiptideHeadProbeRequiresLoadedArea) {
	int headProbes = 0;
	FakeWorld world{
		.enchantments = { { Enchantment::Riptide, 2 } },
		.loaded = [&headProbes](const AABB& area) {
			if (area.min.y == 0.0f && area.max.y == 1.0f && area.min.x == 0.0f && area.max.x == 1.0f && area.min.z == 0.0f && area.max.z == 1.0f) {
				++headProbes;
				return false;
			}

			return true;
		}
	};

	world.water({ 0, 0, 0 });
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.client.position = body.position;
	body.crawlingHeight = 0.6f;
	body.crawling = true;
	body.size.y = body.crawlingHeight;
	body.onGround = true;
	body.hasGravity = true;
	body.riptideReady = true;
	body.startingSpinAttack = true;

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestRiptideHeadProbeRequiresLoadedArea: unknown riptide head probe is unloaded";
	EXPECT_TRUE(headProbes == 1) << std::format("TestRiptideHeadProbeRequiresLoadedArea: head probe checks = {}, want 1", headProbes);
	EXPECT_TRUE(body.riptideTicks == 0 && body.riptideReady && body.startingSpinAttack) << "TestRiptideHeadProbeRequiresLoadedArea: unknown head probe keeps the launch state";
}

TEST(PortParityRegressions, CheckIneligibleRiptideSkipsHeadProbe) {
	int headProbes = 0;
	const FakeWorld world{
		.loaded = [&headProbes](const AABB& area) {
			if (area.min.y == 0.0f && area.max.y == 1.0f && area.min.x == 0.0f && area.max.x == 1.0f && area.min.z == 0.0f && area.max.z == 1.0f) {
				++headProbes;
				return false;
			}

			return true;
		}
	};

	Simulator simulator{ world };
	auto body = baseBody();
	body.crawlingHeight = 0.6f;
	body.crawling = true;
	body.size.y = body.crawlingHeight;
	body.hasGravity = false;

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Normal) << "TestIneligibleRiptideSkipsHeadProbe: normal movement without riptide";
	EXPECT_TRUE(headProbes == 0) << std::format("TestIneligibleRiptideSkipsHeadProbe: ineligible riptide performed {} head probes", headProbes);
}

TEST(PortParityRegressions, CheckCompletedTeleportCounterAdvancesOnce) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.queueTeleport({ 10.0f, 20.0f, 30.0f }, false, 0);

	const auto result = simulator.simulate(body, {});
	EXPECT_TRUE(result.outcome == Outcome::Teleport) << "TestCompletedTeleportCounterAdvancesOnce: outcome is teleport";
	EXPECT_TRUE(body.ticksSinceTeleport == 1) << std::format("TestCompletedTeleportCounterAdvancesOnce: tick counter = {}, want 1", body.ticksSinceTeleport);
}

TEST(PortParityRegressions, CheckMovementRejectsOutOfRangeSweep) {
	const FakeWorld world{ .loaded = originChunkLoaded };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { std::numeric_limits<float>::max(), 0.0f, 0.0f };

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestMovementRejectsOutOfRangeSweep: out-of-range sweep is unloaded";
}

TEST(PortParityRegressions, CheckMovementAreaProviderCannotApproveUnsafeVolume) {
	struct UnsafeVolume {
		std::string_view name;
		AABB area;
	};

	constexpr float maximum = std::numeric_limits<float>::max();
	const std::array volumes{
		UnsafeVolume{ "coordinate", AABB{ { 0.0f, 0.0f, 0.0f }, { maximum, 1.0f, 1.0f } } },
		UnsafeVolume{ "height", AABB{ { 0.0f, 0.0f, 0.0f }, { 1.0f, maximum, 1.0f } } }
	};

	const FakeWorld world;
	const Options options{};
	Scratch scratch;
	const systems::Systems<FakeWorld> systems{ world, options, scratch };
	for (const auto& [name, area] : volumes) {
		EXPECT_TRUE(!systems.context.areaLoaded(area)) << std::format("TestMovementAreaProviderCannotApproveUnsafeVolume/{}: provider approved an unsafe volume", name);
	}
}

TEST(PortParityRegressions, CheckUnloadedTickDoesNotCommitPoseChanges) {
	const FakeWorld world{ .loaded = originChunkLoaded };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 16.5f, 0.0f, 0.5f };
	body.swimming = true;
	body.size.y = body.standingHeight;
	const auto originalSize = body.size;

	const auto result = simulator.simulate(body, { .stopSwimming = true });
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestUnloadedTickDoesNotCommitPoseChanges: outcome is unloaded chunk";
	EXPECT_TRUE(body.swimming && body.size == originalSize) << "TestUnloadedTickDoesNotCommitPoseChanges: unloaded tick does not commit the pose change";
}

TEST(PortParityRegressions, CheckAdjacentClimbableIsNotContact) {
	const EnvironmentWorld world{ .blocks = { { BlockPos{ 1, 0, 0 }, "minecraft:ladder" } } };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.8f, 0.0f, 0.5f };
	body.client.position = body.position;
	body.effectiveJumping = true;
	body.gravity = kGravity;

	simulator.simulateState(body);
	EXPECT_TRUE(std::abs(body.velocity.y - kClimbSpeed) >= 1e-6f) << "TestAdjacentClimbableIsNotContact: an overlapped neighbouring ladder is not climbable contact";
}

TEST(PortParityRegressions, CheckClimbableContactResetsFallDistance) {
	const EnvironmentWorld world{ .blocks = { { BlockPos{ 0, 0, 0 }, "minecraft:ladder" } } };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.client.position = body.position;
	body.velocity = { 0.0f, -0.1f, 0.0f };
	body.fallDistance = 4.0f;
	body.hasGravity = false;

	simulator.simulateState(body);
	EXPECT_TRUE(body.fallDistance == 0.0f) << "TestClimbableContactResetsFallDistance: climbable contact resets fall distance";
}

TEST(PortParityRegressions, CheckStandingOnClimbableBlockDoesNotEnableClimbing) {
	const BlockPos position{ 0, -1, 0 };
	const EnvironmentWorld world{
		.blocks = { { position, "minecraft:ladder" } },
		.solids = { position }
	};

	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.effectiveJumping = true;

	simulator.simulateState(body);
	EXPECT_TRUE(std::abs(body.velocity.y - kClimbSpeed) >= 1e-6f) << "TestStandingOnClimbableBlockDoesNotEnableClimbing: standing on a climbable block does not climb";
}

TEST(PortParityRegressions, CheckClimbableBlockBelowIsNotContact) {
	const EnvironmentWorld world{ .blocks = { { BlockPos{ 0, -1, 0 }, "minecraft:ladder" } } };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.client.position = body.position;
	body.effectiveJumping = true;
	body.gravity = kGravity;

	simulator.simulateState(body);
	EXPECT_TRUE(std::abs(body.velocity.y - kClimbSpeed) >= 1e-6f) << "TestClimbableBlockBelowIsNotContact: a ladder below the player is not climbable contact";
}

TEST(PortParityRegressions, CheckPowderSnowSupportDoesNotEnableTraversal) {
	const BlockPos position{ 0, 0, 0 };
	DynamicCollisionWorld world;
	world.blocks = { { position, "minecraft:powder_snow" } };
	world.solids = { position };
	world.leatherBoots = true;

	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 1.0f, 0.5f };
	body.onGround = true;
	body.pressingAscend = true;

	simulator.simulateState(body);
	EXPECT_TRUE(std::abs(body.velocity.y - 0.2f) >= 1e-6f) << "TestPowderSnowSupportDoesNotEnableTraversal: powder snow below the player does not enable traversal";
}

TEST(PortParityRegressions, CheckDynamicCollisionProviderKeepsStaticSupportFallback) {
	const BlockPos position{ 0, 0, 0 };
	DynamicCollisionWorld world;
	world.solids = { position };

	const Options options{};
	Scratch scratch;
	const systems::Systems<DynamicCollisionWorld> systems{ world, options, scratch };
	auto body = baseBody();
	body.position = { 0.5f, 1.0f, 0.5f };
	body.onGround = true;

	const bool known = systems.collision.tryCollisions(body);
	EXPECT_TRUE(known) << "TestDynamicCollisionProviderKeepsStaticSupportFallback: support probe is known";
	EXPECT_TRUE(body.supportingBlock && *body.supportingBlock == position) << "TestDynamicCollisionProviderKeepsStaticSupportFallback: dynamic collision provider keeps the static support block";
}

TEST(PortParityRegressions, CheckFilteredCollisionBoxesPreserveProviderOrder) {
	const AABB first{ { 2.0f, 0.0f, 0.0f }, { 3.0f, 1.0f, 1.0f } };
	const AABB second{ { 1.0f, 0.0f, 0.0f }, { 2.0f, 1.0f, 1.0f } };
	const ListWorld world{ .boxes = { first, AABB{ { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 1.0f } }, second } };

	const Options options{};
	Scratch scratch;
	const systems::Systems<ListWorld> systems{ world, options, scratch };
	const auto body = baseBody();
	std::vector<AABB> boxes;
	systems.collision.nearbyBoxes(body, body.boundingBox(false), boxes);
	EXPECT_TRUE(boxes.size() == 2 && boxes[0] == first && boxes[1] == second) << "TestFilteredCollisionBoxesPreserveProviderOrder: filtering keeps the provider order";
}

TEST(PortParityRegressions, CheckCollisionPresenceFiltersInvalidBoxes) {
	const ListWorld world{ .boxes = { AABB{ { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 1.0f } } } };
	const Options options{};
	Scratch scratch;
	const systems::Systems<ListWorld> systems{ world, options, scratch };
	const auto body = baseBody();
	EXPECT_TRUE(!systems.collision.hasNearbyBoxes(body, body.boundingBox(false))) << "TestCollisionPresenceFiltersInvalidBoxes: a zero-volume collision box is not present";
}
