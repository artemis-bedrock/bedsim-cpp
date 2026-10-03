#include "helpers.h"
#include "fake_world.h"

#include "bedsim/constants.h"
#include "bedsim/physics.h"
#include "bedsim/semantics.h"
#include "bedsim/simulator.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <new>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <gtest/gtest.h>

struct SemanticsWorld {
	bedsim::BlockMovement semantics{};

	void blockCollisions(const bedsim::BlockPos&, std::vector<bedsim::AABB>&) const { }

	[[nodiscard]] bedsim::BlockMovement movement(const bedsim::BlockPos&) const {
		return semantics;
	}

	[[nodiscard]] bool isLoaded(const bedsim::AABB&) const {
		return true;
	}
};

struct FallbackSemanticsWorld : SemanticsWorld {
	bedsim::BlockMovement fallback{};

	[[nodiscard]] bedsim::BlockMovement fallbackMovement(const bedsim::BlockPos&) const {
		return fallback;
	}
};

using namespace bedsim;
using bedsim::test::BoxWorld;
using bedsim::test::FakeWorld;
using bedsim::test::LoggedWorld;
using bedsim::test::baseBody;
using bedsim::test::kFullBlock;
using bedsim::test::near;

static size_t allocationCount = 0;

void* operator new(const size_t size) {
	++allocationCount;
	if (void* memory = std::malloc(size == 0 ? 1 : size)) {
		return memory;
	}

	throw std::bad_alloc{};
}

void operator delete(void* memory) noexcept {
	std::free(memory);
}

void operator delete(void* memory, size_t) noexcept {
	std::free(memory);
}

using ResultFromState = Result (Simulator<BoxWorld>::*)(const Body&, Outcome) const;

ResultFromState resultFromStateMember();

template <ResultFromState Member>
struct ResultFromStateAccess {
	friend ResultFromState resultFromStateMember() {
		return Member;
	}
};

template struct ResultFromStateAccess<&Simulator<BoxWorld>::resultFromState>;

[[nodiscard]] static Result resultFromState(const Simulator<BoxWorld>& simulator, const Body& body, const Outcome outcome) {
	return (simulator.*resultFromStateMember())(body, outcome);
}

[[nodiscard]] static bool containsLog(const std::vector<std::string>& logs, const std::string_view needle) {
	for (const auto& line : logs) {
		if (line.find(needle) != std::string::npos) {
			return true;
		}
	}

	return false;
}

[[nodiscard]] static bool sameMovement(const BlockMovement& left, const BlockMovement& right) {
	return left.groundFriction == right.groundFriction && left.accelerationFriction == right.accelerationFriction && left.bounce == right.bounce && left.inside == right.inside && left.traversal == right.traversal && left.air == right.air && left.climbable == right.climbable && left.cobweb == right.cobweb && left.honey == right.honey && left.fenceLike == right.fenceLike && left.soulSpeedNeutralizesFriction == right.soulSpeedNeutralizesFriction;
}

TEST(PortSimulator, CheckInsideCobwebTranslatesBlockVolume) {
	const BlockPos position{ 32, 64, -24 };
	FakeWorld world;
	world.set(position, { .name = "minecraft:web", .shapes = kFullBlock });
	Scratch scratch;
	const Options options{};
	systems::Systems<FakeWorld> systems{ world, options, scratch };
	auto body = baseBody();
	body.position = { static_cast<float>(position.x) + 0.5f, static_cast<float>(position.y), static_cast<float>(position.z) + 0.5f };
	EXPECT_TRUE(systems.blocks.insideCobweb(body)) << "TestInsideCobwebTranslatesBlockVolume: intersects the translated cobweb block volume away from the origin";
}

TEST(PortSimulator, CheckInsideCobwebUsesFullBlockVolumeWithoutCollisionBoxes) {
	const BlockPos position{ 32, 64, -24 };
	FakeWorld world;
	world.passable(position, "minecraft:web");
	Scratch scratch;
	const Options options{};
	systems::Systems<FakeWorld> systems{ world, options, scratch };
	auto body = baseBody();
	body.position = { static_cast<float>(position.x) + 0.5f, static_cast<float>(position.y), static_cast<float>(position.z) + 0.5f };
	EXPECT_TRUE(systems.blocks.insideCobweb(body)) << "TestInsideCobwebUsesFullBlockVolumeWithoutCollisionBoxes: a non-collidable cobweb occupies its full block volume";
}

TEST(PortSimulator, CheckSimulateMoveRelative) {
	const FakeWorld world;
	Simulator simulator{ world, Options{ .positionCorrectionThreshold = 0.3f, .useSlideOffset = false } };
	auto body = baseBody();
	const auto result = simulator.simulate(body, { .moveVector = { 0.0f, 1.0f } });
	EXPECT_TRUE(result.velocity.z > 0.0f) << "TestSimulateMoveRelative: forward velocity";
	EXPECT_TRUE(result.velocity.y < 0.0f) << "TestSimulateMoveRelative: gravity applies";
}

TEST(PortSimulator, CheckSimulateStateOutcomeTeleport) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.teleportPosition = { 12.0f, 63.0f, -4.0f };
	body.ticksSinceTeleport = 0;
	body.teleportCompletionTicks = 0;
	body.teleportSmoothed = false;
	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Teleport) << "TestSimulateStateOutcomeTeleport: teleport outcome";
	EXPECT_TRUE(result.position == body.teleportPosition) << "TestSimulateStateOutcomeTeleport: teleported position";
}

TEST(PortSimulator, CheckSimulateStateTeleportDoesNotUpdateFallDistance) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.0f, 70.0f, 0.0f };
	body.teleportPosition = { 0.0f, 60.0f, 0.0f };
	body.ticksSinceTeleport = 0;
	body.teleportCompletionTicks = 0;
	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Teleport) << "TestSimulateStateTeleportDoesNotUpdateFallDistance: teleport outcome";
	EXPECT_TRUE(body.fallDistance == 0.0f) << "TestSimulateStateTeleportDoesNotUpdateFallDistance: teleport does not update fall distance";
}

TEST(PortSimulator, CheckSimulateStateOutcomeUnreliable) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.gameMode = GameMode::Creative;
	body.position = { 10.0f, 70.0f, 10.0f };
	body.client.position = { 3.0f, 64.0f, -1.0f };
	body.velocity = { 0.3f, 0.9f, -0.2f };
	body.client.velocity = { -0.1f, 0.0f, 0.2f };
	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Unreliable) << "TestSimulateStateOutcomeUnreliable: unreliable outcome";
	EXPECT_TRUE(body.position == body.client.position) << "TestSimulateStateOutcomeUnreliable: position resets to the client";
	EXPECT_TRUE(body.velocity == body.client.velocity) << "TestSimulateStateOutcomeUnreliable: velocity resets to the client";
}

TEST(PortSimulator, CheckSimulateStateNoClipPassesThroughClientState) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.noClip = true;
	body.onGround = true;
	body.position = { 10.0f, 70.0f, 10.0f };
	body.client.position = { 3.0f, 64.0f, -1.0f };
	body.velocity = { 1.0f, 2.0f, 3.0f };
	body.client.velocity = { 0.1f, 0.2f, 0.3f };
	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Unreliable) << "TestSimulateStateNoClipPassesThroughClientState: no-clip passes through client state";
	EXPECT_TRUE(body.position == body.client.position) << "TestSimulateStateNoClipPassesThroughClientState: position resets to the client";
	EXPECT_TRUE(body.velocity == body.client.velocity) << "TestSimulateStateNoClipPassesThroughClientState: velocity resets to the client";
	EXPECT_TRUE(!body.onGround) << "TestSimulateStateNoClipPassesThroughClientState: on-ground clears";
}

TEST(PortSimulator, CheckUpdateFallDistanceUsesResolvedGroundState) {
	auto body = baseBody();
	body.position = { 0.0f, 10.0f, 0.0f };

	body.setPosition({ 0.0f, 7.0f, 0.0f });
	updateFallDistance(body, 10.0f);
	EXPECT_TRUE(body.fallDistance == 3.0f) << "TestUpdateFallDistanceUsesResolvedGroundState: downward move accumulates";

	body.setPosition({ 0.0f, 8.0f, 0.0f });
	updateFallDistance(body, 7.0f);
	EXPECT_TRUE(body.fallDistance == 0.0f) << "TestUpdateFallDistanceUsesResolvedGroundState: upward move resets";

	body.fallDistance = 4.0f;
	body.onGround = true;
	body.setPosition({ 0.0f, 6.0f, 0.0f });
	updateFallDistance(body, 8.0f);
	EXPECT_TRUE(body.fallDistance == 0.0f) << "TestUpdateFallDistanceUsesResolvedGroundState: grounded move clears";
}

TEST(PortSimulator, CheckSimulatorBlockSemanticsOverridesDefaults) {
	const SemanticsWorld world{ .semantics = {
		.groundFriction = 0.42f,
		.bounce = Bounce::Bed,
		.climbable = true,
		.cobweb = true
	} };
	Scratch scratch;
	const Options options{};
	const systems::Systems<SemanticsWorld> systems{ world, options, scratch };
	const auto movement = systems.context.movement({});
	EXPECT_TRUE(movement.groundFriction == 0.42f) << "TestSimulatorBlockSemanticsOverridesDefaults: semantic friction";
	EXPECT_TRUE(movement.climbable) << "TestSimulatorBlockSemanticsOverridesDefaults: semantic climbable";
	EXPECT_TRUE(movement.cobweb && movement.bounce == Bounce::Bed) << "TestSimulatorBlockSemanticsOverridesDefaults: complete semantic bundle";
}

TEST(PortSimulator, CheckSimulatorDefaultBlockSemanticsFallback) {
	const FakeWorld world;
	Scratch scratch;
	const Options options{};
	const systems::Systems<FakeWorld> systems{ world, options, scratch };
	EXPECT_TRUE(sameMovement(systems.context.movement({}), vanillaMovement("minecraft:air"))) << "TestSimulatorDefaultBlockSemanticsFallback: default movement semantics";
}

TEST(PortSimulator, CheckSimulatorInvalidBlockSemanticsFrictionFallsBackToDefault) {
	const float expected = vanillaMovement("minecraft:air").groundFriction;

	struct Case {
		std::string_view name;
		float friction;
	};

	constexpr std::array cases{
		Case{ "zero", 0.0f },
		Case{ "negative", -0.42f },
		Case{ "nan", std::numeric_limits<float>::quiet_NaN() },
		Case{ "positive infinity", std::numeric_limits<float>::infinity() },
		Case{ "negative infinity", -std::numeric_limits<float>::infinity() }
	};

	for (const auto& [name, friction] : cases) {
		const SemanticsWorld world{ .semantics = { .groundFriction = friction } };
		Scratch scratch;
		const Options options{};
		const systems::Systems<SemanticsWorld> systems{ world, options, scratch };
		const std::string message = std::string{ "TestSimulatorInvalidBlockSemanticsFrictionFallsBackToDefault/" } + std::string{ name } + ": invalid friction falls back";
		EXPECT_TRUE(systems.context.movement({}).groundFriction == expected) << message;
	}
}

TEST(PortSimulator, CheckSimulatorInvalidAccelerationMultiplierFallsBackToBuiltIn) {
	const float expected = vanillaMovement("minecraft:soul_sand").accelerationFriction;
	const FallbackSemanticsWorld world{
		{ .semantics = { .groundFriction = kBlockFriction, .accelerationFriction = 0.0f } },
		vanillaMovement("minecraft:soul_sand")
	};
	Scratch scratch;
	const Options options{};
	const systems::Systems<FallbackSemanticsWorld> systems{ world, options, scratch };
	EXPECT_TRUE(systems.context.movement({}).accelerationFriction == expected) << "TestSimulatorInvalidAccelerationMultiplierFallsBackToBuiltIn: invalid acceleration multiplier falls back to the soul sand built-in";
}

TEST(PortSimulator, CheckSimulateStateOutcomeUnloadedChunk) {
	const BoxWorld world{ .loaded = false };
	Simulator simulator{ world };
	auto body = baseBody();
	body.velocity = { 0.2f, 0.1f, -0.1f };
	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestSimulateStateOutcomeUnloadedChunk: unloaded chunk outcome";
	EXPECT_TRUE(body.velocity == Vec3{}) << "TestSimulateStateOutcomeUnloadedChunk: velocity clears";
}

TEST(PortSimulator, CheckSimulateStateOutcomeImmobileOrNotReady) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.immobile = true;
	body.velocity = { 0.5f, -0.3f, 0.5f };
	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::ImmobileOrNotReady) << "TestSimulateStateOutcomeImmobileOrNotReady: immobile outcome";
	EXPECT_TRUE(body.velocity == Vec3{}) << "TestSimulateStateOutcomeImmobileOrNotReady: velocity clears";
}

template <World W>
static void checkQueuedStuckMovementCleared(const W& world, Body body, const std::string_view message) {
	body.stuckSpeedMultiplier = { 0.8f, 0.75f, 0.8f };
	Simulator simulator{ world };
	simulator.simulateState(body);
	EXPECT_TRUE(body.stuckSpeedMultiplier == Vec3{}) << message;
}

TEST(PortSimulator, CheckEarlyExitClearsQueuedStuckMovement) {
	const BoxWorld unloaded{ .loaded = false };
	const FakeWorld world;
	checkQueuedStuckMovementCleared(unloaded, baseBody(), "TestEarlyExitClearsQueuedStuckMovement/unloaded chunk");

	auto immobile = baseBody();
	immobile.immobile = true;
	checkQueuedStuckMovementCleared(world, immobile, "TestEarlyExitClearsQueuedStuckMovement/immobile");

	auto unreliable = baseBody();
	unreliable.gameMode = GameMode::Creative;
	checkQueuedStuckMovementCleared(world, unreliable, "TestEarlyExitClearsQueuedStuckMovement/unreliable");

	auto teleport = baseBody();
	teleport.teleportCompletionTicks = 1;
	teleport.pendingTeleports = 1;
	teleport.teleportPosition = { 10.0f, 20.0f, 30.0f };
	checkQueuedStuckMovementCleared(world, teleport, "TestEarlyExitClearsQueuedStuckMovement/teleport");
}

TEST(PortSimulator, CheckSimulateStateSkipsGravityWhenDisabled) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.hasGravity = false;
	body.impulse = { 0.0f, 0.98f };
	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Normal) << "TestSimulateStateSkipsGravityWhenDisabled: normal outcome";
	EXPECT_TRUE(result.velocity.y == 0.0f) << "TestSimulateStateSkipsGravityWhenDisabled: no gravity change on Y";
}

TEST(PortSimulator, CheckSimulateStateInvalidGlideContinuesNormalMovement) {
	const FakeWorld world{ .elytra = false };
	Simulator simulator{ world };
	auto body = baseBody();
	body.gliding = true;
	body.onGround = true;
	body.impulse = { 0.0f, 0.98f };
	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Normal) << "TestSimulateStateInvalidGlideContinuesNormalMovement: normal outcome";
	EXPECT_TRUE(!body.gliding) << "TestSimulateStateInvalidGlideContinuesNormalMovement: gliding clears";
	EXPECT_TRUE(result.velocity.z > 0.0f) << "TestSimulateStateInvalidGlideContinuesNormalMovement: movement continues";
}

TEST(PortSimulator, CheckSimulateStateDebugTraceIncludesCollisionStream) {
	std::vector<std::string> logs;
	const LoggedWorld<FakeWorld> world{ {}, &logs };
	Simulator simulator{ world };
	auto body = baseBody();
	body.impulse = { 0.0f, 0.98f };
	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Normal) << "TestSimulateStateDebugTraceIncludesCollisionStream: normal outcome";

	constexpr std::array<std::string_view, 7> expected{
		"blockUnder=",
		"moveRelative force applied",
		"Y-collision non-step=",
		"(X) hz-collision non-step=",
		"(Z) hz-collision non-step=",
		"finalVel=",
		"(server) xCollision="
	};

	for (const auto needle : expected) {
		EXPECT_TRUE(containsLog(logs, needle)) << std::string{ "TestSimulateStateDebugTraceIncludesCollisionStream: logs contain " } + std::string{ needle };
	}
}

TEST(PortSimulator, CheckSimulateStateResolvesBlockedJumpThroughAutoStep) {
	std::vector<std::string> logs;
	const LoggedWorld<BoxWorld> world{ { .boxes = { AABB{ { 0.0f, 2.0f, 1.0f }, { 1.0f, 3.0f, 2.0f } } } }, &logs };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.0f, 0.0f, 0.69f };
	body.client.position = body.position;
	body.onGround = true;
	body.jumping = true;
	body.sprinting = true;
	body.rotation = {};
	body.jumpHeight = kJumpHeight;
	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Normal) << "TestSimulateStateResolvesBlockedJumpThroughAutoStep: normal outcome";
	EXPECT_TRUE(std::abs(result.position.y) <= 1e-6f && result.position.z > 0.69f) << "TestSimulateStateResolvesBlockedJumpThroughAutoStep: the filtered auto-step path moves under the block";
	EXPECT_TRUE(containsLog(logs, "auto-step collision boxes=0/1")) << "TestSimulateStateResolvesBlockedJumpThroughAutoStep: the overhead box is excluded from auto-step";
	EXPECT_TRUE(!containsLog(logs, "jump determined to be blocked")) << "TestSimulateStateResolvesBlockedJumpThroughAutoStep: no pre-emptive jump cancellation";
}

TEST(PortSimulator, CheckCalculateAutoStepExcludesBoxesAtPlayerTop) {
	const AABB original{ { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.8f, 1.0f } };
	const std::array boxes{
		AABB{ { 1.0f, 0.0f, 0.0f }, { 2.0f, 0.5f, 1.0f } },
		AABB{ { 0.0f, 1.8f, 0.0f }, { 1.0f, 2.8f, 1.0f } }
	};

	const auto result = autoStep(original, { 1.0f, 0.0f, 0.0f }, boxes, false);
	EXPECT_TRUE(result.boxCount == 1) << "TestCalculateAutoStepExcludesBoxesAtPlayerTop: only the low obstacle joins the auto-step set";
	EXPECT_TRUE(result.up.y == kStepHeight) << "TestCalculateAutoStepExcludesBoxesAtPlayerTop: the overhead box leaves the upward step unchanged";
}

TEST(PortSimulator, CheckCalculateAutoStepUsesReverseCollisionOrder) {
	const AABB original{ { -0.3f, 0.0f, -0.3f }, { 0.3f, 1.8f, 0.3f } };
	const std::array boxes{
		AABB{ { 0.2f, 0.1f, -0.2f }, { 0.6f, 0.3f, 0.1f } },
		AABB{ { -0.8f, 0.3f, -0.6f }, { 0.0f, 1.1f, 0.2f } }
	};

	const auto result = autoStep(original, { 0.4f, 0.0f, 0.2f }, boxes, false);
	EXPECT_TRUE(std::abs(result.up.x + 0.1f) <= 1e-6f) << "TestCalculateAutoStepUsesReverseCollisionOrder: reverse-order depenetration on X";
}

TEST(PortSimulator, CheckCalculateAutoStepDoesNotAllocate) {
	const AABB original{ { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.8f, 1.0f } };
	const std::vector boxes{
		AABB{ { 1.0f, 0.0f, 0.0f }, { 2.0f, 0.5f, 1.0f } },
		AABB{ { 0.0f, 1.8f, 0.0f }, { 1.0f, 2.8f, 1.0f } }
	};

	const size_t before = allocationCount;
	float total = 0.0f;
	for (int run = 0; run < 100; ++run) {
		total += autoStep(original, { 1.0f, 0.0f, 0.0f }, boxes, false).velocity.x;
	}

	EXPECT_TRUE(allocationCount == before) << "TestCalculateAutoStepDoesNotAllocate: autoStep performs no allocations";
	EXPECT_TRUE(std::isfinite(total)) << "TestCalculateAutoStepDoesNotAllocate: autoStep results are finite";
}

TEST(PortSimulator, CheckSprintStallIncludesMomentum) {
	struct Case {
		std::string_view name;
		Vec3 velocity;
		bool blocked;
	};

	const std::array cases{
		Case{ "glancing along X", { 0.1f, 0.2f, 0.02f }, false },
		Case{ "head on along Z", { 0.02f, 0.2f, 0.1f }, true }
	};

	for (const auto& [name, velocity, blocked] : cases) {
		const BoxWorld world{ .boxes = { AABB{ { -10.0f, -10.0f, 1.0f }, { 10.0f, 10.0f, 2.0f } } } };
		const Simulator simulator{ world };
		Scratch scratch;
		const Options options{};
		systems::Systems<BoxWorld> systems{ world, options, scratch };
		auto body = baseBody();
		body.setPosition({ 0.5f, 1.0f, 0.7f });
		body.setVelocity(velocity);
		body.onGround = false;
		const std::string prefix = std::string{ "TestSimulator_SprintStallIncludesMomentum/" } + std::string{ name };
		EXPECT_TRUE(systems.collision.tryCollisions(body)) << prefix + ": collision simulation available";

		const auto result = resultFromState(simulator, body, Outcome::Normal);
		EXPECT_TRUE(result.collideZ && result.sprintMovementBlocked == blocked) << prefix + ": Z collision and sprint stall";
		EXPECT_TRUE(!resultFromState(simulator, body, Outcome::UnloadedChunk).sprintMovementBlocked) << prefix + ": unavailable simulation does not publish a sprint stall";
	}
}

TEST(PortSimulator, CheckAutoStepGroundContact) {
	struct Case {
		std::string_view name;
		float velocityY;
		bool ground;
	};

	const std::array cases{
		Case{ "descending", -0.0784f, true },
		Case{ "stationary", 0.0f, false },
		Case{ "jumping", kJumpHeight, false }
	};

	for (const auto& [name, velocityY, ground] : cases) {
		const BoxWorld world{ .boxes = {
			AABB{ { -2.0f, -1.0f, -2.0f }, { 2.0f, 0.0f, 2.0f } },
			AABB{ { 1.0f, 0.0f, -2.0f }, { 2.0f, 0.5f, 2.0f } }
		} };
		Scratch scratch;
		const Options options{ .ignoreClientStepTiebreaker = true };
		systems::Systems<BoxWorld> systems{ world, options, scratch };
		auto body = baseBody();
		body.setPosition({ 0.65f, 0.0f, 0.5f });
		body.setVelocity({ 0.3f, velocityY, 0.0f });
		body.onGround = true;
		const std::string prefix = std::string{ "TestSimulator_AutoStepGroundContact/" } + std::string{ name };
		const bool stepped = systems.collision.tryCollisions(body);
		EXPECT_TRUE(stepped && std::abs(body.position.y - 0.5f) <= 1e-5f) << prefix + ": successful half-block step";
		EXPECT_TRUE(body.onGround == ground) << prefix + ": ground state follows the requested Y movement";

		body.setVelocity({ 0.1f, -0.0784f, 0.0f });
		const bool landed = systems.collision.tryCollisions(body);
		EXPECT_TRUE(landed && body.onGround) << prefix + ": the following downward collision has ground contact";
	}
}

[[nodiscard]] static Body runStepUp(std::vector<AABB> boxes, const bool ignoreTiebreaker) {
	const BoxWorld world{ .boxes = std::move(boxes) };
	Simulator simulator{ world, Options{ .positionCorrectionThreshold = 0.3f, .ignoreClientStepTiebreaker = ignoreTiebreaker } };
	const Vec3 start{ 0.5f, 0.0f, 0.5f };
	auto body = baseBody();
	body.position = start;
	body.client.position = start;
	body.onGround = true;
	body.jumpHeight = kJumpHeight;

	Input input{
		.moveVector = { 0.0f, 1.0f },
		.yaw = -90.0f,
		.headYaw = -90.0f,
		.clientPosition = start
	};

	for (int tick = 0; tick < 10; ++tick) {
		simulator.simulate(body, input);
		input.clientPosition = body.position;
		input.clientVelocity = body.velocity;
	}

	return body;
}

TEST(PortSimulator, CheckStepUpTiebreaker) {
	const AABB slab{ { 1.0f, 0.0f, -1.0f }, { 2.0f, 0.5f, 2.0f } };
	const AABB ground{ { -1.0f, -1.0f, -1.0f }, { 1.0f, 0.0f, 2.0f } };
	const AABB ceiling{ { 1.0f, 1.3f, -1.0f }, { 2.0f, 2.3f, 2.0f } };

	const auto rejected = runStepUp({ slab, ground }, false);
	EXPECT_TRUE(rejected.position.y < 0.45f) << "TestStepUpTiebreaker/rejected without flag: the tie-breaker rejects the step-up";

	const auto accepted = runStepUp({ slab, ground }, true);
	EXPECT_TRUE(accepted.position.y >= 0.45f) << "TestStepUpTiebreaker/accepted with flag: the step-up is accepted";
	EXPECT_TRUE(accepted.onGround) << "TestStepUpTiebreaker/accepted with flag: an accepted positive-height step remains grounded";

	const auto blocked = runStepUp({ slab, ground, ceiling }, true);
	EXPECT_TRUE(blocked.position.y < 0.45f) << "TestStepUpTiebreaker/blocked step still rejected with flag: a blocked step is rejected";
}

TEST(PortSimulator, CheckResultFromStateCorrectionModes) {
	const BoxWorld world;

	struct Case {
		std::string_view name;
		SimulationMode mode;
		bool positionDrift;
		bool expected;
	};

	constexpr std::array cases{
		Case{ "authoritative velocity-only drift", SimulationMode::Authoritative, false, true },
		Case{ "permissive velocity-only drift", SimulationMode::Permissive, false, false },
		Case{ "permissive position drift", SimulationMode::Permissive, true, true },
		Case{ "passive position drift", SimulationMode::Passive, true, false }
	};

	for (const auto& [name, mode, positionDrift, expected] : cases) {
		auto body = baseBody();
		if (positionDrift) {
			body.position = { 0.5f, 0.0f, 0.0f };
			body.client.position = {};
		} else {
			body.velocity = { 0.5f, 0.0f, 0.0f };
			body.client.velocity = {};
		}

		const Simulator simulator{ world, Options{ .mode = mode, .positionCorrectionThreshold = 0.1f, .velocityCorrectionThreshold = 0.1f } };
		const auto result = resultFromState(simulator, body, Outcome::Normal);
		EXPECT_TRUE(result.needsCorrection == expected) << std::string{ "TestResultFromStateCorrectionModes/" } + std::string{ name };
	}
}

TEST(PortSimulator, CheckVanillaAirSteering) {
	struct Case {
		std::string_view name;
		float yaw;
		Vec3 before;
		Vec3 after;
	};

	const std::array cases{
		Case{ "first turn", -81.40101f, { 0.3311201f, 0.0f, 0.04030281f }, { 0.32424545f, 0.0f, 0.0401424f } },
		Case{ "second turn", -73.75747f, { 0.32312074f, 0.0f, 0.050257172f }, { 0.31630123f, 0.0f, 0.052219465f } }
	};

	for (const auto& [name, yaw, before, after] : cases) {
		auto body = baseBody();
		body.rotation.z = yaw;
		body.impulse = { 0.0f, 0.98f };
		body.velocity = before;
		moveRelative(body, 0.026f);
		const auto result = body.velocity * 0.91f;
		EXPECT_TRUE(length(result - after) <= 4e-8f) << std::string{ "TestMoveRelative_VanillaAirSteering/" } + std::string{ name };
	}
}

TEST(PortSimulator, CheckJumpBoostUsesZeroBasedEffectAmplifier) {
	const FakeWorld world{ .effects = { { Effect::JumpBoost, 0 } } };
	Scratch scratch;
	const Options options{};
	systems::Systems<FakeWorld> systems{ world, options, scratch };
	auto body = baseBody();
	const auto applied = systems.inputs.apply(body, {});
	EXPECT_TRUE(applied.poseKnown) << "TestJumpBoostUsesZeroBasedEffectAmplifier: input applies with a known pose";
	EXPECT_TRUE(near(body.jumpHeight, 0.52f)) << "TestJumpBoostUsesZeroBasedEffectAmplifier: jump boost I height";
}

TEST(PortSimulator, CheckLevitationUsesZeroBasedEffectAmplifier) {
	const FakeWorld world{ .effects = { { Effect::Levitation, 0 } } };
	Simulator simulator{ world };
	auto body = baseBody();
	body.hasGravity = false;
	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.y, 0.01f)) << "TestLevitationUsesZeroBasedEffectAmplifier: levitation I velocity";
}

TEST(PortSimulator, CheckSlowFallingOnlyChangesGravityWhileDescending) {
	const FakeWorld world{ .effects = { { Effect::SlowFalling, 0 } } };
	Simulator simulator{ world };
	auto body = baseBody();
	body.velocity = { 0.0f, 0.2f, 0.0f };
	body.gravity = kGravity;
	body.slowFalling = true;
	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.y, (0.2f - kGravity) * kGravityDrag)) << "TestSlowFallingOnlyChangesGravityWhileDescending: normal gravity while ascending";
}

TEST(PortSimulator, CheckBedrockStepHeight) {
	EXPECT_TRUE(kStepHeight == 0.5625f) << "TestBedrockStepHeight: Bedrock step height";
}

TEST(PortSimulator, CheckBedBounceUsesVanillaRestitutionWithoutCap) {
	const SemanticsWorld world{ .semantics = { .bounce = Bounce::Bed } };
	Scratch scratch;
	const Options options{};
	const systems::Systems<SemanticsWorld> systems{ world, options, scratch };
	const auto under = systems.context.movement({});
	auto body = baseBody();
	body.velocity = { 0.0f, -2.0f, 0.0f };
	auto previous = body.velocity;
	landOnBlock(body, previous, under);
	EXPECT_TRUE(near(body.velocity.y, 1.5f)) << "TestBedBounceUsesVanillaRestitutionWithoutCap: bed bounce from -2";

	body.velocity = { 0.0f, -1.0f, 0.0f };
	previous = body.velocity;
	landOnBlock(body, previous, under);
	EXPECT_TRUE(near(body.velocity.y, 0.75f)) << "TestBedBounceUsesVanillaRestitutionWithoutCap: bed bounce from -1";
}

TEST(PortSimulator, CheckTinyVelocityIsNotDiscardedPrematurely) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.hasGravity = false;
	body.velocity = { 1e-7f, 0.0f, 0.0f };
	simulator.simulateState(body);
	EXPECT_TRUE(body.velocity.x != 0.0f) << "TestTinyVelocityIsNotDiscardedPrematurely: Bedrock-scale tiny velocity remains non-zero";
}

TEST(PortSimulator, CheckSlowFallingChangesGlideGravity) {
	const FakeWorld world{ .elytra = true };
	Simulator simulator{ world };
	auto body = baseBody();
	body.gliding = true;
	body.gravity = kGravity;
	body.slowFalling = true;
	body.velocity.y = -0.01f;
	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.y, -0.011025f)) << "TestSlowFallingChangesGlideGravity: slow-falling glide velocity";
}

TEST(PortSimulator, CheckSlowFallingChangesGlideGravityWhileAscending) {
	auto body = baseBody();
	body.gravity = kGravity;
	body.slowFalling = true;
	body.velocity.y = 0.2f;
	simulateGlide(body);
	EXPECT_TRUE(near(body.velocity.y, 0.19355f)) << "TestSlowFallingChangesGlideGravityWhileAscending: ascending glide uses slow-falling gravity";
}

TEST(PortSimulator, CheckGlideLimitsFallDistanceDuringShallowDescent) {
	auto body = baseBody();
	body.fallDistance = 5.0f;
	body.velocity.y = -0.4f;
	simulateGlide(body);
	EXPECT_TRUE(body.fallDistance == 1.0f) << "TestGlideLimitsFallDistanceDuringShallowDescent: shallow descent sets fall distance to 1";
}

TEST(PortSimulator, CheckGlidePreservesFallDistanceDuringFastDescent) {
	auto body = baseBody();
	body.fallDistance = 5.0f;
	body.velocity.y = -0.6f;
	simulateGlide(body);
	EXPECT_TRUE(body.fallDistance == 5.0f) << "TestGlidePreservesFallDistanceDuringFastDescent: fast descent preserves fall distance";
}

TEST(PortSimulator, CheckSneakEdgeProtectionRequiresGround) {
	const BoxWorld world{ .boxes = { AABB{ { -1.0f, -1.0f, -1.0f }, { 0.0f, 0.0f, 1.0f } } } };
	Scratch scratch;
	const Options options{};
	systems::Systems<BoxWorld> systems{ world, options, scratch };
	auto body = baseBody();
	body.sneaking = true;
	body.onGround = false;
	body.fallDistance = 0.1f;
	body.velocity = { 0.5f, 0.0f, 0.0f };
	EXPECT_TRUE(systems.collision.avoidEdge(body)) << "TestSneakEdgeProtectionRequiresGround: edge avoidance completes";
	EXPECT_TRUE(body.velocity.x == 0.5f) << "TestSneakEdgeProtectionRequiresGround: edge protection does not run while airborne";
}

TEST(PortSimulator, CheckJumpImpulse) {
	struct Case {
		std::string_view name;
		Vec3 velocity;
		float yaw;
		bool sprint;
		Vec3 expected;
	};

	const std::array cases{
		Case{ "walking", { 0.1f, 0.3f, -0.2f }, 0.0f, false, { 0.1f, 0.42f, -0.2f } },
		Case{ "sprint counters backward impulse", { 0.0f, 0.3f, -0.2f }, 0.0f, true, { 0.0f, 0.42f, 0.0f } },
		Case{ "sprint rotated", { 0.2f, 0.3f, 0.0f }, 90.0f, true, { 0.0f, 0.42f, 0.0f } },
		Case{ "strong upward impulse retained", { 0.0f, 0.7f, -0.2f }, 0.0f, true, { 0.0f, 0.7f, 0.0f } }
	};

	for (const auto& [name, velocity, yaw, sprint, expected] : cases) {
		const auto result = jumpImpulse(velocity, 0.42f, yaw, sprint);
		EXPECT_TRUE(length(result - expected) <= 1e-5f) << std::string{ "TestJumpImpulse/" } + std::string{ name };
	}
}

TEST(PortSimulator, CheckVanillaGroundAcceleration) {
	const FakeWorld world{ .fill = "minecraft:grass_block" };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 1.0f, 0.5f };
	body.onGround = true;
	body.hasGravity = false;

	constexpr std::array expected{ 0.05350801f, 0.08272339f, 0.09867499f, 0.10738456f, 0.112139985f };
	for (size_t tick = 0; tick < expected.size(); ++tick) {
		const auto result = simulator.simulate(body, { .moveVector = { 0.0f, 1.0f } });
		EXPECT_TRUE(result.velocity.z == expected[tick]) << std::string{ "TestSimulator_VanillaGroundAcceleration: tick " } + std::to_string(tick + 1) + " matches the client";
	}
}

TEST(PortSimulator, CheckVanillaSoulSandAcceleration) {
	const FakeWorld world{ .fill = "minecraft:soul_sand" };
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 0.875f, 0.5f };
	body.onGround = true;
	body.hasGravity = false;
	const auto result = simulator.simulate(body, { .moveVector = { 0.0f, 1.0f } });
	EXPECT_TRUE(result.velocity.z == 0.029107885f) << "TestSimulator_VanillaSoulSandAcceleration: soul sand velocity matches the client";
}

TEST(PortSimulator, CheckSprintMovementBlocked) {
	struct Case {
		std::string_view name;
		Vec3 requested;
		Vec3 actual;
		bool expected;
	};

	const std::array cases{
		Case{ "blocked z", { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.33f, 0.0f }, true },
		Case{ "blocked x", { -1.0f, 0.0f, 0.0f }, { 0.0f, 0.33f, 0.0f }, true },
		Case{ "wall slide", { 0.2f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.12f }, false },
		Case{ "threshold", { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.00005f }, false },
		Case{ "stationary request", {}, {}, false },
		Case{ "equal axes", { 1.0f, 0.0f, 1.0f }, {}, false }
	};

	for (const auto& [name, requested, actual, expected] : cases) {
		EXPECT_TRUE(sprintMovementBlocked(requested, actual) == expected) << std::string{ "TestSprintMovementBlocked/" } + std::string{ name };
	}
}

TEST(PortSimulator, CheckDefaultPlayerSizeMatchesPoseFallback) {
	const Vec3 size = Body{}.size;
	EXPECT_TRUE(size.x == kPlayerWidth && size.y == kPlayerHeight && size.z == 1.0f) << "TestDefaultPlayerSizeMatchesPoseFallback: default player size";

	Body body{};
	body.size = {};
	body.ensurePoseHeights();
	EXPECT_TRUE(body.standingHeight == kPlayerHeight) << "TestDefaultPlayerSizeMatchesPoseFallback: zero-size standing height is the default player height";
	EXPECT_TRUE(body.standingHeight > body.sneakingHeight && body.sneakingHeight > body.crawlingHeight) << "TestDefaultPlayerSizeMatchesPoseFallback: pose heights strictly decrease";
}

TEST(PortSimulator, CheckMovementSpeedWithoutSprint) {
	struct Case {
		std::string_view name;
		float value;
		float expected;
		std::vector<AttributeModifier> modifiers;
	};

	const std::vector<Case> cases{
		{ .name = "walking", .value = 0.1f, .expected = 0.1f },
		{ .name = "custom speed without sprint", .value = 0.13f, .expected = 0.13f },
		{
			.name = "sprinting",
			.value = 0.13f,
			.expected = 0.1f,
			.modifiers = { { .id = "d208fc00-42aa-4aad-9276-d5446530de43", .operation = ModifierOperation::MultiplyTotal, .operand = ModifierOperand::Current, .amount = 0.3f } }
		},
		{
			.name = "speed effect and sprint",
			.value = 0.156f,
			.expected = 0.120000005f,
			.modifiers = { { .id = "D208FC00-42AA-4AAD-9276-D5446530DE43", .operation = ModifierOperation::MultiplyTotal, .operand = ModifierOperand::Current, .amount = 0.3f } }
		},
		{
			.name = "name alone is not identity",
			.value = 0.13f,
			.expected = 0.13f,
			.modifiers = { { .id = "custom", .operation = ModifierOperation::MultiplyTotal, .operand = ModifierOperand::Current, .amount = 0.3f } }
		}
	};

	for (const auto& [name, value, expected, modifiers] : cases) {
		EXPECT_TRUE(movementSpeedWithoutSprint(value, modifiers) == expected) << std::string{ "TestMovementSpeedWithoutSprint/" } + std::string{ name };
	}
}
