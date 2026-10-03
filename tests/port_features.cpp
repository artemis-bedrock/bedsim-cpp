#include "helpers.h"
#include "fake_world.h"

#include "bedsim/constants.h"
#include "bedsim/physics.h"
#include "bedsim/simulator.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <format>
#include <functional>
#include <limits>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>
#include <gtest/gtest.h>

using namespace bedsim;
using bedsim::test::BoxWorld;
using bedsim::test::FakeWorld;
using bedsim::test::baseBody;
using bedsim::test::kFullBlock;
using bedsim::test::near;

using BodyBytes = std::array<std::byte, sizeof(Body)>;

[[nodiscard]] static bool approxEqual(const float value, const float expected) {
	return std::abs(value - expected) < 1e-6f;
}

[[nodiscard]] static float boxHeight(const AABB& box) {
	return box.max.y - box.min.y;
}

[[nodiscard]] static BodyBytes bodyBytes(const Body& body) {
	BodyBytes bytes{};
	std::memcpy(bytes.data(), &body, sizeof(Body));
	return bytes;
}

template <World W>
static systems::AppliedInput applyInput(const W& world, Body& body, const Input& input, const Options& options = {}) {
	Scratch scratch;
	systems::Systems<W> systems{ world, options, scratch };
	return systems.inputs.apply(body, input);
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

[[nodiscard]] static FakeWorld filledColumn() {
	FakeWorld world;
	for (int x = -2; x <= 2; ++x) {
		for (int y = 0; y <= 3; ++y) {
			for (int z = -2; z <= 2; ++z) {
				world.water({ x, y, z });
			}
		}
	}

	return world;
}

TEST(PortFeatures, CheckHorizontalFlightPosePassesUnderCeiling) {
	const BoxWorld world{ .boxes = { AABB{ { -1.0f, 0.75f, 0.8f }, { 1.0f, 2.0f, 3.0f } } } };
	for (const bool spin : { false, true }) {
		auto body = baseBody();
		body.gliding = !spin;
		if (spin) {
			body.riptideTicks = 10;
		}

		body.velocity = { 0.0f, 0.0f, 1.0f };
		const Options options{};
		Scratch scratch;
		systems::Systems<BoxWorld> systems{ world, options, scratch };
		const bool known = systems.collision.tryCollisions(body);
		EXPECT_TRUE(known && !body.collideZ && body.position.z == 1.0f) << std::format("TestHorizontalFlightPosePassesUnderCeiling spin={}: compact flight is not blocked by the overhead block", spin);

		for (const auto& box : { body.boundingBox(false), body.clientBoundingBox(false) }) {
			EXPECT_TRUE(approxEqual(boxHeight(box), 0.6f)) << std::format("TestHorizontalFlightPosePassesUnderCeiling spin={}: compact flight box height is 0.6", spin);
		}

		body.size.z = 2.0f;
		EXPECT_TRUE(approxEqual(boxHeight(body.boundingBox(false)), 1.2f)) << std::format("TestHorizontalFlightPosePassesUnderCeiling spin={}: compact flight height follows scale", spin);
	}
}

TEST(PortFeatures, CheckHorizontalFlightExitFitsCeiling) {
	struct Ceiling {
		std::string_view name;
		float height;
		float want;
	};

	constexpr std::array exits{ std::string_view{ "stop glide" }, std::string_view{ "invalid glide" }, std::string_view{ "stop spin" }, std::string_view{ "spin expires" }, std::string_view{ "spin collision" } };
	constexpr std::array ceilings{ Ceiling{ "standing", 3.0f, 1.8f }, Ceiling{ "sneaking", 1.6f, 1.49f }, Ceiling{ "crawling", 0.75f, 0.6f } };
	for (const auto exit : exits) {
		for (const auto& [name, height, want] : ceilings) {
			const auto label = std::format("TestHorizontalFlightExitFitsCeiling/{}/{}", exit, name);
			auto body = baseBody();
			body.hasGravity = false;
			const BoxWorld world{ .boxes = { AABB{ { -2.0f, height, -2.0f }, { 2.0f, 4.0f, 2.0f } } } };
			Simulator simulator{ world };
			Result result{};
			if (exit == "stop glide") {
				body.gliding = true;
				result = simulator.simulate(body, { .stopGliding = true });
			} else if (exit == "invalid glide") {
				body.gliding = true;
				result = simulator.simulateState(body);
			} else if (exit == "stop spin") {
				body.riptideTicks = 10;
				body.riptideCollision = true;
				result = simulator.simulate(body, { .stopSpinAttack = true });
			} else if (exit == "spin expires") {
				body.riptideTicks = 1;
				result = simulator.simulate(body, {});
			} else {
				body.ensurePoseHeights();
				body.riptideTicks = 10;
				body.collideX = true;
				const Options options{};
				Scratch scratch;
				systems::Systems<BoxWorld> systems{ world, options, scratch };
				EXPECT_TRUE(systems.poses.stopRiptideOnBlockCollision(body)) << label + ": block collision is known";
				result.outcome = Outcome::Normal;
			}

			EXPECT_TRUE(result.outcome == Outcome::Normal) << label + ": outcome is normal";

			const auto box = body.boundingBox(false);
			EXPECT_TRUE(!body.gliding && body.riptideTicks == 0 && approxEqual(boxHeight(box), want)) << std::format("{}: exit pose glide={} spin={} height={} want {}", label, body.gliding, body.riptideTicks, boxHeight(box), want);
			EXPECT_TRUE(body.crawling == (name == "crawling") && body.sneaking == (name == "sneaking")) << std::format("{}: pose flags crawling={} sneaking={}", label, body.crawling, body.sneaking);
		}
	}
}

TEST(PortFeatures, CheckHorizontalFlightExitWaitsForLoadedWorld) {
	const BoxWorld world{ .loaded = false };
	for (const bool spin : { false, true }) {
		auto body = baseBody();
		body.gliding = !spin;
		const Input input{ .stopGliding = !spin, .stopSpinAttack = spin };
		if (spin) {
			body.riptideTicks = 10;
			body.riptideCollision = true;
		}

		Simulator simulator{ world };
		const auto result = simulator.simulate(body, input);
		const bool spinKept = !spin || body.riptideTicks == 10;
		EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk && body.gliding == !spin && spinKept) << std::format("TestHorizontalFlightExitWaitsForLoadedWorld spin={}: unknown world keeps the flight pose", spin);
	}
}

TEST(PortFeatures, CheckSoulSpeedSkipsSoulSandSlowdown) {
	FakeWorld world;
	world.passable({ 0, 0, 0 }, "minecraft:soul_sand");
	auto base = baseBody();
	base.position = { 0.5f, 1.0f, 0.5f };
	base.onGround = true;
	base.hasGravity = false;

	auto without = base;
	Simulator plain{ world };
	plain.simulate(without, { .moveVector = { 0.0f, 1.0f } });

	FakeWorld enchanted = world;
	enchanted.enchantments[Enchantment::SoulSpeed] = 1;
	auto with = base;
	Simulator soulSpeed{ enchanted };
	soulSpeed.simulate(with, { .moveVector = { 0.0f, 1.0f } });

	EXPECT_TRUE(with.velocity.z > without.velocity.z) << std::format("TestSoulSpeedSkipsSoulSandSlowdown: with={} without={}", with.velocity.z, without.velocity.z);
}

TEST(PortFeatures, CheckSwiftSneakAppliesAfterTwoSlowdownTicks) {
	const FakeWorld world{ .enchantments = { { Enchantment::SwiftSneak, 3 } } };
	auto body = baseBody();
	const Input input{ .moveVector = { 0.0f, 1.0f }, .sneakDown = true };

	applyInput(world, body, input);
	EXPECT_TRUE(near(body.impulse.y, static_cast<float>(0.3 * 0.98))) << std::format("TestSwiftSneakAppliesAfterTwoSlowdownTicks: first-tick sneak impulse, got {}", body.impulse.y);

	applyInput(world, body, input);
	applyInput(world, body, input);
	EXPECT_TRUE(near(body.impulse.y, static_cast<float>(0.75 * 0.98))) << std::format("TestSwiftSneakAppliesAfterTwoSlowdownTicks: Swift Sneak impulse after two ticks, got {}", body.impulse.y);
}

TEST(PortFeatures, CheckItemUseAndInventoryActionInputRules) {
	struct Case {
		std::string_view name;
		Input input;
		float want;
	};

	const std::array cases{
		Case{ "using item", { .moveVector = { 0.0f, 1.0f }, .usingItem = true }, kMaxConsumingImpulse * 0.98f },
		Case{ "using spear", { .moveVector = { 0.0f, 1.0f }, .usingItem = true, .usingSpear = true }, 0.98f },
		Case{ "inventory action", { .moveVector = { 0.0f, 1.0f }, .inventoryAction = true }, 0.0f }
	};

	const FakeWorld world;
	for (const auto& [name, input, want] : cases) {
		auto body = baseBody();
		applyInput(world, body, input);
		EXPECT_TRUE(near(body.impulse.y, want)) << std::format("TestItemUseAndInventoryActionInputRules/{}: impulse {} want {}", name, body.impulse.y, want);
	}
}

TEST(PortFeatures, CheckCrawlingUpdatesPoseAndSlowdown) {
	const BoxWorld world{ .boxes = { AABB{ { -1.0f, 0.7f, -1.0f }, { 1.0f, 2.0f, 1.0f } } } };
	auto body = baseBody();
	applyInput(world, body, { .moveVector = { 0.0f, 1.0f }, .startCrawling = true });

	EXPECT_TRUE(body.crawling && body.size.y == 0.6f) << std::format("TestCrawlingUpdatesPoseAndSlowdown: crawling={} size.y={}", body.crawling, body.size.y);
	EXPECT_TRUE(near(body.impulse.y, static_cast<float>(0.3 * 0.98))) << std::format("TestCrawlingUpdatesPoseAndSlowdown: crawling slowdown, got {}", body.impulse.y);
}

TEST(PortFeatures, CheckLevitationStopsGliding) {
	const FakeWorld world{ .effects = { { Effect::Levitation, 0 } }, .elytra = true };
	Simulator simulator{ world };
	auto body = baseBody();
	body.gliding = true;
	body.hasGravity = true;

	simulator.simulateState(body);

	EXPECT_TRUE(!body.gliding) << "TestLevitationStopsGliding: levitation stops gliding";
	EXPECT_TRUE(body.velocity.y > 0.0f) << std::format("TestLevitationStopsGliding: levitation velocity after the glide stops, got {}", body.velocity.y);
}

TEST(PortFeatures, CheckStoppingGlideDoesNotCancelActiveBoost) {
	const FakeWorld world;
	auto body = baseBody();
	body.gliding = true;
	body.glideBoostTicks = 10;

	applyInput(world, body, { .stopGliding = true });

	EXPECT_TRUE(body.glideBoostTicks == 10) << std::format("TestStoppingGlideDoesNotCancelActiveBoost: boost keeps ticking independently, got {}", body.glideBoostTicks);
}

TEST(PortFeatures, CheckRawMovementResolution) {
	struct Case {
		std::string_view name;
		Input input;
		float want;
	};

	const std::array cases{
		Case{ "normal", {}, 1.0f },
		Case{ "consuming", { .usingConsumable = true }, kMaxConsumingImpulse },
		Case{ "held item", { .usingItem = true }, kMaxConsumingImpulse },
		Case{ "spear", { .usingItem = true, .usingSpear = true }, 1.0f },
		Case{ "sneaking", { .sneakDown = true }, kMaxSneakImpulse },
		Case{ "consuming sneak", { .sneakDown = true, .usingConsumable = true }, kMaxConsumingImpulse * kMaxSneakImpulse },
		Case{ "inventory", { .inventoryAction = true }, 0.0f }
	};

	const FakeWorld world;
	for (const bool upstream : { false, true }) {
		for (const auto& [name, caseInput, want] : cases) {
			auto body = baseBody();
			Simulator simulator{ world, { .upstreamImpulseClamping = upstream } };
			auto input = caseInput;
			input.moveVector = { 0.25f, -0.5f };
			input.moveVectorIsRaw = true;
			const auto result = simulator.simulate(body, input);
			const auto expected = input.moveVector * want;
			EXPECT_TRUE(result.inputMoveVector == expected && body.impulse == expected * 0.98f) << std::format("TestRawMovementResolution/{} upstream={}: input {} impulse {} want {}", name, upstream, result.inputMoveVector, body.impulse, expected);
		}
	}
}

TEST(PortFeatures, CheckRawMovementSwiftSneakUsesSimulationHistory) {
	const FakeWorld world{ .enchantments = { { Enchantment::SwiftSneak, 3 } } };
	auto body = baseBody();
	Simulator simulator{ world, { .upstreamImpulseClamping = true } };
	constexpr std::array scales{ 0.3f, 0.3f, 0.75f };
	for (size_t tick = 0; tick < scales.size(); ++tick) {
		const auto result = simulator.simulate(body, { .moveVector = { 0.0f, 0.5f }, .moveVectorIsRaw = true, .sneakDown = true });
		const Vec2 expected{ 0.0f, 0.5f * scales[tick] };
		EXPECT_TRUE(result.inputMoveVector == expected) << std::format("TestRawMovementSwiftSneakUsesSimulationHistory tick {}: got {} want scale {}", tick, result.inputMoveVector, scales[tick]);
	}
}

TEST(PortFeatures, CheckMovementStateCloneDetachesReferences) {
	EXPECT_TRUE(std::is_trivially_copyable_v<Body>) << "TestMovementStateCloneDetachesReferences: Body copies member-wise without shared references";

	auto original = baseBody();
	original.supportingBlock = BlockPos{ 1, 2, 3 };
	auto copy = original;
	EXPECT_TRUE(copy.supportingBlock == original.supportingBlock) << "TestMovementStateCloneDetachesReferences: copy keeps the supporting block";

	copy.supportingBlock->x = 9;
	EXPECT_TRUE(original.supportingBlock->x == 1) << "TestMovementStateCloneDetachesReferences: copy does not retain the supporting-block reference";
}

TEST(PortFeatures, CheckItemUseMovementModifier) {
	const FakeWorld world;
	for (const float value : { 0.0f, kMaxConsumingImpulse, 0.5f, 1.0f }) {
		for (const bool sneak : { false, true }) {
			for (const bool upstream : { false, true }) {
				auto body = baseBody();
				Simulator simulator{ world, { .upstreamImpulseClamping = upstream } };
				const Input input{
					.moveVector = { 0.25f, -0.5f },
					.moveVectorIsRaw = true,
					.sneakDown = sneak,
					.itemUseMovementModifier = value,
					.usingConsumable = true,
					.usingItem = true
				};

				const auto result = simulator.simulate(body, input);
				float scale = value;
				if (sneak) {
					scale *= kMaxSneakImpulse;
				}

				const auto expected = input.moveVector * scale;
				EXPECT_TRUE(result.inputMoveVector == expected) << std::format("TestItemUseMovementModifier: modifier {} sneak {} upstream {} got {} want {}", value, sneak, upstream, result.inputMoveVector, expected);
			}
		}
	}

	constexpr float infinity = std::numeric_limits<float>::infinity();
	for (const float value : { -1.0f, 1.1f, std::numeric_limits<float>::quiet_NaN(), infinity, -infinity }) {
		auto body = baseBody();
		const auto before = bodyBytes(body);
		Simulator simulator{ world };
		const auto result = simulator.simulate(body, { .itemUseMovementModifier = value });
		EXPECT_TRUE(result.outcome == Outcome::InvalidInput && bodyBytes(body) == before) << std::format("TestItemUseMovementModifier: invalid modifier {} is rejected without mutation", value);
	}
}

TEST(PortFeatures, CheckEyePositionUsesVanillaPoseOffset) {
	struct Case {
		std::string_view name;
		std::function<void(Body&)> pose;
		float wantY;
	};

	const std::array cases{
		Case{ "standing", [](Body&) { }, 11.62f },
		Case{ "sneaking", [](Body& body) {
			body.sneaking = true;
		}, 11.27f },
		Case{ "swimming", [](Body& body) {
			body.swimming = true;
			body.swimWaterGraceTicks = 1;
		}, 10.4f },
		Case{ "crawling", [](Body& body) {
			body.crawling = true;
		}, 10.4f },
		Case{ "gliding", [](Body& body) {
			body.gliding = true;
		}, 10.4f },
		Case{ "riptide", [](Body& body) {
			body.riptideTicks = 1;
		}, 10.4f },
		Case{ "scaled", [](Body& body) {
			body.size = { 0.6f, 1.8f, 2.0f };
		}, 13.24f }
	};

	for (const auto& [name, pose, wantY] : cases) {
		Body body{};
		body.size = {};
		pose(body);
		body.position = { 2.0f, 10.0f, 3.0f };
		const auto eye = body.eyePosition();
		EXPECT_TRUE((eye == Vec3{ 2.0f, wantY, 3.0f })) << std::format("TestMovementState_EyePositionUsesVanillaPoseOffset/{}: eye {} want y {}", name, eye, wantY);
	}
}

TEST(PortFeatures, CheckObserveHeadLiquidFailsClosedWithoutWorld) {
	const BoxWorld world{ .loaded = false };
	Simulator simulator{ world };
	const auto body = submergedBody();
	EXPECT_TRUE(!simulator.observeHeadLiquid(body).known) << "TestSimulator_ObserveHeadLiquidFailsClosedWithoutWorld: observation without world data is unknown";
}

TEST(PortFeatures, CheckObserveHeadLiquidUsesPoseAndSurfaceHeight) {
	FakeWorld world;
	world.water({ 0, 0, 0 }, 4);
	Simulator simulator{ world, liquidOptions() };
	auto body = submergedBody();
	body.position = { 0.5f, 0.1f, 0.5f };

	const auto standing = simulator.observeHeadLiquid(body);
	EXPECT_TRUE(standing.known && !standing.water) << "TestSimulator_ObserveHeadLiquidUsesPoseAndSurfaceHeight: standing head is known and breathable";

	body.swimming = true;
	body.swimWaterGraceTicks = 1;
	const auto swimming = simulator.observeHeadLiquid(body);
	EXPECT_TRUE(swimming.known && swimming.water) << "TestSimulator_ObserveHeadLiquidUsesPoseAndSurfaceHeight: swimming head is below the partial water surface";
}

TEST(PortFeatures, CheckObserveHeadLiquidReadsSecondLayerAndFailsClosed) {
	FakeWorld layered;
	layered.set({ 0, 0, 0 }, { .name = "minecraft:oak_stairs", .shapes = kFullBlock, .liquid = Liquid{ .kind = LiquidKind::Water, .depth = 8 } });
	Simulator simulator{ layered, liquidOptions() };
	auto body = submergedBody();
	body.swimming = true;
	body.swimWaterGraceTicks = 1;
	body.position = { 0.5f, 0.1f, 0.5f };
	const auto waterlogged = simulator.observeHeadLiquid(body);
	EXPECT_TRUE(waterlogged.known && waterlogged.water) << "TestSimulator_ObserveHeadLiquidReadsSecondLayerAndFailsClosed: waterlogged observation is known water";

	const BoxWorld withoutLayer;
	auto options = liquidOptions();
	options.requireLiquidLayer = true;
	Simulator missing{ withoutLayer, options };
	EXPECT_TRUE(!missing.observeHeadLiquid(body).known) << "TestSimulator_ObserveHeadLiquidReadsSecondLayerAndFailsClosed: missing required liquid layer is unknown";

	FakeWorld unloaded;
	unloaded.loaded = [](const AABB&) {
		return false;
	};

	Simulator unloadedSimulator{ unloaded, liquidOptions() };
	EXPECT_TRUE(!unloadedSimulator.observeHeadLiquid(body).known) << "TestSimulator_ObserveHeadLiquidReadsSecondLayerAndFailsClosed: unloaded observation is unknown";
}

TEST(PortFeatures, CheckReplayPreservesTickContinuityAndSnapshots) {
	const auto world = filledColumn();
	Simulator simulator{ world, liquidOptions() };
	auto initial = submergedBody();
	initial.hasGravity = true;
	initial.movementSpeed = 0.1f;
	initial.defaultMovementSpeed = 0.1f;
	const std::vector<Input> inputs{
		{ .moveVector = { 0.0f, 1.0f }, .clientPosition = initial.position },
		{ .moveVector = { 0.0f, 1.0f }, .clientPosition = initial.position }
	};

	auto replayed = simulator.replay(initial, inputs);
	EXPECT_TRUE(replayed.frames.size() == inputs.size()) << std::format("TestSimulator_ReplayPreservesTickContinuityAndSnapshots: frames {} want {}", replayed.frames.size(), inputs.size());
	if (replayed.frames.size() != inputs.size()) {
		return;
	}

	EXPECT_TRUE(replayed.frames[1].body.position != replayed.frames[0].body.position) << "TestSimulator_ReplayPreservesTickContinuityAndSnapshots: second tick advances the continuous state";
	EXPECT_TRUE(replayed.body.position == replayed.frames[1].body.position) << "TestSimulator_ReplayPreservesTickContinuityAndSnapshots: final body matches the last frame";
	EXPECT_TRUE((initial.position == Vec3{ 0.5f, 0.5f, 0.5f })) << "TestSimulator_ReplayPreservesTickContinuityAndSnapshots: replay does not mutate the initial body";
	for (size_t index = 0; index < replayed.frames.size(); ++index) {
		const auto& frame = replayed.frames[index];
		EXPECT_TRUE(frame.result.outcome == Outcome::Normal) << std::format("TestSimulator_ReplayPreservesTickContinuityAndSnapshots: frame {} outcome is normal", index);
		EXPECT_TRUE(frame.headLiquid.known && frame.headLiquid.water) << std::format("TestSimulator_ReplayPreservesTickContinuityAndSnapshots: frame {} head is known submerged water", index);
	}

	initial.supportingBlock = BlockPos{ 1, 2, 3 };
	const std::vector<Input> idle{
		{ .clientPosition = initial.position },
		{ .clientPosition = initial.position }
	};

	replayed = simulator.replay(initial, idle);
	const auto secondSupport = replayed.frames[1].body.supportingBlock;
	const auto finalSupport = replayed.body.supportingBlock;
	replayed.frames[0].body.supportingBlock = BlockPos{ 9, 9, 9 };
	EXPECT_TRUE(replayed.frames[1].body.supportingBlock == secondSupport && replayed.body.supportingBlock == finalSupport) << "TestSimulator_ReplayPreservesTickContinuityAndSnapshots: frame snapshots do not alias the supporting block";
	EXPECT_TRUE((initial.supportingBlock == BlockPos{ 1, 2, 3 })) << "TestSimulator_ReplayPreservesTickContinuityAndSnapshots: replay snapshots do not alias the initial supporting block";
}

TEST(PortFeatures, CheckCloneMovementStateHandlesEveryReferenceField) {
	EXPECT_TRUE(std::is_trivially_copyable_v<Body> && std::is_trivially_copyable_v<ClientState> && std::is_trivially_copyable_v<RetainedBox>) << "TestCloneMovementStateHandlesEveryReferenceField: Body holds no reference-bearing fields";
}

TEST(PortFeatures, CheckFiniteMovementStateRejectsEveryFloatField) {
	struct Field {
		std::string_view name;
		std::function<void(Body&)> poison;
	};

	constexpr float nan = std::numeric_limits<float>::quiet_NaN();
	const std::vector<Field> fields{
		{ "client.position", [](Body& body) {
			body.client.position.x = nan;
		} },
		{ "client.lastPosition", [](Body& body) {
			body.client.lastPosition.x = nan;
		} },
		{ "client.velocity", [](Body& body) {
			body.client.velocity.x = nan;
		} },
		{ "client.lastVelocity", [](Body& body) {
			body.client.lastVelocity.x = nan;
		} },
		{ "client.movement", [](Body& body) {
			body.client.movement.x = nan;
		} },
		{ "client.lastMovement", [](Body& body) {
			body.client.lastMovement.x = nan;
		} },
		{ "position", [](Body& body) {
			body.position.x = nan;
		} },
		{ "lastPosition", [](Body& body) {
			body.lastPosition.x = nan;
		} },
		{ "velocity", [](Body& body) {
			body.velocity.x = nan;
		} },
		{ "lastVelocity", [](Body& body) {
			body.lastVelocity.x = nan;
		} },
		{ "movement", [](Body& body) {
			body.movement.x = nan;
		} },
		{ "lastMovement", [](Body& body) {
			body.lastMovement.x = nan;
		} },
		{ "rotation", [](Body& body) {
			body.rotation.x = nan;
		} },
		{ "lastRotation", [](Body& body) {
			body.lastRotation.x = nan;
		} },
		{ "slideOffset", [](Body& body) {
			body.slideOffset.x = nan;
		} },
		{ "impulse", [](Body& body) {
			body.impulse.x = nan;
		} },
		{ "size", [](Body& body) {
			body.size.x = nan;
		} },
		{ "stuckSpeedMultiplier", [](Body& body) {
			body.stuckSpeedMultiplier.x = nan;
		} },
		{ "standingHeight", [](Body& body) {
			body.standingHeight = nan;
		} },
		{ "sneakingHeight", [](Body& body) {
			body.sneakingHeight = nan;
		} },
		{ "crawlingHeight", [](Body& body) {
			body.crawlingHeight = nan;
		} },
		{ "gravity", [](Body& body) {
			body.gravity = nan;
		} },
		{ "jumpHeight", [](Body& body) {
			body.jumpHeight = nan;
		} },
		{ "jumpStrength", [](Body& body) {
			body.jumpStrength = nan;
		} },
		{ "fallDistance", [](Body& body) {
			body.fallDistance = nan;
		} },
		{ "movementSpeed", [](Body& body) {
			body.movementSpeed = nan;
		} },
		{ "defaultMovementSpeed", [](Body& body) {
			body.defaultMovementSpeed = nan;
		} },
		{ "airSpeed", [](Body& body) {
			body.airSpeed = nan;
		} },
		{ "underwaterMovementSpeed", [](Body& body) {
			body.underwaterMovementSpeed = nan;
		} },
		{ "lavaMovementSpeed", [](Body& body) {
			body.lavaMovementSpeed = nan;
		} },
		{ "swimSpeedMultiplier", [](Body& body) {
			body.swimSpeedMultiplier = nan;
		} },
		{ "knockback", [](Body& body) {
			body.knockback.x = nan;
		} },
		{ "pendingTeleportPosition", [](Body& body) {
			body.pendingTeleportPosition.x = nan;
		} },
		{ "teleportPosition", [](Body& body) {
			body.teleportPosition.x = nan;
		} },
		{ "swimAmount", [](Body& body) {
			body.swimAmount = nan;
		} }
	};

	const auto base = baseBody();
	EXPECT_TRUE(isFinite(base)) << "TestFiniteMovementStateRejectsEveryFloatField: the base body is finite";
	for (const auto& [name, poison] : fields) {
		auto body = base;
		poison(body);
		EXPECT_TRUE(!isFinite(body)) << std::format("TestFiniteMovementStateRejectsEveryFloatField/Body.{}: NaN is rejected", name);
	}
}
