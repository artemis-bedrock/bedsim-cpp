#include "helpers.h"
#include "fake_world.h"

#include "bedsim/constants.h"
#include "bedsim/physics.h"
#include "bedsim/semantics.h"
#include "bedsim/simulator.h"

#include <array>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <gtest/gtest.h>

namespace bedsim::test {
	struct SemanticBoxWorld {
		std::vector<AABB> boxes;
		std::map<BlockPos, std::string, BlockPosLess> names;
		std::optional<BlockMovement> semantics;

		void blockCollisions(const BlockPos&, std::vector<AABB>&) const { }

		void collisions(const AABB& area, std::vector<AABB>& out) const {
			for (const auto& box : boxes) {
				if (area.intersects(box)) {
					out.push_back(box);
				}
			}
		}

		[[nodiscard]] BlockMovement movement(const BlockPos& position) const {
			if (semantics) {
				return *semantics;
			}

			const auto name = names.find(position);
			return vanillaMovement(name != names.end() ? name->second : "minecraft:air");
		}

		[[nodiscard]] bool isLoaded(const AABB&) const {
			return true;
		}
	};

	struct CountingBoxWorld {
		std::vector<AABB> boxes;
		bool loaded{};
		mutable int nearbyCalls{};

		void blockCollisions(const BlockPos&, std::vector<AABB>&) const { }

		void collisions(const AABB& area, std::vector<AABB>& out) const {
			++nearbyCalls;
			for (const auto& box : boxes) {
				if (area.intersects(box)) {
					out.push_back(box);
				}
			}
		}

		[[nodiscard]] BlockMovement movement(const BlockPos&) const {
			return vanillaMovement("minecraft:air");
		}

		[[nodiscard]] bool isLoaded(const AABB&) const {
			return loaded;
		}
	};

	struct ProbeBoxWorld {
		std::vector<AABB> boxes;

		void blockCollisions(const BlockPos&, std::vector<AABB>&) const { }

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
			return area.max.y <= 2.81f;
		}
	};

	struct DynamicCollisionWorld {
		mutable CollisionContext lastContext;

		void blockCollisions(const BlockPos&, std::vector<AABB>&) const { }

		void movementCollisions(const AABB&, const CollisionContext& context, std::vector<AABB>& out) const {
			lastContext = context;
			if (context.leatherBoots && !context.descending && !context.wantDown) {
				out.push_back(AABB{ {}, Vec3{ 1.0f } });
			}
		}

		[[nodiscard]] BlockMovement movement(const BlockPos&) const {
			return vanillaMovement("minecraft:air");
		}

		[[nodiscard]] bool isLoaded(const AABB&) const {
			return true;
		}

		[[nodiscard]] std::optional<Liquid> liquid(const BlockPos&) const {
			return std::nullopt;
		}

		[[nodiscard]] int enchantmentLevel(const Enchantment) const {
			return 0;
		}

		[[nodiscard]] bool wearingLeatherBoots() const {
			return true;
		}
	};

	template <World W>
	struct SystemsFixture {
		Options options;
		Scratch scratch;
		systems::Systems<W> systems;

		explicit SystemsFixture(const W& world, Options value = {})
			: options(std::move(value)), systems(world, options, scratch) { }
	};

	struct RoundedContactFixture {
		Body body;
		BoxWorld world;
	};

	static_assert(HasCollisionQuery<SemanticBoxWorld> && HasCollisionQuery<CountingBoxWorld> && HasCollisionQuery<ProbeBoxWorld>);
	static_assert(HasMovementCollisions<DynamicCollisionWorld> && HasEquipment<DynamicCollisionWorld>);
}

using namespace bedsim;
using bedsim::test::BoxWorld;
using bedsim::test::CountingBoxWorld;
using bedsim::test::DynamicCollisionWorld;
using bedsim::test::FakeWorld;
using bedsim::test::ProbeBoxWorld;
using bedsim::test::RoundedContactFixture;
using bedsim::test::SemanticBoxWorld;
using bedsim::test::SystemsFixture;
using bedsim::test::baseBody;
using bedsim::test::near;

template <World W>
static systems::AppliedInput applyInput(const W& world, Body& body, const Input& input) {
	SystemsFixture fixture{ world };
	return fixture.systems.inputs.apply(body, input);
}

static BoxWorld wallSlideWorld() {
	return BoxWorld{ .boxes = {
		AABB{ { 255.0f, 0.0f, -10.0f }, { 258.0f, 1.0f, 10.0f } },
		AABB{ { 255.0f, 0.0f, -10.0f }, { 256.0f, 4.0f, 10.0f } }
	} };
}

static void checkWallSlideTicks(SystemsFixture<BoxWorld>& fixture, Body& body) {
	for (int tick = 0; tick < 3; ++tick) {
		body.velocity = { 0.0f, -0.0784f, 0.1f };
		const float previousZ = body.position.z;
		const bool completed = fixture.systems.collision.tryCollisions(body);
		EXPECT_TRUE(completed) << "wall slide collision simulation completes";
		EXPECT_TRUE(!body.collideX && !body.collideZ && !body.penetratedLastFrame && !body.stuckInCollider) << "wall slide acquires no collision or penetration";
		EXPECT_TRUE(body.onGround && body.position.z > previousZ) << "wall slide keeps moving along the floor";
	}
}

TEST(PortCollision, CheckBoundingBoxSweptWallSlide) {
	const auto world = wallSlideWorld();
	SystemsFixture fixture{ world };
	auto body = baseBody();
	body.position = { 256.5f, 1.0f, 0.5f };
	body.client.position = body.position;
	body.onGround = true;

	body.velocity = { -0.3f, -0.0784f, 0.0f };
	const bool approached = fixture.systems.collision.tryCollisions(body);
	EXPECT_TRUE(approached) << "TestBoundingBoxSweptWallSlide: approach available";
	checkWallSlideTicks(fixture, body);
}

TEST(PortCollision, CheckBoundingBoxVanillaWallContact) {
	struct Case {
		std::string_view name;
		Vec3 position;
		AABB wall;
		Vec3 velocity;
		int axis;
		float want;
	};

	const std::array cases{
		Case{ "captured climb", { 186.61009f, -59.580002f, 4.5f }, AABB{ { 187.0f, -61.0f, 3.0f }, { 188.0f, -59.0f, 6.0f } }, { 0.28f, 0.0f, 0.0f }, 0, 186.7f },
		Case{ "captured wall", { 26.5f, -59.0f, 5.611176f }, AABB{ { 24.0f, -60.0f, 6.0f }, { 29.0f, -58.0f, 7.0f } }, { 0.0f, 0.0f, 0.2584f }, 2, 5.7f }
	};

	for (const auto& [name, position, wall, velocity, axis, want] : cases) {
		auto body = baseBody();
		body.position = position;
		body.client.position = position;
		for (const auto& box : { body.boundingBox(false), body.clientBoundingBox(false) }) {
			const auto clipped = clipCollide(wall, box, velocity, false);
			const auto moved = box.translate(clipped.velocity);
			const auto center = (moved.min + moved.max) * 0.5f;
			EXPECT_TRUE(center[axis] == want) << (name == "captured climb" ? "TestBoundingBox_VanillaWallContact/captured climb: contact matches the BDS position" : "TestBoundingBox_VanillaWallContact/captured wall: contact matches the BDS position");
		}
	}
}

TEST(PortCollision, CheckClipCollideContactEpsilonDoesNotShrinkSweeps) {
	struct Case {
		float gap;
		bool blocked;
	};

	const AABB wall{ { -1.0f, -1.0f, -1.0f }, { 0.0f, 3.0f, 3.0f } };
	for (const auto [gap, blocked] : std::array{ Case{ -5e-7f, false }, Case{ -2e-6f, true }, Case{ 5e-7f, false } }) {
		const AABB box{ { gap, 0.0f, 0.0f }, { 0.6f + gap, 1.8f, 0.6f } };
		const auto clip = clipCollide(wall, box, { 0.0f, 0.0f, 0.1f }, false);
		EXPECT_TRUE((clip.penetration.x > 0.0f) == blocked && (clip.velocity.x > 0.0f) == blocked) << "TestBBClipCollide_ContactEpsilonDoesNotShrinkSweeps: contact epsilon only snaps true contacts";
	}
}

TEST(PortCollision, CheckRetainedShapeInvalidatesForExternalChanges) {
	auto body = baseBody();
	body.position = { 256.3f, 1.0f, 0.5f };
	const AABB box{ { 256.0f, 1.0f, 0.2f }, { 256.6f, 2.8f, 0.8f } };
	body.rememberCollisionBox(box, false);
	auto clone = body;
	EXPECT_TRUE(body.boundingBox(false) == box) << "TestBoundingBox_RetainedShapeInvalidatesForExternalChanges: keeps exact swept endpoints";

	body.setPosition({ 10.0f, 2.0f, 3.0f });
	EXPECT_TRUE(body.boundingBox(false).min.x == 9.7f) << "TestBoundingBox_RetainedShapeInvalidatesForExternalChanges: position change drops the retained box";
	EXPECT_TRUE(clone.boundingBox(false) == box) << "TestBoundingBox_RetainedShapeInvalidatesForExternalChanges: clone keeps its own geometry";

	clone.size.x = 1.0f;
	EXPECT_TRUE(clone.boundingBox(false).min.x == clone.position.x - 0.5f) << "TestBoundingBox_RetainedShapeInvalidatesForExternalChanges: width change drops the retained box";

	clone = body;
	clone.rememberCollisionBox(clone.boundingBox(false), false);
	clone.crawling = true;
	clone.size.y = 0.6f;
	EXPECT_TRUE(clone.boundingBox(false).max.y == 2.6f) << "TestBoundingBox_RetainedShapeInvalidatesForExternalChanges: height change drops the retained box";
}

TEST(PortCollision, CheckBoundingBoxRoundedWallSlide) {
	const auto world = wallSlideWorld();
	SystemsFixture fixture{ world };
	auto body = baseBody();
	body.position = { 256.3f, 1.0f, 0.5f };
	body.client.position = body.position;
	body.onGround = true;
	checkWallSlideTicks(fixture, body);
}

static RoundedContactFixture roundedContactFixture(const int axis, const float sign) {
	auto body = baseBody();
	body.position = { 0.5f, 1.0f, 0.5f };
	body.position[axis] = sign * 256.3f;
	body.client.position = body.position;
	body.onGround = true;
	body.velocity = { 0.0f, -0.0784f, 0.0f };
	body.velocity[2 - axis] = 0.1f;

	Vec3 low{ -300.0f, 0.0f, -300.0f };
	Vec3 high{ 300.0f, 4.0f, 300.0f };
	if (sign > 0.0f) {
		high[axis] = 256.0f;
	} else {
		low[axis] = -256.0f;
	}

	return {
		.body = body,
		.world = BoxWorld{ .boxes = {
			AABB{ { -300.0f, 0.0f, -300.0f }, { 300.0f, 1.0f, 300.0f } },
			AABB{ low, high }
		} }
	};
}

TEST(PortCollision, CheckSimulatorRoundedContactSeed) {
	for (const int axis : { 0, 2 }) {
		for (const float sign : { -1.0f, 1.0f }) {
			for (const bool withInput : { false, true }) {
				auto [body, world] = roundedContactFixture(axis, sign);
				Simulator simulator{ world };
				for (int tick = 0; tick < 3; ++tick) {
					auto position = body.position;
					position[2 - axis] += 0.01f;
					body.setPosition(position);
					body.velocity = { 0.0f, -0.0784f, 0.0f };
					body.velocity[2 - axis] = 0.1f;

					Result result{};
					if (withInput) {
						result = simulator.simulate(body, { .clientPosition = body.position, .clientVelocity = body.velocity });
					} else {
						result = simulator.simulateState(body);
					}

					EXPECT_TRUE(result.outcome == Outcome::Normal && !result.collideX && !result.collideZ && !body.penetratedLastFrame && !body.stuckInCollider) << "TestSimulatorRoundedContactSeed: reseeded rounded contact reports no false contact";
					EXPECT_TRUE(body.position[axis] == sign * 256.3f && body.onGround) << "TestSimulatorRoundedContactSeed: seed keeps the contact center and floor";
				}
			}
		}
	}
}

TEST(PortCollision, CheckRoundedContactRecoveryKeepsRealPenetration) {
	for (const int axis : { 0, 2 }) {
		for (const float sign : { -1.0f, 1.0f }) {
			auto [body, world] = roundedContactFixture(axis, sign);
			body.position[axis] -= sign * 0.005f;
			body.client.position = body.position;
			Simulator simulator{ world };
			const auto result = simulator.simulateState(body);
			EXPECT_TRUE(result.outcome == Outcome::Normal && body.penetratedLastFrame && (result.collideX || result.collideZ)) << "TestRoundedContactRecoveryKeepsRealPenetration: real penetration remains";
		}
	}
}

TEST(PortCollision, CheckRoundedContactRecoveryCornerOrder) {
	for (const bool reverse : { false, true }) {
		auto body = baseBody();
		body.position = { 256.3f, 1.0f, 256.3f };
		std::vector boxes{
			AABB{ { 255.0f, 0.0f, 255.0f }, { 256.0f, 4.0f, 258.0f } },
			AABB{ { 255.0f, 0.0f, 255.0f }, { 258.0f, 4.0f, 256.0f } }
		};

		if (reverse) {
			std::swap(boxes[0], boxes[1]);
		}

		const BoxWorld world{ .boxes = boxes };
		SystemsFixture fixture{ world };
		fixture.systems.collision.prepareCollisionBox(body);
		const auto box = body.boundingBox(false);
		EXPECT_TRUE((box.min == Vec3{ 256.0f, 1.0f, 256.0f })) << "TestRoundedContactRecoveryCornerOrder: corner minimum is independent of provider order";

		const auto center = (box.min + box.max) * 0.5f;
		EXPECT_TRUE(center.x == body.position.x && center.z == body.position.z) << "TestRoundedContactRecoveryCornerOrder: recovery keeps the reported center";
	}
}

TEST(PortCollision, CheckRoundedContactRecoveryDefersUnloadedWorld) {
	auto [body, world] = roundedContactFixture(0, 1.0f);
	world.loaded = false;
	Simulator simulator{ world };
	auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk && !body.retainedBox) << "TestRoundedContactRecoveryDefersUnloadedWorld: unloaded world establishes no contact geometry";

	world.loaded = true;
	body.velocity = { 0.0f, -0.0784f, 0.1f };
	result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Normal && !result.collideX && !body.penetratedLastFrame) << "TestRoundedContactRecoveryDefersUnloadedWorld: loaded world recovers the contact";
}

TEST(PortCollision, CheckRoundedContactRecoveryNativeChanges) {
	constexpr std::array<std::string_view, 4> changes{ "teleport", "smoothed teleport", "height", "slide offset" };
	for (const auto change : changes) {
		auto [body, world] = roundedContactFixture(0, 1.0f);
		SystemsFixture fixture{ world };
		fixture.systems.collision.prepareCollisionBox(body);
		if (change == "teleport" || change == "smoothed teleport") {
			body.queueTeleport(body.position, change == "smoothed teleport", 0);
			Simulator simulator{ world, fixture.options };
			const auto result = simulator.simulate(body, { .clientPosition = body.position });
			EXPECT_TRUE(result.outcome == Outcome::Teleport) << "TestRoundedContactRecoveryNativeChanges: teleport completes";
		} else if (change == "height") {
			body.size.y = 1.49f;
		} else {
			fixture.options.useSlideOffset = true;
			body.slideOffset.y = 0.1f;
		}

		body.velocity = { 0.0f, -0.0784f, 0.1f };
		const bool completed = fixture.systems.collision.tryCollisions(body);
		EXPECT_TRUE(completed && body.collideX && body.penetratedLastFrame) << (change == "teleport" ? "TestRoundedContactRecoveryNativeChanges/teleport: native reconstruction is kept" : change == "smoothed teleport" ? "TestRoundedContactRecoveryNativeChanges/smoothed teleport: native reconstruction is kept" : change == "height" ? "TestRoundedContactRecoveryNativeChanges/height: native reconstruction is kept" : "TestRoundedContactRecoveryNativeChanges/slide offset: native reconstruction is kept");
	}
}

TEST(PortCollision, CheckRoundedContactRecoveryAfterClientReset) {
	auto [body, world] = roundedContactFixture(0, 1.0f);
	SystemsFixture fixture{ world };
	body.queueTeleport(body.position, false, 0);
	const bool teleported = fixture.systems.movement.attemptTeleport(body);
	EXPECT_TRUE(teleported) << "TestRoundedContactRecoveryAfterClientReset: teleport succeeds";

	fixture.systems.movement.resetToClient(body);
	body.velocity = { 0.0f, -0.0784f, 0.1f };
	Simulator simulator{ world };
	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Normal && !result.collideX && !body.penetratedLastFrame) << "TestRoundedContactRecoveryAfterClientReset: client reset drops native geometry";
}

TEST(PortCollision, CheckRoundedContactRecoveryDoesNotEraseNewObstacles) {
	auto [body, world] = roundedContactFixture(0, 1.0f);
	const auto wall = world.boxes[1];
	world.boxes.resize(1);
	SystemsFixture fixture{ world };
	fixture.systems.collision.prepareCollisionBox(body);

	world.boxes.push_back(wall);
	Simulator simulator{ world };
	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(result.outcome == Outcome::Normal && result.collideX && body.penetratedLastFrame) << "TestRoundedContactRecoveryDoesNotEraseNewObstacles: new obstacle is a real collision";
}

TEST(PortCollision, CheckRoundedContactRecoveryRollsBackUnloadedPose) {
	auto [body, world] = roundedContactFixture(0, 1.0f);
	SystemsFixture fixture{ world };
	fixture.systems.collision.prepareCollisionBox(body);
	const auto original = body.boundingBox(false);

	const ProbeBoxWorld probeWorld{ .boxes = world.boxes };
	Simulator probeSimulator{ probeWorld };
	auto result = probeSimulator.simulate(body, { .clientPosition = body.position, .startSneaking = true });
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestRoundedContactRecoveryRollsBackUnloadedPose: movement probe is unloaded";
	EXPECT_TRUE(!body.sneaking && body.boundingBox(false) == original) << "TestRoundedContactRecoveryRollsBackUnloadedPose: unloaded pose keeps the retained contact geometry";

	Simulator simulator{ world };
	body.velocity = { 0.0f, -0.0784f, 0.1f };
	result = simulator.simulate(body, { .clientPosition = body.position });
	EXPECT_TRUE(result.outcome == Outcome::Normal && !result.collideX && !body.penetratedLastFrame) << "TestRoundedContactRecoveryRollsBackUnloadedPose: retry keeps the contact geometry";
}

TEST(PortCollision, CheckMovementCollisionProviderReceivesPlayerDependentContext) {
	const DynamicCollisionWorld world;
	SystemsFixture fixture{ world };
	auto body = baseBody();
	body.sneaking = true;
	body.pressingDescend = false;

	std::vector<AABB> boxes;
	fixture.systems.collision.nearbyBoxes(body, body.boundingBox(false), boxes);
	EXPECT_TRUE(boxes.size() == 1) << "TestMovementCollisionProviderReceivesPlayerDependentContext: dynamic powder-snow collision";
	EXPECT_TRUE(world.lastContext.leatherBoots && world.lastContext.sneaking) << "TestMovementCollisionProviderReceivesPlayerDependentContext: equipment and sneak context";
}

TEST(PortCollision, CheckPoseTransitionsDoNotReadCollisionsFromUnloadedChunks) {
	const CountingBoxWorld world{ .loaded = false };
	Simulator simulator{ world };
	auto body = baseBody();
	body.sneaking = true;
	body.size.y = 1.49f;

	const auto result = simulator.simulate(body, { .stopSneaking = true });
	EXPECT_TRUE(result.outcome == Outcome::UnloadedChunk) << "TestPoseTransitionsDoNotReadCollisionsFromUnloadedChunks: unloaded outcome";
	EXPECT_TRUE(world.nearbyCalls == 0) << "TestPoseTransitionsDoNotReadCollisionsFromUnloadedChunks: no collision queries in an unloaded chunk";
}

TEST(PortCollision, CheckCannotUnsneakUnderLowCeiling) {
	const BoxWorld world{ .boxes = { AABB{ { -1.0f, 1.5f, -1.0f }, { 1.0f, 2.0f, 1.0f } } } };
	auto body = baseBody();
	body.sneaking = true;
	body.size.y = 1.49f;

	applyInput(world, body, { .stopSneaking = true });
	applyInput(world, body, {});
	EXPECT_TRUE(body.sneaking && body.size.y == 1.49f) << "TestCannotUnsneakUnderLowCeiling: forced sneak pose under a ceiling";
}

TEST(PortCollision, CheckReleasingSneakDownRestoresStandingPose) {
	const BoxWorld world;
	auto body = baseBody();

	applyInput(world, body, { .sneakDown = true });
	applyInput(world, body, {});
	EXPECT_TRUE(!body.sneaking && body.size.y == body.standingHeight) << "TestReleasingSneakDownRestoresStandingPose: standing pose after releasing sneak";
}

TEST(PortCollision, CheckCanFitHeightUsesRequestedHeightWhileSwimming) {
	const BoxWorld world{ .boxes = { AABB{ { -1.0f, 1.0f, -1.0f }, { 1.0f, 2.0f, 1.0f } } } };
	SystemsFixture fixture{ world };
	auto body = baseBody();
	body.swimming = true;
	body.swimWaterGraceTicks = 1;

	const auto fit = fixture.systems.collision.canFitHeight(body, 1.8f);
	EXPECT_TRUE(!(fit.known && fit.fits)) << "TestCanFitHeightUsesRequestedHeightWhileSwimming: standing-height fit collides despite the swim pose";
}

TEST(PortCollision, CheckCannotStopCrawlingUnderLowCeiling) {
	const BoxWorld world{ .boxes = { AABB{ { -1.0f, 0.7f, -1.0f }, { 1.0f, 2.0f, 1.0f } } } };
	auto body = baseBody();
	body.crawling = true;
	body.size.y = 0.6f;

	applyInput(world, body, { .stopCrawling = true });
	EXPECT_TRUE(body.crawling && body.size.y == 0.6f) << "TestCannotStopCrawlingUnderLowCeiling: forced crawl pose under a ceiling";
}

TEST(PortCollision, CheckStopCrawlingWhileSneakingUsesCrouchHeight) {
	const BoxWorld world{ .boxes = { AABB{ { -1.0f, 1.6f, -1.0f }, { 1.0f, 2.0f, 1.0f } } } };
	auto body = baseBody();
	body.crawling = true;
	body.size.y = 0.6f;

	applyInput(world, body, { .sneakDown = true, .stopCrawling = true });
	EXPECT_TRUE(!body.crawling && body.sneaking && body.size.y == 1.49f) << "TestStopCrawlingWhileSneakingUsesCrouchHeight: crouch pose under a ceiling";
}

TEST(PortCollision, CheckStopSwimmingFallsBackToCrawlUnderLowCeiling) {
	const BoxWorld world{ .boxes = { AABB{ { -1.0f, 0.7f, -1.0f }, { 1.0f, 2.0f, 1.0f } } } };
	auto body = baseBody();
	body.swimming = true;
	body.swimWaterGraceTicks = 1;

	applyInput(world, body, { .stopSwimming = true });
	EXPECT_TRUE(!body.swimming && body.crawling && body.size.y == 0.6f) << "TestStopSwimmingFallsBackToCrawlUnderLowCeiling: crawl fallback after swimming";
}

TEST(PortCollision, CheckStartSwimmingWithoutSwimPosePreservesFittingCrawl) {
	const BoxWorld world{ .boxes = { AABB{ { -1.0f, 0.7f, -1.0f }, { 1.0f, 2.0f, 1.0f } } } };
	auto body = baseBody();
	body.crawling = true;
	body.size.y = 0.6f;

	applyInput(world, body, { .startSwimming = true });
	EXPECT_TRUE(body.swimming && body.crawling && body.size.y == 0.6f) << "TestStartSwimmingWithoutSwimPosePreservesFittingCrawl: crawl pose until swim collapse is observed";
}

TEST(PortCollision, CheckPoseRestoresCustomStandingHeight) {
	const FakeWorld world;
	auto body = baseBody();
	body.size.y = 2.0f;

	applyInput(world, body, { .startSneaking = true });
	applyInput(world, body, { .stopSneaking = true });
	EXPECT_TRUE(body.size.y == 2.0f) << "TestPoseRestoresCustomStandingHeight: custom standing height is restored";
}

TEST(PortCollision, CheckSneakingInWaterDescends) {
	FakeWorld world;
	world.water({ 0, 0, 0 });
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.gravity = kGravity;

	simulator.simulate(body, { .sneakDown = true, .sneaking = true });
	EXPECT_TRUE(near(body.velocity.y, -0.037f)) << "TestSneakingInWaterDescends: water descent velocity";
}

TEST(PortCollision, CheckStuckMovementMultiplierKeepsStrongestOverlappingEffect) {
	auto body = baseBody();
	body.velocity = { 1.0f, -1.0f, 1.0f };

	applyInsideBlockMovement(body, Inside::PowderSnow);
	applyInsideBlockMovement(body, Inside::SweetBerryBush);
	applyInsideBlockMovement(body, Inside::SweetBerryBush);
	EXPECT_TRUE((body.stuckSpeedMultiplier == Vec3{ 0.8f, 0.75f, 0.8f })) << "TestStuckMovementMultiplierKeepsStrongestOverlappingEffect: strongest multiplier without compounding";
	EXPECT_TRUE((body.velocity == Vec3{ 1.0f, -1.0f, 1.0f })) << "TestStuckMovementMultiplierKeepsStrongestOverlappingEffect: inside-block scan keeps velocity";
}

TEST(PortCollision, CheckStuckMovementMultiplierAppliesOnceAndClearsVelocity) {
	const FakeWorld world;
	Simulator simulator{ world };
	auto body = baseBody();
	body.hasGravity = false;
	body.velocity = { 1.0f, -1.0f, 1.0f };
	body.stuckSpeedMultiplier = { 0.8f, 0.75f, 0.8f };

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE((result.movement == Vec3{ 0.8f, -0.75f, 0.8f })) << "TestStuckMovementMultiplierAppliesOnceAndClearsVelocity: one scaled displacement";
	EXPECT_TRUE(body.velocity == Vec3{}) << "TestStuckMovementMultiplierAppliesOnceAndClearsVelocity: persistent velocity clears";
	EXPECT_TRUE(body.stuckSpeedMultiplier == Vec3{}) << "TestStuckMovementMultiplierAppliesOnceAndClearsVelocity: pending multiplier clears";
}

TEST(PortCollision, CheckNoClipDiscardsQueuedStuckMovement) {
	auto body = baseBody();
	body.noClip = true;
	body.stuckSpeedMultiplier = { 0.8f, 0.75f, 0.8f };

	EXPECT_TRUE(!applyStuckSpeedMultiplier(body)) << "TestNoClipDiscardsQueuedStuckMovement: no-clip does not consume a multiplier";
	EXPECT_TRUE(body.stuckSpeedMultiplier == Vec3{}) << "TestNoClipDiscardsQueuedStuckMovement: no-clip discards the queued effect";
}

TEST(PortCollision, CheckStuckMovementDoesNotBounce) {
	const SemanticBoxWorld world{
		.boxes = { AABB{ { -1.0f, 0.0f, -1.0f }, { 1.0f, 1.0f, 1.0f } } },
		.semantics = BlockMovement{ .bounce = Bounce::Slime, .air = true }
	};

	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.0f, 1.0f, 0.0f };
	body.velocity = { 0.0f, -1.0f, 0.0f };
	body.gravity = kGravity;
	body.stuckSpeedMultiplier = { 0.8f, 0.75f, 0.8f };

	simulator.simulateState(body);
	EXPECT_TRUE(body.velocity.y <= 0.0f) << "TestStuckMovementDoesNotBounce: stuck movement suppresses the bounce";
}

TEST(PortCollision, CheckHoneyBlockReducesJumpPower) {
	FakeWorld world;
	world.passable({ 0, 0, 0 }, "minecraft:honey_block");
	Simulator simulator{ world };
	auto body = baseBody();
	body.onGround = true;
	body.jumping = true;
	body.jumpHeight = kJumpHeight;
	body.hasGravity = false;

	simulator.simulateState(body);
	EXPECT_TRUE(body.jumpDelay == kJumpDelayTicks) << "TestHoneyBlockReducesJumpPower: jump is applied";
	EXPECT_TRUE(near(body.movement.y, kJumpHeight * 0.6f)) << "TestHoneyBlockReducesJumpPower: honey jump displacement";
	EXPECT_TRUE(near(body.velocity.y, kJumpHeight * 0.6f)) << "TestHoneyBlockReducesJumpPower: honey jump velocity";
}

TEST(PortCollision, CheckHoneyWallSlideAppliesOnSolidSideContact) {
	const SemanticBoxWorld world{
		.boxes = { AABB{ { 1.0f, -1.0f, 0.0f }, { 2.0f, 2.0f, 1.0f } } },
		.names = { { BlockPos{ 1, 0, 0 }, "minecraft:honey_block" } }
	};

	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.7f, 0.0f, 0.5f };
	body.velocity = { 1.0f, -0.2f, 0.1f };
	body.hasGravity = false;

	simulator.simulateState(body);
	EXPECT_TRUE(body.collideX) << "TestHoneyWallSlideAppliesOnSolidSideContact: horizontal collision with the honey wall";
	EXPECT_TRUE(body.velocity.y == -0.12f) << "TestHoneyWallSlideAppliesOnSolidSideContact: honey slide downward cap";
	EXPECT_TRUE(near(body.velocity.z, 0.1f * kAirFriction * 0.4f)) << "TestHoneyWallSlideAppliesOnSolidSideContact: honey slide lateral slowdown";
}

static void checkHoneySlideFallDistance(const Vec3& position, const float expected, const std::string_view message) {
	FakeWorld world;
	world.passable({ 0, 0, 0 }, "minecraft:honey_block");
	SystemsFixture fixture{ world };
	auto body = baseBody();
	body.position = position;
	body.velocity = { 0.0f, -0.2f, 0.0f };
	body.fallDistance = 4.0f;

	fixture.systems.blocks.applyInsideBlockEffects(body);
	EXPECT_TRUE(body.fallDistance == expected) << message;
}

TEST(PortCollision, CheckHoneySideSlideResetsFallDistance) {
	checkHoneySlideFallDistance({ 1.25f, 0.0f, 0.5f }, 0.0f, "TestHoneySideSlideResetsFallDistance: side slide resets fall distance");
}

TEST(PortCollision, CheckHoneyTopContactPreservesFallDistance) {
	checkHoneySlideFallDistance({ 0.5f, 0.0f, 0.5f }, 4.0f, "TestHoneyTopContactPreservesFallDistance: top contact keeps fall distance");
}

TEST(PortCollision, CheckScaffoldingAscendAndDescendSpeeds) {
	auto body = baseBody();
	body.pressingAscend = true;
	EXPECT_TRUE(!applyAscendableMovement(body, Traversal::Scaffolding, false)) << "TestScaffoldingAscendAndDescendSpeeds: ascent keeps vertical travel";
	EXPECT_TRUE(body.velocity.y == 0.15f) << "TestScaffoldingAscendAndDescendSpeeds: ascend velocity";

	body.pressingAscend = false;
	body.pressingDescend = true;
	EXPECT_TRUE(applyAscendableMovement(body, Traversal::Scaffolding, false)) << "TestScaffoldingAscendAndDescendSpeeds: descent skips vertical travel";
	EXPECT_TRUE(body.velocity.y == -0.15f) << "TestScaffoldingAscendAndDescendSpeeds: descend velocity";
}

TEST(PortCollision, CheckPowderSnowTraversalRequiresLeatherBoots) {
	auto body = baseBody();
	body.pressingAscend = true;
	EXPECT_TRUE(!applyAscendableMovement(body, Traversal::PowderSnow, false)) << "TestPowderSnowTraversalRequiresLeatherBoots: ascent keeps vertical travel";
	EXPECT_TRUE(body.velocity.y == 0.0f) << "TestPowderSnowTraversalRequiresLeatherBoots: no ascent without leather boots";

	EXPECT_TRUE(!applyAscendableMovement(body, Traversal::PowderSnow, true)) << "TestPowderSnowTraversalRequiresLeatherBoots: ascent keeps vertical travel with boots";
	EXPECT_TRUE(body.velocity.y == 0.2f) << "TestPowderSnowTraversalRequiresLeatherBoots: leather-boots ascent";
}

TEST(PortCollision, CheckHoneyWalkSlowdownMatchesSlime) {
	auto body = baseBody();
	body.onGround = true;
	body.velocity = { 1.0f, 0.05f, 1.0f };

	walkOnBlock(body, vanillaMovement("minecraft:honey_block"));
	EXPECT_TRUE(near(body.velocity.x, 0.41f) && near(body.velocity.z, 0.41f)) << "TestHoneyWalkSlowdownMatchesSlime: honey walk slowdown";
}

TEST(PortCollision, CheckSimulationAppliesScaffoldingTraversal) {
	FakeWorld world;
	world.passable({ 0, 0, 0 }, "minecraft:scaffolding");
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.hasGravity = false;

	simulator.simulate(body, { .ascendBlock = true });
	EXPECT_TRUE(body.velocity.y == 0.15f) << "TestSimulationAppliesScaffoldingTraversal: integrated scaffolding ascent";
}

TEST(PortCollision, CheckScaffoldingDescendSkipsAirGravity) {
	FakeWorld world;
	world.passable({ 0, 0, 0 }, "minecraft:scaffolding");
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.gravity = kGravity;
	body.hasGravity = true;
	body.pressingDescend = true;
	body.fallDistance = 4.0f;

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.y, -0.15f)) << "TestScaffoldingDescendSkipsAirGravity: scaffolding descent velocity";
	EXPECT_TRUE(body.fallDistance == 0.0f) << "TestScaffoldingDescendSkipsAirGravity: scaffolding descent resets fall distance";
}

TEST(PortCollision, CheckScaffoldingSupportEnablesDescent) {
	FakeWorld world;
	world.passable({ 0, 0, 0 }, "minecraft:scaffolding");
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 1.0f, 0.5f };
	body.onGround = true;
	body.hasGravity = true;
	body.supportingBlock = BlockPos{ 0, 0, 0 };
	body.pressingDescend = true;

	simulator.simulateState(body);
	EXPECT_TRUE(near(body.velocity.y, -0.15f)) << "TestScaffoldingSupportEnablesDescent: supported scaffolding descent velocity";
}

TEST(PortCollision, CheckSimulationDetectsNonSolidWebAndAppliesWeaving) {
	FakeWorld world{ .effects = { { Effect::Weaving, 0 } } };
	world.passable({ 0, 0, 0 }, "minecraft:web");
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.velocity = { 0.1f, 0.0f, 0.0f };
	body.hasGravity = false;

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(near(result.movement.x, 0.05f)) << "TestSimulationDetectsNonSolidWebAndAppliesWeaving: Weaving web movement";
}

TEST(PortCollision, CheckSoulBlocksKeepOrdinaryGroundFriction) {
	struct Case {
		std::string_view name;
		float accelerationFriction;
		bool soulSpeedNeutralizes;
	};

	for (const auto& [name, accelerationFriction, soulSpeedNeutralizes] : std::array{ Case{ "minecraft:soul_sand", kSoulSandAccelerationFriction, true }, Case{ "minecraft:soul_soil", 1.0f, false } }) {
		const auto movement = vanillaMovement(name);
		EXPECT_TRUE(movement.groundFriction == kBlockFriction) << "TestSoulBlocksKeepOrdinaryGroundFriction: ordinary ground friction";
		EXPECT_TRUE(movement.accelerationFriction == accelerationFriction) << "TestSoulBlocksKeepOrdinaryGroundFriction: acceleration friction multiplier";
		EXPECT_TRUE(movement.soulSpeedNeutralizesFriction == soulSpeedNeutralizes) << "TestSoulBlocksKeepOrdinaryGroundFriction: soul-speed neutralization";
	}
}

TEST(PortCollision, CheckDefaultMovementBlockSemantics) {
	struct Case {
		std::string_view name;
		bool climbable;
		bool cobweb;
		Bounce bounce;
		float groundFriction;
	};

	const std::array cases{
		Case{ "minecraft:air", false, false, Bounce::None, kBlockFriction },
		Case{ "minecraft:ladder", true, false, Bounce::None, kBlockFriction },
		Case{ "minecraft:vine", true, false, Bounce::None, kBlockFriction },
		Case{ "minecraft:soul_soil", false, false, Bounce::None, kBlockFriction }
	};

	for (const auto& [name, climbable, cobweb, bounce, groundFriction] : cases) {
		const auto movement = vanillaMovement(name);
		EXPECT_TRUE(near(movement.groundFriction, groundFriction)) << "TestDefaultMovementBlockSemantics: ground friction";
		EXPECT_TRUE(movement.climbable == climbable) << "TestDefaultMovementBlockSemantics: climbable";
		EXPECT_TRUE(movement.cobweb == cobweb) << "TestDefaultMovementBlockSemantics: cobweb";
		EXPECT_TRUE(movement.bounce == bounce) << "TestDefaultMovementBlockSemantics: bounce";
	}
}

TEST(PortCollision, CheckBlueIceFrictionMatchesAcrossBlockRepresentations) {
	EXPECT_TRUE(vanillaMovement("minecraft:blue_ice").groundFriction == 0.989f) << "TestBlueIceFrictionMatchesAcrossBlockRepresentations: blue ice friction";
}

TEST(PortCollision, CheckFrostedIceFrictionMatchesVanillaIce) {
	EXPECT_TRUE(vanillaMovement("minecraft:frosted_ice").groundFriction == 0.98f) << "TestFrostedIceFrictionMatchesVanillaIce: frosted ice friction";
}

TEST(PortCollision, CheckDefaultMovementBlockSemanticsSpecialBlocks) {
	EXPECT_TRUE(vanillaMovement("minecraft:slime").bounce == Bounce::Slime) << "TestDefaultMovementBlockSemanticsSpecialBlocks/slime: slime bounce";
	EXPECT_TRUE(vanillaMovement("minecraft:bed").bounce == Bounce::Bed) << "TestDefaultMovementBlockSemanticsSpecialBlocks/bed: bed bounce";
	EXPECT_TRUE(vanillaMovement("minecraft:bamboo").bounce == Bounce::None) << "TestDefaultMovementBlockSemanticsSpecialBlocks/bamboo: no bounce";

	const auto web = vanillaMovement("minecraft:web");
	EXPECT_TRUE(web.bounce == Bounce::None && web.cobweb) << "TestDefaultMovementBlockSemanticsSpecialBlocks/cobweb: cobweb semantics";
}

TEST(PortCollision, CheckEnvironmentMovementSemantics) {
	struct Case {
		std::string_view name;
		Inside inside;
		Traversal traversal;
		bool honey;
	};

	const std::array cases{
		Case{ "minecraft:honey_block", Inside::None, Traversal::None, true },
		Case{ "minecraft:sweet_berry_bush", Inside::SweetBerryBush, Traversal::None, false },
		Case{ "minecraft:powder_snow", Inside::PowderSnow, Traversal::PowderSnow, false },
		Case{ "minecraft:scaffolding", Inside::None, Traversal::Scaffolding, false }
	};

	for (const auto& [name, inside, traversal, honey] : cases) {
		const auto movement = vanillaMovement(name);
		EXPECT_TRUE(movement.inside == inside && movement.traversal == traversal && movement.honey == honey) << "TestEnvironmentMovementSemantics: environment semantics";
	}
}

TEST(PortCollision, CheckHoneyBlockFrictionMatchesVanilla) {
	EXPECT_TRUE(vanillaMovement("minecraft:honey_block").groundFriction == 0.8f) << "TestHoneyBlockFrictionMatchesVanilla: honey block friction";
}

TEST(PortCollision, CheckSimulatorCompleteBlockSemanticsProvider) {
	const SemanticBoxWorld world{ .semantics = BlockMovement{
		.groundFriction = 0.37f,
		.accelerationFriction = 1.25f,
		.bounce = Bounce::Bed,
		.climbable = true,
		.cobweb = true
	} };

	SystemsFixture fixture{ world };
	const auto movement = fixture.systems.context.movement({ 0, 0, 0 });
	EXPECT_TRUE(movement.groundFriction == 0.37f && movement.accelerationFriction == 1.25f && movement.climbable && movement.cobweb && movement.bounce == Bounce::Bed) << "TestSimulatorCompleteBlockSemanticsProvider: complete semantic bundle";
}

TEST(PortCollision, CheckBambooDoesNotInvalidateSimulation) {
	const FakeWorld world{ .fill = "minecraft:bamboo" };
	SystemsFixture fixture{ world };
	EXPECT_TRUE(fixture.systems.movement.simulationIsReliable(baseBody())) << "TestBambooDoesNotInvalidateSimulation: bamboo uses ordinary collision simulation";
}

TEST(PortCollision, CheckSimulateSoulGroundSeparatesAccelerationFromDrag) {
	struct Case {
		std::string_view name;
		float accelerationFrictionFactor;
	};

	for (const auto& [name, accelerationFrictionFactor] : std::array{ Case{ "minecraft:soul_sand", 1.225000023841858f }, Case{ "minecraft:soul_soil", 1.0f } }) {
		const FakeWorld world{ .fill = std::string{ name } };
		Simulator simulator{ world };
		auto body = baseBody();
		body.position = { 0.0f, 1.0f, 0.0f };
		body.client.position = body.position;
		body.onGround = true;
		body.hasGravity = false;
		body.impulse = { 0.0f, 0.98f };

		const auto result = simulator.simulateState(body);
		const float groundFriction = kAirFriction * kBlockFriction;
		const float accelerationFriction = groundFriction * accelerationFrictionFactor;
		const float moveRelativeSpeed = body.movementSpeed * (0.16277136f / (accelerationFriction * accelerationFriction * accelerationFriction));
		const float wantZ = 0.98f * moveRelativeSpeed * groundFriction;
		EXPECT_TRUE(near(result.velocity.z, wantZ, 1e-5f)) << (name == "minecraft:soul_sand" ? "TestSimulateSoulGroundSeparatesAccelerationFromDrag/soul sand: ground velocity" : "TestSimulateSoulGroundSeparatesAccelerationFromDrag/soul soil: ground velocity");
	}
}

TEST(PortCollision, CheckBlockAirRecognisesOnlyBedrockAirIdentifier) {
	EXPECT_TRUE(vanillaMovement("minecraft:air").air) << "TestBlockAirRecognisesOnlyBedrockAirIdentifier/minecraft:air: air";
	EXPECT_TRUE(!vanillaMovement("minecraft:cave_air").air) << "TestBlockAirRecognisesOnlyBedrockAirIdentifier/minecraft:cave_air: not air";
	EXPECT_TRUE(!vanillaMovement("minecraft:void_air").air) << "TestBlockAirRecognisesOnlyBedrockAirIdentifier/minecraft:void_air: not air";
}

TEST(PortCollision, CheckJavaWebIdentifierHasNoBedrockMovementEffect) {
	FakeWorld world;
	world.passable({ 0, 0, 0 }, "minecraft:cobweb");
	Simulator simulator{ world };
	auto body = baseBody();
	body.position = { 0.5f, 0.0f, 0.5f };
	body.velocity = { 0.1f, 0.0f, 0.0f };
	body.hasGravity = false;

	const auto result = simulator.simulateState(body);
	EXPECT_TRUE(near(result.movement.x, 0.1f)) << "TestJavaWebIdentifierHasNoBedrockMovementEffect: Java web identifier keeps Bedrock movement";
}

TEST(PortCollision, CheckDefaultSneakingHeightMatchesDragonflyBedrockPlayer) {
	const FakeWorld world;
	auto body = baseBody();

	applyInput(world, body, { .startSneaking = true });
	EXPECT_TRUE(body.size.y == 1.49f) << "TestDefaultSneakingHeightMatchesDragonflyBedrockPlayer: sneaking height";
}

TEST(PortCollision, CheckCrawlingCannotStartInOpenAir) {
	const FakeWorld world;
	auto body = baseBody();

	applyInput(world, body, { .startCrawling = true });
	EXPECT_TRUE(!body.crawling && body.size.y == 1.8f) << "TestCrawlingCannotStartInOpenAir: open-air crawl is rejected";
}

TEST(PortCollision, CheckResolveVisitsEachRuleOnce) {
	const auto first = vanillaMovement("minecraft:test");
	const auto second = vanillaMovement("minecraft:test");
	EXPECT_TRUE(first.groundFriction == kBlockFriction && first.accelerationFriction == 1.0f) << "TestResolveVisitsEachRuleOnce: unknown block resolves default friction";
	EXPECT_TRUE(!first.air && !first.climbable && !first.cobweb && !first.honey && !first.fenceLike && !first.soulSpeedNeutralizesFriction) << "TestResolveVisitsEachRuleOnce: unknown block resolves no flags";
	EXPECT_TRUE(first.bounce == Bounce::None && first.inside == Inside::None && first.traversal == Traversal::None) << "TestResolveVisitsEachRuleOnce: unknown block resolves no behaviour";
	EXPECT_TRUE(second.groundFriction == first.groundFriction && second.accelerationFriction == first.accelerationFriction) << "TestResolveVisitsEachRuleOnce: resolution is repeatable";
}
