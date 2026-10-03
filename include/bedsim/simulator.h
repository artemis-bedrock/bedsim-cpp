#pragma once

#include "bedsim/body.h"
#include "bedsim/input.h"
#include "bedsim/options.h"
#include "bedsim/physics.h"
#include "bedsim/result.h"
#include "bedsim/scratch.h"
#include "bedsim/systems/systems.h"
#include "bedsim/vec.h"
#include "bedsim/world.h"

#include <algorithm>
#include <cstddef>
#include <span>
#include <vector>

namespace bedsim {
	template <World W>
	class Simulator {
	public:
		explicit Simulator(const W& world, const Options& options = {})
			: mWorld(world), mOptions(options) { }

		Simulator(W&& world, Options options = {}) = delete;

		Result simulate(Body& body, const Input& input) {
			if (!isFinite(body)) {
				return invalidResult(nullptr);
			}

			if (!isFinite(input)) {
				return invalidResult(&body);
			}

			systems::Systems<W> systems{ mWorld, mOptions, mScratch };
			systems.collision.prepareCollisionBox(body);
			const auto pose = systems::PoseSnapshot::capture(body);
			const auto applied = systems.inputs.apply(body, input);
			const bool wasOnGround = body.onGround;
			const float fallDistance = body.fallDistance;

			auto outcome = Outcome::UnloadedChunk;
			if (applied.poseKnown || body.hasTeleport() || body.inVehicle) {
				outcome = systems.movement.simulateCore(body, true);
			} else {
				body.setVelocity({});
				body.swimWaterGraceTicks = 0;
				body.swimWaterContact = false;
				body.stuckSpeedMultiplier = {};
			}

			if (mOptions.sprintTiming == SprintTiming::Legacy) {
				applyLegacySprint(body, input);
			}

			if (outcome == Outcome::UnloadedChunk) {
				pose.restore(body);
			} else {
				body.airSpeed = effectiveAirSpeed(body);
				tickState(body, outcome != Outcome::Teleport || body.hasTeleport());
			}

			auto result = resultFromState(body, outcome);
			result.inputMoveVector = applied.move;
			result.landed = outcome == Outcome::Normal && !wasOnGround && body.onGround;
			result.landingFallDistance = result.landed ? fallDistance : 0.0f;
			return result;
		}

		Result simulateState(Body& body) {
			if (!isFinite(body)) {
				return invalidResult(nullptr);
			}

			systems::Systems<W> systems{ mWorld, mOptions, mScratch };
			const bool wasOnGround = body.onGround;
			const float fallDistance = body.fallDistance;
			const auto outcome = systems.movement.simulateCore(body, false);
			auto result = resultFromState(body, outcome);
			result.landed = outcome == Outcome::Normal && !wasOnGround && body.onGround;
			result.landingFallDistance = result.landed ? fallDistance : 0.0f;
			return result;
		}

		[[nodiscard]] HeadLiquid observeHeadLiquid(const Body& body) {
			const systems::Systems<W> systems{ mWorld, mOptions, mScratch };
			return systems.liquids.observeHead(body);
		}

		[[nodiscard]] ReplayResult replay(const Body& initial, const std::span<const Input> inputs) {
			ReplayResult replayed{ .body = initial };
			replayed.frames.reserve(inputs.size());
			for (const auto& input : inputs) {
				const auto result = simulate(replayed.body, input);
				replayed.frames.push_back({
					.body = replayed.body,
					.result = result,
					.headLiquid = observeHeadLiquid(replayed.body)
				});
			}

			return replayed;
		}

		[[nodiscard]] std::vector<ReplayFrame> run(const Body& initial, const Input& input, const int ticks, const bool stopOnLand = false) {
			std::vector<ReplayFrame> frames;
			frames.reserve(static_cast<size_t>(std::max(ticks, 0)));
			Body body = initial;
			for (int tick = 0; tick < ticks; ++tick) {
				const auto result = simulate(body, input);
				frames.push_back({
					.body = body,
					.result = result,
					.headLiquid = observeHeadLiquid(body)
				});

				if (result.outcome != Outcome::Normal || (stopOnLand && result.landed)) {
					break;
				}
			}

			return frames;
		}

		[[nodiscard]] static constexpr bool hasLiquidLayer() {
			return HasLiquids<W>;
		}

		[[nodiscard]] Options& options() {
			return mOptions;
		}

		[[nodiscard]] const Options& options() const {
			return mOptions;
		}

	private:
		[[nodiscard]] Result resultFromState(const Body& body, const Outcome outcome) const {
			Result result{
				.position = body.position,
				.velocity = body.velocity,
				.movement = body.movement,
				.sprintMovementBlocked = outcome == Outcome::Normal && body.sprintMovementBlocked,
				.onGround = body.onGround,
				.collideX = body.collideX,
				.collideY = body.collideY,
				.collideZ = body.collideZ,
				.positionDelta = body.position - body.client.position,
				.velocityDelta = body.velocity - body.client.velocity,
				.outcome = outcome
			};

			const bool positionDrift = mOptions.positionCorrectionThreshold > 0.0f && length(result.positionDelta) > mOptions.positionCorrectionThreshold;
			const bool velocityDrift = mOptions.velocityCorrectionThreshold > 0.0f && length(result.velocityDelta) > mOptions.velocityCorrectionThreshold;
			switch (mOptions.mode) {
			case SimulationMode::Passive:
				result.needsCorrection = false;
				break;
			case SimulationMode::Permissive:
				result.needsCorrection = positionDrift;
				break;
			case SimulationMode::Authoritative:
			default:
				result.needsCorrection = positionDrift || velocityDrift;
				break;
			}

			return result;
		}

		[[nodiscard]] Result invalidResult(const Body* body) const {
			Result result{
				.needsCorrection = mOptions.mode != SimulationMode::Passive,
				.outcome = Outcome::InvalidInput
			};

			if (!body || !isFinite(*body)) {
				return result;
			}

			result.position = body->position;
			result.velocity = body->velocity;
			result.movement = body->movement;
			result.onGround = body->onGround;
			result.collideX = body->collideX;
			result.collideY = body->collideY;
			result.collideZ = body->collideZ;
			result.positionDelta = body->position - body->client.position;
			result.velocityDelta = body->velocity - body->client.velocity;
			return result;
		}

		const W& mWorld;
		Options mOptions;
		Scratch mScratch;
	};
}
