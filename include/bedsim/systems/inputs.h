#pragma once

#include "bedsim/body.h"
#include "bedsim/constants.h"
#include "bedsim/input.h"
#include "bedsim/physics.h"
#include "bedsim/systems/collision.h"
#include "bedsim/systems/context.h"
#include "bedsim/systems/poses.h"
#include "bedsim/vec.h"

#include <algorithm>

namespace bedsim::systems {
	struct AppliedInput {
		bool poseKnown{};
		Vec2 move{};
	};

	template <World W>
	struct FitProbe {
		const Collision<W>& collision;
		bool known{ true };

		[[nodiscard]] bool fits(const Body& body, const float height) {
			const auto fit = collision.canFitHeight(body, height);
			known = known && fit.known;
			return fit.known && fit.fits;
		}
	};

	template <World W>
	class Inputs {
	public:
		Inputs(const Context<W>& context, const Collision<W>& collision, const Poses<W>& poses)
			: mContext(context), mCollision(collision), mPoses(poses) { }

		[[nodiscard]] AppliedInput apply(Body& body, const Input& input) const {
			const auto& options = mContext.options();
			const bool wasFlightPose = body.gliding || body.riptideTicks > 0;
			body.ensurePoseHeights();
			const bool collisionsAvailable = mCollision.poseCollisionsAvailable(body);
			FitProbe<W> probe{ mCollision };

			body.client.horizontalCollision = input.horizontalCollision;
			body.client.verticalCollision = input.verticalCollision;
			body.client.lastPosition = body.client.position;
			body.client.position = input.clientPosition;
			body.client.lastVelocity = body.client.velocity;
			body.client.velocity = input.clientVelocity;
			body.client.lastMovement = body.client.movement;
			body.client.movement = body.client.position - body.client.lastPosition;

			if (input.startFlying) {
				body.client.toggledFly = true;
				if (body.trustFlyStatus) {
					body.flying = true;
				}
			} else if (input.stopFlying) {
				if (body.flying) {
					body.justDisabledFlight = true;
				}

				body.flying = false;
				body.client.toggledFly = false;
			}

			body.setRotation({ input.pitch, input.headYaw, input.yaw });

			body.pressingSneak = input.sneaking;
			body.pressingSprint = input.sprintDown;
			body.pressingAscend = input.ascendBlock || input.jumping;
			body.pressingDescend = input.descendBlock || input.sneaking;

			applySprint(body, input, options.sprintTiming == SprintTiming::Modern);
			applySneak(body, input, collisionsAvailable, probe);
			applyCrawl(body, input, collisionsAvailable, probe);

			const bool wasSwimming = body.swimming;
			body.stoppedSwimmingThisTick = wasSwimming && input.stopSwimming;
			bool swimPoseKnown = true;
			if (input.stopSwimming) {
				body.swimming = false;
				swimPoseKnown = mPoses.restoreUprightPose(body, collisionsAvailable);
			} else if (input.startSwimming) {
				body.swimming = true;
				if (body.swimPose() || (collisionsAvailable && probe.fits(body, body.standingHeight))) {
					setSwimmingPoseFlags(body);
				}
			}

			body.swimAmount = std::clamp(body.swimAmount + (wasSwimming ? 0.1f : -0.1f), 0.0f, 1.0f);
			body.autoJumpingInWater = input.autoJumpingInWater;
			body.wantDown = input.wantDown;
			body.wantDownSlow = input.wantDownSlow;

			const auto move = processMove(body, input, options.upstreamImpulseClamping);

			body.jumping = input.startJumping;
			body.pressingJump = input.jumping;
			body.effectiveJumping = input.jumping || input.autoJumpingInWater || input.ascendBlock;
			body.jumpHeight = body.jumpStrength > 0.0f ? body.jumpStrength : kJumpHeight;
			if (const auto amplifier = mContext.effectAmplifier(Effect::JumpBoost)) {
				body.jumpHeight += static_cast<float>(*amplifier + 1) * 0.1f;
			}

			if (!body.pressingJump) {
				body.jumpDelay = 0;
			}

			if (body.gravity == 0.0f) {
				body.gravity = kGravity;
			}

			body.slowFalling = mContext.effectAmplifier(Effect::SlowFalling).has_value();

			if (input.stopGliding) {
				body.gliding = false;
			} else if (input.startGliding) {
				body.gliding = true;
			}

			body.startingSpinAttack = input.startSpinAttack || (body.riptideReady && body.startingSpinAttack);
			if (input.stopSpinAttack && body.riptideTicks > 0 && body.riptideCollision) {
				body.riptideTicks = 0;
				body.riptideCollision = false;
				body.setVelocity(body.velocity * -0.2f);
			}

			bool flightPoseKnown = true;
			if (wasFlightPose && !body.gliding && body.riptideTicks == 0 && !body.swimPose()) {
				flightPoseKnown = mPoses.restoreUprightPose(body, collisionsAvailable);
			}

			body.impulse = move * kImpulseScale;
			const bool poseKnown = collisionsAvailable && probe.known && swimPoseKnown && flightPoseKnown;
			return { .poseKnown = poseKnown, .move = move };
		}

	private:
		static void applySprint(Body& body, const Input& input, const bool modern) {
			const bool start = input.startSprinting;
			const bool stop = input.stopSprinting;
			bool adjustSpeed = false;
			if (start && stop) {
				adjustSpeed = modern;
				body.sprinting = false;
			} else if (!start && !stop && !body.serverSprintApplied && body.serverSprint != body.sprinting) {
				body.sprinting = body.serverSprint;
			} else if (start) {
				body.sprinting = true;
				adjustSpeed = modern;
			} else if (stop) {
				body.sprinting = false;
				adjustSpeed = modern && !body.serverUpdatedSpeed;
			}

			body.serverSprintApplied = true;
			if (adjustSpeed) {
				body.serverUpdatedSpeed = false;
				body.movementSpeed = body.defaultMovementSpeed;
				if (body.sprinting) {
					body.movementSpeed *= kSprintSpeedMultiplier;
				}
			}

			body.airSpeed = effectiveAirSpeed(body);
		}

		static void applySneak(Body& body, const Input& input, const bool collisionsAvailable, FitProbe<W>& probe) {
			if (input.startSneaking) {
				body.sneaking = true;
				if (!body.crawling) {
					body.size.y = body.sneakingHeight;
				}
			} else if (input.stopSneaking) {
				if (body.crawling) {
					body.sneaking = false;
				} else if (collisionsAvailable && probe.fits(body, body.standingHeight)) {
					body.sneaking = false;
					body.size.y = body.standingHeight;
				} else {
					body.sneaking = true;
					body.size.y = body.sneakingHeight;
				}
			} else if (body.crawling) {
				body.sneaking = false;
			} else if (input.sneakDown) {
				body.sneaking = true;
				body.size.y = body.sneakingHeight;
			} else if (body.sneaking && (!collisionsAvailable || !probe.fits(body, body.standingHeight))) {
				body.size.y = body.sneakingHeight;
			} else {
				body.sneaking = false;
				body.size.y = body.standingHeight;
			}
		}

		static void applyCrawl(Body& body, const Input& input, const bool collisionsAvailable, FitProbe<W>& probe) {
			const bool wantSneak = !input.stopSneaking && (input.sneakDown || input.startSneaking);
			if (input.startCrawling) {
				if (collisionsAvailable && !probe.fits(body, body.standingHeight)) {
					body.crawling = true;
					body.sneaking = false;
					body.size.y = body.crawlingHeight;
				}
			} else if (input.stopCrawling) {
				const float target = wantSneak ? body.sneakingHeight : body.standingHeight;
				if (collisionsAvailable && probe.fits(body, target)) {
					body.crawling = false;
					body.sneaking = wantSneak;
					body.size.y = target;
				} else {
					body.crawling = true;
					body.size.y = body.crawlingHeight;
				}
			}
		}

		[[nodiscard]] Vec2 processMove(Body& body, const Input& input, const bool upstreamImpulseClamping) const {
			float maxImpulse = 1.0f;
			if (input.moveVectorIsRaw || !upstreamImpulseClamping) {
				if (input.itemUseMovementModifier) {
					maxImpulse *= *input.itemUseMovementModifier;
				} else if (input.usingConsumable || (input.usingItem && !input.usingSpear)) {
					maxImpulse *= kMaxConsumingImpulse;
				}

				if (body.sneaking || body.crawling || body.gliding) {
					++body.ticksSinceCanSlowdown;
					float sneakImpulse = kMaxSneakImpulse;
					if (body.ticksSinceCanSlowdown > 2 && mContext.hasEquipment()) {
						sneakImpulse += 0.15f * static_cast<float>(mContext.enchantmentLevel(Enchantment::SwiftSneak));
					}

					maxImpulse *= std::clamp(sneakImpulse, 0.0f, 1.0f);
				} else {
					body.ticksSinceCanSlowdown = 0;
				}
			}

			Vec2 move = clamp(input.moveVector, Vec2{ -maxImpulse }, Vec2{ maxImpulse });
			if (input.moveVectorIsRaw) {
				move = clamp(input.moveVector, Vec2{ -1.0f }, Vec2{ 1.0f }) * maxImpulse;
			}

			if (input.inventoryAction) {
				move = {};
			}

			return move;
		}

		const Context<W>& mContext;
		const Collision<W>& mCollision;
		const Poses<W>& mPoses;
	};
}
