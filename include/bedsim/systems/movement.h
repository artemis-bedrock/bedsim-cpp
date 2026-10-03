#pragma once

#include "bedsim/body.h"
#include "bedsim/constants.h"
#include "bedsim/physics.h"
#include "bedsim/result.h"
#include "bedsim/systems/blocks.h"
#include "bedsim/systems/collision.h"
#include "bedsim/systems/context.h"
#include "bedsim/systems/liquids.h"
#include "bedsim/systems/poses.h"
#include "bedsim/systems/riptide.h"
#include "bedsim/vec.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace bedsim::systems {
	template <World W>
	class Movement {
	public:
		Movement(const Context<W>& context, const Collision<W>& collision, const Blocks<W>& blocks, const Poses<W>& poses, const Liquids<W>& liquids, const Riptide<W>& riptide)
			: mContext(context), mCollision(collision), mBlocks(blocks), mPoses(poses), mLiquids(liquids), mRiptide(riptide) { }

		[[nodiscard]] Outcome simulateCore(Body& body, const bool consumeTransient) const {
			const auto outcome = runCore(body, consumeTransient);
			if (consumeTransient && outcome != Outcome::UnloadedChunk) {
				body.riptideReady = false;
			}

			return outcome;
		}

		void resetToClient(Body& body) const {
			const auto& options = mContext.options();
			body.swimWaterGraceTicks = 0;
			body.swimWaterContact = false;
			body.stuckSpeedMultiplier = {};
			body.lastPosition = body.client.lastPosition;
			body.position = body.client.position;
			body.retainedBox.reset();
			body.lastVelocity = body.client.lastVelocity;
			body.velocity = body.client.velocity;
			body.lastMovement = body.client.lastMovement;
			body.movement = body.client.movement;
			body.supportingBlock.reset();
			if (body.flying || body.noClip) {
				body.onGround = false;
			}

			if (options.limitAllVelocity) {
				const float limit = std::abs(options.limitAllVelocityThreshold);
				body.velocity = clamp(body.velocity, Vec3{ -limit }, Vec3{ limit });
			}
		}

		[[nodiscard]] bool simulationIsReliable(const Body& body) const {
			if (body.remainingTeleportTicks() > 0) {
				return true;
			}

			if (body.gameMode != GameMode::Survival && body.gameMode != GameMode::Adventure) {
				return false;
			}

			return !body.flying && !body.justDisabledFlight && !body.noClip && body.alive;
		}

		[[nodiscard]] bool attemptTeleport(Body& body) const {
			const bool useSlideOffset = mContext.options().useSlideOffset;
			if (body.pendingTeleports > 0 && (body.teleportPending || body.pendingTeleportPosition != Vec3{})) {
				body.teleportPosition = body.pendingTeleportPosition;
			}

			if (!body.hasTeleport()) {
				return false;
			}

			if (!body.teleportSmoothed) {
				body.setPosition(body.teleportPosition);
				body.rememberCollisionBox(body.collisionBoxAt(body.position, useSlideOffset), useSlideOffset);
				body.supportingBlock.reset();
				body.setVelocity({});
				body.jumpDelay = 0;
				finishTeleport(body);
				return true;
			}

			const auto delta = body.teleportPosition - body.position;
			int remaining = body.remainingTeleportTicks();
			if (remaining < std::numeric_limits<int>::max()) {
				++remaining;
			}

			body.setPosition(body.position + delta * (1.0f / static_cast<float>(remaining)));
			body.rememberCollisionBox(body.collisionBoxAt(body.position, useSlideOffset), useSlideOffset);
			body.supportingBlock.reset();
			body.jumpDelay = 0;
			if (remaining == 1) {
				finishTeleport(body);
			}

			return true;
		}

	private:
		static void clearObservedLiquid(Body& body) {
			body.swimWaterGraceTicks = 0;
			body.swimWaterContact = false;
			body.stuckSpeedMultiplier = {};
		}

		static void finishTeleport(Body& body) {
			body.teleportPending = false;
			if (body.pendingTeleports > 0) {
				--body.pendingTeleports;
			}

			if (body.pendingTeleports == 0) {
				body.pendingTeleportPosition = {};
			}

			completeTeleport(body);
		}

		[[nodiscard]] Outcome runCore(Body& body, const bool consumeTransient) const {
			body.sprintMovementBlocked = false;
			body.ensurePoseHeights();
			if (attemptTeleport(body)) {
				clearObservedLiquid(body);
				return Outcome::Teleport;
			}

			if (body.inVehicle) {
				resetToClient(body);
				body.onGround = false;
				body.collideX = false;
				body.collideY = false;
				body.collideZ = false;
				return Outcome::Mounted;
			}

			if (!simulationIsReliable(body)) {
				resetToClient(body);
				return Outcome::Unreliable;
			}

			if (mContext.options().requireLiquidLayer && !HasLiquids<W>) {
				mContext.debug("no liquid layer available and RequireLiquidLayer is set");
				resetToClient(body);
				return Outcome::Unreliable;
			}

			mCollision.prepareCollisionBox(body);
			const auto currentArea = mContext.boundingBox(body);
			if (!mContext.areaLoaded(currentArea)) {
				body.setVelocity({});
				clearObservedLiquid(body);
				return Outcome::UnloadedChunk;
			}

			if (body.immobile || !body.ready) {
				body.setVelocity({});
				clearObservedLiquid(body);
				return Outcome::ImmobileOrNotReady;
			}

			const auto sweepVelocity = body.hasKnockback() ? body.knockback : body.velocity;
			if (!mContext.areaLoaded(movementProbeArea(currentArea.extend(sweepVelocity)))) {
				body.setVelocity({});
				clearObservedLiquid(body);
				return Outcome::UnloadedChunk;
			}

			const Body prePhysics = body;
			bool known = simulateMovement(body);
			if (known && consumeTransient && body.riptideTicks == 1 && !body.gliding && !body.swimPose()) {
				known = mPoses.restoreUprightPose(body);
			}

			if (!known) {
				body = prePhysics;
				body.setVelocity({});
				clearObservedLiquid(body);
				return Outcome::UnloadedChunk;
			}

			return Outcome::Normal;
		}

		[[nodiscard]] bool simulateMovement(Body& body) const {
			auto velocity = body.velocity;
			for (int axis = 0; axis < 3; ++axis) {
				if (std::abs(velocity[axis]) < 1e-8f) {
					velocity[axis] = 0.0f;
				}
			}

			body.setVelocity(velocity);

			const bool wasSwimPose = body.swimPose();
			const auto grace = mContext.options().resolvedSwimWaterGraceTicks();
			body.swimWaterGraceTicks = std::min(body.swimWaterGraceTicks, grace);

			auto& scratch = mContext.scratch();
			mLiquids.touchingBlocks(body, LiquidKind::Water, scratch.water);
			mLiquids.touchingBlocks(body, LiquidKind::Lava, scratch.lava);
			const bool inWater = !scratch.water.empty();
			body.swimWaterContact = inWater;
			if (wasSwimPose && !body.swimPose() && !body.gliding && body.riptideTicks == 0) {
				if (!mPoses.restoreUprightPose(body)) {
					return false;
				}
			}

			if (inWater && body.swimming) {
				body.swimWaterGraceTicks = grace;
				setSwimmingPoseFlags(body);
			}

			bool known = travel(body, inWater);

			const bool hadSwimPose = body.swimPose();
			if (inWater) {
				body.swimWaterGraceTicks = grace;
			} else if (body.swimWaterGraceTicks > 0) {
				--body.swimWaterGraceTicks;
			}

			if (known && hadSwimPose && !body.swimPose() && !body.gliding && body.riptideTicks == 0) {
				known = mPoses.restoreUprightPose(body);
			}

			return known;
		}

		[[nodiscard]] bool travel(Body& body, const bool inWater) const {
			const auto& scratch = mContext.scratch();
			if (!body.flying) {
				const auto launch = mRiptide.attempt(body, inWater);
				if (!launch.known) {
					return false;
				}

				if (launch.launched) {
					mContext.debug("riptide launch applied: {}", body.velocity);
				}
			}

			const bool touchingLava = !scratch.lava.empty();
			const bool waterTravel = inWater || (body.swimming && body.swimWaterGraceTicks > 0 && !touchingLava);
			if (!body.flying && (waterTravel || touchingLava)) {
				if (attemptKnockback(body)) {
					mContext.debug("knockback applied in liquid: {}", body.velocity);
				}

				if (waterTravel) {
					if (body.gliding) {
						if (!mPoses.stopGliding(body)) {
							return false;
						}

						body.glideBoostTicks = 0;
					}

					return mLiquids.applyFlow(body, scratch.water, LiquidKind::Water) && mLiquids.travel(body, LiquidKind::Water, inWater);
				}

				return mLiquids.applyFlow(body, scratch.lava, LiquidKind::Lava) && mLiquids.travel(body, LiquidKind::Lava, true);
			}

			const auto under = mContext.movement(blockAt(body.position - Vec3{ 0.0f, 0.5f, 0.0f }));
			float friction = kAirFriction;
			float speed = body.airSpeed;
			if (body.onGround) {
				float accelerationFriction = under.accelerationFriction;
				if (under.soulSpeedNeutralizesFriction && mContext.enchantmentLevel(Enchantment::SoulSpeed) > 0) {
					accelerationFriction = 1.0f;
				}

				friction *= under.groundFriction;
				speed = groundAcceleration(body.movementSpeed, under.groundFriction, accelerationFriction);
			}

			if (body.gliding && mContext.effectAmplifier(Effect::Levitation)) {
				if (!mPoses.stopGliding(body)) {
					return false;
				}
			}

			if (body.gliding) {
				const bool hasElytra = mContext.hasElytra();
				if (hasElytra && !body.onGround) {
					return glide(body);
				}

				if (!mPoses.stopGliding(body)) {
					return false;
				}

				mContext.debug("cannot allow glide (onGround={} hasElytra={})", body.onGround, hasElytra);
			}

			if (attemptKnockback(body)) {
				mContext.debug("knockback applied: {}", body.velocity);
			}

			mContext.debug("blockUnder={}, blockFriction={}, speed={}", under.air ? "air" : "block", friction, speed);
			moveRelative(body, speed);
			mContext.debug("moveRelative force applied (vel={})", body.velocity);
			if (attemptJump(body)) {
				mContext.debug("jump force applied (sprint={}): {}", body.sprinting, body.velocity);
			}

			const auto inside = mContext.movement(blockAt(body.position));
			const bool scaffoldDescend = applyAscendableMovement(body, mBlocks.traversal(body, inside), mContext.wearingLeatherBoots());

			const bool nearClimbable = mBlocks.climbableContact(body, inside.climbable);
			if (nearClimbable) {
				auto climb = body.velocity;
				climb.y = std::max(climb.y, -kClimbSpeed);
				if (body.effectiveJumping || body.collideX || body.collideZ) {
					climb.y = kClimbSpeed;
				}

				if (body.sneaking && climb.y < 0.0f) {
					climb.y = 0.0f;
				}

				body.setVelocity(climb);
				mContext.debug("added climb velocity: {} (collided={} effectiveJumping={})", climb, body.collideX || body.collideZ, body.effectiveJumping);
			}

			const bool inCobweb = mBlocks.insideCobweb(body);
			if (inCobweb) {
				const bool weaving = mContext.effectAmplifier(Effect::Weaving).has_value();
				const float horizontal = weaving ? 0.5f : 0.25f;
				const float vertical = weaving ? 0.25f : 0.05f;
				body.setVelocity(body.velocity * Vec3{ horizontal, vertical, horizontal });
				mContext.debug("web force applied (vel={})", body.velocity);
			}

			const bool stuck = applyStuckSpeedMultiplier(body);
			if (!mCollision.movementSweepLoaded(body) || !mCollision.avoidEdge(body)) {
				return false;
			}

			auto previousVelocity = body.velocity;
			const bool previousOnGround = body.onGround;
			const float previousY = body.position.y;
			if (!mCollision.tryCollisions(body) || !mPoses.stopRiptideOnBlockCollision(body)) {
				return false;
			}

			updateFallDistance(body, previousY);
			if (scaffoldDescend || nearClimbable) {
				body.fallDistance = 0.0f;
			}

			const auto blockUnder = mBlocks.blockUnderAfterMove(body);
			if (previousY == body.position.y) {
				walkOnBlock(body, blockUnder);
			} else {
				mContext.debug("walkOnBlock: y changed, skipping block walk effects");
			}

			body.setMovement(body.velocity);
			if (stuck) {
				body.setVelocity({});
				previousVelocity = {};
			}

			setPostCollisionMotion(body, previousVelocity, previousOnGround, blockUnder);
			if (inCobweb) {
				mContext.debug("post-move cobweb force applied (0 vel)");
				body.setVelocity({});
			}

			auto next = body.velocity;
			if (!scaffoldDescend) {
				if (const auto levitation = mContext.effectAmplifier(Effect::Levitation)) {
					const float target = kLevitationSpeed * static_cast<float>(*levitation + 1);
					next.y += (target - next.y) * 0.2f;
				} else if (body.hasGravity) {
					next.y -= effectiveGravity(body, next);
					next.y *= kGravityDrag;
				}
			}

			next.x *= friction;
			next.z *= friction;
			body.setVelocity(next);
			mBlocks.applyInsideBlockEffects(body);
			mLiquids.applyBubbleColumns(body);
			return true;
		}

		[[nodiscard]] bool glide(Body& body) const {
			body.onGround = false;
			simulateGlide(body);
			const bool stuck = applyStuckSpeedMultiplier(body);
			if (!mCollision.movementSweepLoaded(body)) {
				return false;
			}

			const auto previousVelocity = body.velocity;
			const float previousY = body.position.y;
			if (!mCollision.tryCollisions(body) || !mPoses.stopRiptideOnBlockCollision(body)) {
				return false;
			}

			updateFallDistance(body, previousY);
			mContext.debug("(glide) oldVel={}, collisions={} diff={}", previousVelocity, body.velocity, body.velocity - body.client.velocity);
			body.setMovement(body.velocity);
			if (stuck) {
				body.setVelocity({});
			}

			mBlocks.applyInsideBlockEffects(body);
			mLiquids.applyBubbleColumns(body);
			return true;
		}

		[[nodiscard]] bool attemptJump(Body& body) const {
			if (!body.jumping || !body.onGround || body.jumpDelay > 0) {
				if (body.jumping) {
					mContext.debug("rejected jump from client (onGround={} jumpDelay={})", body.onGround, body.jumpDelay);
				}

				return false;
			}

			float jumpHeight = body.jumpHeight;
			const bool honeyInside = mContext.movement(blockAt(body.position)).honey;
			const bool honeyBelow = mContext.movement(blockAt(body.position - Vec3{ 0.0f, 0.1f, 0.0f })).honey;
			if (honeyInside || honeyBelow) {
				jumpHeight *= kHoneyJumpMultiplier;
			}

			body.setVelocity(jumpImpulse(body.velocity, jumpHeight, body.rotation.z, body.sprinting));
			body.jumpDelay = kJumpDelayTicks;
			return true;
		}

		const Context<W>& mContext;
		const Collision<W>& mCollision;
		const Blocks<W>& mBlocks;
		const Poses<W>& mPoses;
		const Liquids<W>& mLiquids;
		const Riptide<W>& mRiptide;
	};
}
