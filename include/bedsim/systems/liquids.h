#pragma once

#include "bedsim/body.h"
#include "bedsim/constants.h"
#include "bedsim/physics.h"
#include "bedsim/result.h"
#include "bedsim/systems/blocks.h"
#include "bedsim/systems/collision.h"
#include "bedsim/systems/context.h"
#include "bedsim/systems/poses.h"
#include "bedsim/vec.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <span>
#include <vector>

namespace bedsim::systems {
	template <World W>
	class Liquids {
	public:
		Liquids(const Context<W>& context, const Collision<W>& collision, const Blocks<W>& blocks, const Poses<W>& poses)
			: mContext(context), mCollision(collision), mBlocks(blocks), mPoses(poses) { }

		[[nodiscard]] std::optional<Liquid> liquidAt(const BlockPos& position) const {
			return mContext.liquid(position);
		}

		void touchingBlocks(const Body& body, const LiquidKind kind, std::vector<BlockPos>& out) const {
			out.clear();
			const Vec3 offset = kind == LiquidKind::Lava ? Vec3{ 0.1f, 0.4f, 0.1f } : Vec3{ 0.001f, 0.401f, 0.001f };
			const auto box = shrinkLiquidBox(mContext.boundingBox(body), offset);
			const auto minimum = blockAt(box.min);
			const auto maximum = blockAt(box.max + Vec3{ 1.0f });
			for (int x = minimum.x; x < maximum.x; ++x) {
				for (int y = minimum.y; y < maximum.y; ++y) {
					for (int z = minimum.z; z < maximum.z; ++z) {
						const BlockPos position{ x, y, z };
						const auto liquid = liquidAt(position);
						if (!liquid || liquid->kind != kind || !liquidIntersects(box, position, *liquid)) {
							continue;
						}

						mContext.debug("liquid block type={} pos={} depth={} falling={} boxY=[{:.6f} {:.6f}]", kind == LiquidKind::Lava ? "lava" : "water", position, liquid->depth, liquid->falling, box.min.y, box.max.y);
						out.push_back(position);
					}
				}
			}
		}

		[[nodiscard]] bool containsAnyLiquid(const AABB& box) const {
			const auto minimum = blockAt(box.min);
			const auto maximum = blockCeiling(box.max);
			for (int x = minimum.x; x < maximum.x; ++x) {
				for (int z = minimum.z; z < maximum.z; ++z) {
					for (int y = minimum.y; y < maximum.y; ++y) {
						const BlockPos position{ x, y, z };
						if (const auto liquid = liquidAt(position); liquid && liquidIntersects(box, position, *liquid)) {
							return true;
						}
					}
				}
			}

			return false;
		}

		[[nodiscard]] bool applyFlow(Body& body, const std::span<const BlockPos> positions, const LiquidKind kind) const {
			if (positions.empty()) {
				return true;
			}

			if (!mContext.areaLoaded(mContext.boundingBox(body).grow(1.0f))) {
				return false;
			}

			Vec3 total{};
			for (const auto& position : positions) {
				const auto liquid = liquidAt(position);
				if (!liquid || liquid->kind != kind) {
					continue;
				}

				total += flow(position, *liquid);
			}

			if (const float magnitude = length(total); magnitude >= 1e-4f) {
				const float strength = kind == LiquidKind::Lava ? 0.0035f : 0.014f;
				body.setVelocity(body.velocity + total * (strength / magnitude));
				mContext.debug("{} flow applied strength={:.6f} flow={} vel={}", kind == LiquidKind::Lava ? "lava" : "water", strength, total, body.velocity);
			}

			return true;
		}

		[[nodiscard]] bool travel(Body& body, const LiquidKind kind, const bool touchingLiquid) const {
			const float initialY = body.position.y;
			const bool water = kind == LiquidKind::Water;
			const bool jumping = body.effectiveJumping;
			if (water) {
				if (body.wantDown || body.wantDownSlow || body.pressingDescend) {
					body.setVelocity(body.velocity - Vec3{ 0.0f, 0.04f, 0.0f });
				}

				updateSwimTravel(body);
			}

			if (jumping) {
				auto velocity = body.velocity;
				if ((body.swimAmount > 0.0f && body.swimAmount < 1.0f) || (water && body.swimming && !touchingLiquid)) {
					velocity.y = 0.0f;
				} else {
					velocity.y += 0.04f;
				}

				body.setVelocity(velocity);
			}

			float speed = body.lavaMovementSpeed != 0.0f ? body.lavaMovementSpeed : kLavaMovementSpeed;
			float depthStrider = 0.0f;
			float swimSpeedMultiplier = kSwimSpeedMultiplier;
			if (water) {
				speed = body.underwaterMovementSpeed != 0.0f ? body.underwaterMovementSpeed : kUnderwaterMovementSpeed;
				if (body.swimming && body.swimSpeedMultiplier != 0.0f) {
					swimSpeedMultiplier = body.swimSpeedMultiplier;
				}

				depthStrider = std::min(std::max(static_cast<float>(mContext.enchantmentLevel(Enchantment::DepthStrider)), 0.0f), 3.0f);
				const float depthStriderFraction = depthStrider / 3.0f;
				if (swimSpeedMultiplier > 1.0f) {
					speed *= (0.7f + depthStriderFraction * 0.3f) * swimSpeedMultiplier;
				} else {
					if (!body.onGround) {
						depthStrider *= 0.5f;
					}

					speed += (body.movementSpeed - speed) * (depthStrider / 3.0f);
				}
			}

			moveRelative(body, speed);
			const bool stuck = applyStuckSpeedMultiplier(body);
			if (!mCollision.movementSweepLoaded(body)) {
				return false;
			}

			auto previousVelocity = body.velocity;
			const bool previousOnGround = body.onGround;
			if (!mCollision.tryCollisions(body)) {
				return false;
			}

			if (!mPoses.stopRiptideOnBlockCollision(body)) {
				return false;
			}

			if (stuck) {
				body.setMovement(body.velocity);
				body.setVelocity({});
				previousVelocity = {};
			}

			setPostCollisionMotion(body, previousVelocity, previousOnGround, BlockMovement{ .air = true });
			if (!stuck) {
				body.setMovement(body.velocity);
			}

			auto velocity = body.velocity;
			if (water) {
				float drag = body.sprinting || body.stoppedSwimmingThisTick ? 0.9f : 0.8f;
				if (depthStrider > 0.0f && swimSpeedMultiplier <= 1.0f) {
					drag += (0.54600006f - drag) * (depthStrider / 3.0f);
				}

				velocity.x *= drag;
				velocity.y *= 0.8f;
				velocity.z *= drag;
			} else {
				velocity *= 0.5f;
			}

			if (const auto levitation = mContext.effectAmplifier(Effect::Levitation)) {
				const float target = kLevitationSpeed * static_cast<float>(*levitation + 1);
				velocity.y += (target - velocity.y) * 0.2f;
			} else if (body.hasGravity) {
				velocity.y -= liquidGravity(body.swimming, water);
			}

			if (body.collideX || body.collideZ) {
				const Vec3 raised{ velocity.x, velocity.y + 0.6f + initialY - body.position.y, velocity.z };
				const auto raisedBox = mContext.boundingBox(body).translate(raised);
				if (!mContext.areaLoaded(raisedBox)) {
					return false;
				}

				const bool hasCollision = mCollision.hasNearbyBoxes(body, raisedBox);
				const bool hasLiquid = containsAnyLiquid(raisedBox);
				mContext.debug("liquid exit probe collision={} liquid={} box={}", hasCollision, hasLiquid, raisedBox);
				if (!hasCollision && !hasLiquid) {
					velocity.y = 0.3f;
				}
			}

			body.setVelocity(velocity);
			applyBubbleColumns(body);
			mBlocks.applyInsideBlockEffects(body);
			body.fallDistance = 0.0f;
			return true;
		}

		void applyBubbleColumns(Body& body) const {
			if constexpr (HasBubbleColumns<W>) {
				const auto box = mContext.boundingBox(body);
				const auto minimum = blockAt(box.min);
				const auto maximum = blockCeiling(box.max);
				bool found = false;
				for (int x = minimum.x; x < maximum.x; ++x) {
					for (int y = minimum.y; y < maximum.y; ++y) {
						for (int z = minimum.z; z < maximum.z; ++z) {
							const BlockPos position{ x, y, z };
							const auto direction = mContext.world().bubbleColumn(position);
							if (!direction) {
								continue;
							}

							found = true;
							applyBubbleColumn(body, *direction, isBubbleSurface(position));
						}
					}
				}

				if (found) {
					body.fallDistance = 0.0f;
				}
			}
		}

		[[nodiscard]] HeadLiquid observeHead(const Body& body) const {
			HeadLiquid observation{};
			if (!isFinite(body)) {
				return observation;
			}

			const auto position = body.eyePosition();
			observation.position = position;
			const auto cell = blockAt(position);
			if (!mContext.areaLoaded(AABB::unitAt(cell)) || (mContext.options().requireLiquidLayer && !HasLiquids<W>)) {
				return observation;
			}

			observation.known = true;
			const auto liquid = liquidAt(cell);
			if (!liquid || position.y >= static_cast<float>(cell.y) + liquidHeight(*liquid)) {
				return observation;
			}

			observation.water = liquid->kind == LiquidKind::Water;
			observation.lava = liquid->kind == LiquidKind::Lava;
			return observation;
		}

	private:
		struct FlowFace {
			BlockPos delta;
			Vec3 direction;
		};

		static constexpr std::array kFlowFaces{
			FlowFace{ { -1, 0, 0 }, { -1.0f, 0.0f, 0.0f } },
			FlowFace{ { 1, 0, 0 }, { 1.0f, 0.0f, 0.0f } },
			FlowFace{ { 0, 0, -1 }, { 0.0f, 0.0f, -1.0f } },
			FlowFace{ { 0, 0, 1 }, { 0.0f, 0.0f, 1.0f } }
		};

		void updateSwimTravel(Body& body) const {
			if (!body.swimming || body.effectiveJumping) {
				return;
			}

			const float target = -mcSin(body.rotation.x * std::numbers::pi_v<float> / 180.0f);
			const float rate = target < -0.2f ? 0.085f : 0.06f;
			if (target > 0.0f && !body.wantDownSlow && !body.pressingDescend) {
				const auto below = blockAt(body.position + Vec3{ 0.0f, kEyeHeight - 1.1f, 0.0f });
				if (!liquidAt(below) && mContext.isAir(below)) {
					const auto surface = blockAt(body.position + Vec3{ 0.0f, kEyeHeight - 1.2f, 0.0f });
					if (!liquidAt(surface)) {
						body.setVelocity({ body.velocity.x, 0.0f, body.velocity.z });
						return;
					}
				}
			}

			auto velocity = body.velocity;
			velocity.y += (target - velocity.y) * rate;
			body.setVelocity(velocity);
		}

		[[nodiscard]] Vec3 flow(const BlockPos& position, const Liquid& liquid) const {
			const int currentDecay = liquidDecay(liquid);
			Vec3 total{};
			for (const auto& face : kFlowFaces) {
				const auto neighbour = position + face.delta;
				if (const auto other = liquidAt(neighbour); other && other->kind == liquid.kind) {
					if (!isSideClosed(position, neighbour) && !isSideClosed(neighbour, position)) {
						total += face.direction * static_cast<float>(liquidDecay(*other) - currentDecay);
					}

					continue;
				}

				if (isSideClosed(position, neighbour) || isSideClosed(neighbour, position)) {
					continue;
				}

				const auto below = neighbour + faceOffset(Face::Down);
				if (const auto lower = liquidAt(below); lower && lower->kind == liquid.kind) {
					total += face.direction * static_cast<float>(liquidDecay(*lower) - currentDecay + 8);
				}
			}

			if (liquid.falling) {
				for (const auto& face : kFlowFaces) {
					const auto neighbour = position + face.delta;
					const auto above = neighbour + faceOffset(Face::Up);
					if (isFlowBarrier(neighbour) || isFlowBarrier(above)) {
						if (const float magnitude = length(total); magnitude > 1e-4f) {
							total *= 1.0f / magnitude;
						}

						total.y -= 6.0f;
						break;
					}
				}
			}

			if (const float magnitude = length(total); magnitude > 1e-4f) {
				return total * (1.0f / magnitude);
			}

			return {};
		}

		[[nodiscard]] bool isSideClosed(const BlockPos& position, const BlockPos& side) const {
			const auto face = faceTowards(position, side);
			if constexpr (HasLiquidFlow<W>) {
				return mContext.world().liquidFaceClosed(position, face);
			} else {
				return isFaceSolid(position, face);
			}
		}

		[[nodiscard]] bool isFlowBarrier(const BlockPos& position) const {
			if constexpr (HasLiquidFlow<W>) {
				return mContext.world().liquidFlowBarrier(position);
			} else {
				auto& local = mContext.scratch().blockBoxes;
				local.clear();
				mContext.blockCollisions(position, local);
				return !local.empty();
			}
		}

		[[nodiscard]] bool isFaceSolid(const BlockPos& position, const Face face) const {
			auto& local = mContext.scratch().blockBoxes;
			local.clear();
			mContext.blockCollisions(position, local);
			const auto normal = faceOffset(face);
			const int axis = normal.x != 0 ? 0 : (normal.y != 0 ? 1 : 2);
			const int first = (axis + 1) % 3;
			const int second = (axis + 2) % 3;
			const auto isFlush = [&](const AABB& box) {
				return normal[axis] > 0 ? box.max[axis] >= 1.0f : box.min[axis] <= 0.0f;
			};

			auto& firstCuts = mContext.scratch().firstCuts;
			auto& secondCuts = mContext.scratch().secondCuts;
			firstCuts.assign({ 0.0f, 1.0f });
			secondCuts.assign({ 0.0f, 1.0f });
			for (const auto& box : local) {
				if (!isFlush(box)) {
					continue;
				}

				firstCuts.push_back(std::clamp(box.min[first], 0.0f, 1.0f));
				firstCuts.push_back(std::clamp(box.max[first], 0.0f, 1.0f));
				secondCuts.push_back(std::clamp(box.min[second], 0.0f, 1.0f));
				secondCuts.push_back(std::clamp(box.max[second], 0.0f, 1.0f));
			}

			std::ranges::sort(firstCuts);
			std::ranges::sort(secondCuts);
			for (size_t i = 0; i + 1 < firstCuts.size(); ++i) {
				for (size_t j = 0; j + 1 < secondCuts.size(); ++j) {
					if (firstCuts[i] == firstCuts[i + 1] || secondCuts[j] == secondCuts[j + 1]) {
						continue;
					}

					const float u = (firstCuts[i] + firstCuts[i + 1]) * 0.5f;
					const float v = (secondCuts[j] + secondCuts[j + 1]) * 0.5f;
					const bool covered = std::ranges::any_of(local, [&](const AABB& box) {
						return isFlush(box) && box.min[first] <= u && box.max[first] >= u && box.min[second] <= v && box.max[second] >= v;
					});

					if (!covered) {
						return false;
					}
				}
			}

			return true;
		}

		[[nodiscard]] bool isBubbleSurface(const BlockPos& position) const {
			if constexpr (HasBubbleColumnSurface<W>) {
				if (const auto surface = mContext.world().bubbleColumnSurface(position)) {
					return *surface;
				}
			}

			const auto above = position + faceOffset(Face::Up);
			return !liquidAt(above) && mContext.isAir(above);
		}

		const Context<W>& mContext;
		const Collision<W>& mCollision;
		const Blocks<W>& mBlocks;
		const Poses<W>& mPoses;
	};
}
