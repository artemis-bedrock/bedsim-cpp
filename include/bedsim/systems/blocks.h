#pragma once

#include "bedsim/body.h"
#include "bedsim/physics.h"
#include "bedsim/systems/context.h"
#include "bedsim/vec.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <span>

namespace bedsim::systems {
	template <World W>
	class Blocks {
	public:
		explicit Blocks(const Context<W>& context)
			: mContext(context) { }

		[[nodiscard]] bool insideCobweb(const Body& body) const {
			const auto box = mContext.boundingBox(body);
			const auto area = box.grow(1.0f);
			const auto minimum = blockAt(area.min);
			const auto maximum = blockCeiling(area.max);
			for (int y = minimum.y; y <= maximum.y; ++y) {
				for (int x = minimum.x; x <= maximum.x; ++x) {
					for (int z = minimum.z; z <= maximum.z; ++z) {
						const BlockPos position{ x, y, z };
						const auto movement = mContext.movement(position);
						if (movement.air || !box.intersects(AABB::unitAt(position))) {
							continue;
						}

						if (movement.cobweb) {
							return true;
						}
					}
				}
			}

			return false;
		}

		[[nodiscard]] bool climbableContact(const Body& body, const bool insideClimbable) const {
			if constexpr (HasClimbableContact<W>) {
				return mContext.world().climbableContact(mContext.boundingBox(body));
			} else {
				return insideClimbable;
			}
		}

		[[nodiscard]] std::optional<ClimberFlags> climberFlags(const Body& body, const bool leatherBoots) const {
			const auto box = mContext.boundingBox(body);
			const float feetLayer = std::floor(box.min.y);
			const float overLayer = std::floor(box.min.y + -1.0f);
			const auto probe = AABB::ordered(
				{ std::floor(box.min.x), overLayer - 1.0f, std::floor(box.min.z) },
				{ std::floor(box.max.x) + 1.0f, feetLayer + 1.0f, std::floor(box.max.z) + 1.0f }
			);

			if (!mContext.areaLoaded(probe)) {
				return std::nullopt;
			}

			ClimberFlags flags{};
			climberLayer(box, feetLayer, leatherBoots, flags.inScaffolding, flags.inAscendable);
			climberLayer(box, overLayer, leatherBoots, flags.overScaffolding, flags.overDescendable);
			flags.climbable = climbableAtFeet(body, leatherBoots, flags.holdOnSneak);
			return flags;
		}

		[[nodiscard]] bool climbableAtFeet(const Body& body, const bool leatherBoots, bool& holdOnSneak) const {
			const auto inside = mContext.movement(blockAt(body.position));
			holdOnSneak = climbableContact(body, inside.climbable);
			return holdOnSneak || (leatherBoots && inside.traversal == Traversal::PowderSnow);
		}

		[[nodiscard]] BlockMovement groundFriction(const Body& body) const {
			const float minimumY = mContext.boundingBox(body).min.y;
			const auto under = mContext.movement(blockAt({ body.position.x, minimumY + kGroundFrictionProbeOffset, body.position.z }));
			return under.air ? BlockMovement{ .air = true } : under;
		}

		[[nodiscard]] std::optional<BlockMovement> landingBlock(const AABB& box, const std::span<const AABB> boxes) const {
			const float planeY = box.min.y + kStandingOnProbeOffset;
			const float centerX = (box.max.x - box.min.x) * 0.5f + box.min.x;
			const float centerZ = (box.max.z - box.min.z) * 0.5f + box.min.z;
			const AABB* best = nullptr;
			BlockPos bestPosition{};
			float bestGap = std::numeric_limits<float>::max();
			float bestDistance = 0.0f;
			for (const auto& candidate : boxes) {
				const float candidateY = (candidate.max.y - candidate.min.y) * 0.5f + candidate.min.y;
				const float gap = planeY - candidateY;
				if (gap < 0.0f) {
					continue;
				}

				const float deltaX = (candidate.max.x - candidate.min.x) * 0.5f + candidate.min.x - centerX;
				const float deltaY = candidateY - planeY;
				const float deltaZ = (candidate.max.z - candidate.min.z) * 0.5f + candidate.min.z - centerZ;
				const float distance = (deltaZ * deltaZ + deltaY * deltaY) + deltaX * deltaX;
				const auto position = blockAt(candidate.min);
				if (!best || gap < bestGap || (gap == bestGap && (distance < bestDistance || (distance == bestDistance && gathersBefore(position, bestPosition))))) {
					best = &candidate;
					bestPosition = position;
					bestGap = gap;
					bestDistance = distance;
				}
			}

			if (!best || best->hasZeroVolume()) {
				return std::nullopt;
			}

			return mContext.movement(bestPosition);
		}

		[[nodiscard]] std::optional<BlockMovement> standingOnBlock(const Body& body) const {
			const auto box = mContext.boundingBox(body);
			const float planeY = box.min.y + kStandingOnProbeOffset;
			const Vec3 center{ (box.max.x - box.min.x) * 0.5f + box.min.x, planeY, (box.max.z - box.min.z) * 0.5f + box.min.z };
			const auto minimum = blockAt({ box.min.x, planeY, box.min.z });
			const auto maximum = blockAt({ box.max.x, planeY, box.max.z });
			std::optional<BlockPos> bestPosition;
			float bestTop = std::numeric_limits<float>::lowest();
			float bestDistance = 0.0f;
			for (int x = minimum.x; x <= maximum.x; ++x) {
				for (int z = minimum.z; z <= maximum.z; ++z) {
					for (int y = minimum.y; y >= minimum.y - 1; --y) {
						const BlockPos position{ x, y, z };
						const auto local = collisionShape(position);
						if (!local) {
							continue;
						}

						const auto shape = local->translate(Vec3{ position });
						if (shape.max.x <= box.min.x || shape.min.x >= box.max.x || shape.max.z <= box.min.z || shape.min.z >= box.max.z || shape.max.y <= planeY || shape.min.y >= planeY || bestTop > shape.max.y) {
							continue;
						}

						const Vec3 delta{
							center.x - ((shape.max.x - shape.min.x) * 0.5f + shape.min.x),
							center.y - ((shape.max.y - shape.min.y) * 0.5f + shape.min.y),
							center.z - ((shape.max.z - shape.min.z) * 0.5f + shape.min.z)
						};

						const float distance = (delta.z * delta.z + delta.y * delta.y) + delta.x * delta.x;
						if (bestTop < shape.max.y || distance < bestDistance) {
							bestTop = shape.max.y;
							bestDistance = distance;
							bestPosition = blockAt(shape.min);
						}
					}
				}
			}

			if (!bestPosition) {
				return std::nullopt;
			}

			return mContext.movement(*bestPosition);
		}

		void applyStandOnDamping(Body& body) const {
			if (!body.onGround || body.sneaking || body.velocity.y >= 0.1f) {
				return;
			}

			const auto under = standingOnBlock(body);
			if (!under || (under->bounce != Bounce::Slime && !under->honey)) {
				return;
			}

			const float factor = (std::abs(body.velocity.y) * 0.2f) + 0.4f;
			body.velocity.x *= factor;
			body.velocity.z *= factor;
		}

		[[nodiscard]] bool jumpFactorBlockHoney(const Body& body) const {
			const float minimumY = mContext.boundingBox(body).min.y;
			const float feetY = std::floor(minimumY);
			const auto position = blockAt({ body.position.x, feetY, body.position.z });
			const auto feet = mContext.movement(position);
			if (feet.honey) {
				return true;
			}

			if (minimumY > feetY && !feet.carpet) {
				return false;
			}

			return mContext.movement(position + faceOffset(Face::Down)).honey;
		}

		void applyInsideBlockEffects(Body& body) const {
			const auto box = mContext.boundingBox(body);
			forEachOverlapped(box, [&](const BlockPos& position) {
				const auto movement = mContext.movement(position);
				if (!movement.air) {
					applyInsideBlockMovement(body, movement.inside);
				}
			});

			applyHoneyWallSlide(body);
		}

	private:
		[[nodiscard]] static bool gathersBefore(const BlockPos& first, const BlockPos& second) {
			if (first.x != second.x) {
				return first.x < second.x;
			}

			if (first.z != second.z) {
				return first.z < second.z;
			}

			return first.y < second.y;
		}

		[[nodiscard]] std::optional<AABB> collisionShape(const BlockPos& position) const {
			auto& local = mContext.scratch().blockBoxes;
			local.clear();
			mContext.blockCollisions(position, local);
			std::optional<AABB> shape;
			for (const auto& box : local) {
				if (box.hasZeroVolume()) {
					continue;
				}

				shape = shape ? AABB{ min(shape->min, box.min), max(shape->max, box.max) } : box;
			}

			return shape;
		}

		void climberLayer(const AABB& box, const float layer, const bool leatherBoots, bool& scaffolding, bool& ascendable) const {
			const auto minimum = blockAt({ box.min.x, layer, box.min.z });
			const auto maximum = blockAt({ box.max.x, layer, box.max.z });
			for (int x = minimum.x; x <= maximum.x; ++x) {
				for (int z = minimum.z; z <= maximum.z; ++z) {
					const BlockPos position{ x, minimum.y, z };
					const auto traversal = mContext.movement(position).traversal;
					if (traversal == Traversal::Scaffolding) {
						scaffolding = true;
						if (!ascendable) {
							const auto below = position + faceOffset(Face::Down);
							const auto liquid = mContext.liquid(below);
							ascendable = !mContext.isAir(below) && !(liquid && liquid->kind == LiquidKind::Water);
						}
					} else if (traversal == Traversal::PowderSnow) {
						ascendable = ascendable || leatherBoots;
					}
				}
			}
		}

		void applyHoneyWallSlide(Body& body) const {
			const auto box = mContext.boundingBox(body).grow({ 1e-3f, 0.0f, 1e-3f });
			forEachOverlapped(box, [&](const BlockPos& position) {
				if (!mContext.movement(position).honey) {
					return;
				}

				auto velocity = body.velocity;
				velocity.x *= 0.4f;
				velocity.y = std::max(-0.12f, velocity.y);
				velocity.z *= 0.4f;
				body.setVelocity(velocity);
				if (honeySlideResetsFallDistance(body, position)) {
					body.fallDistance = 0.0f;
				}
			});
		}

		template <typename Fn>
		static void forEachOverlapped(const AABB& box, Fn&& visit) {
			const auto minimum = blockAt(box.min);
			const auto maximum = blockCeiling(box.max);
			for (int x = minimum.x; x < maximum.x; ++x) {
				for (int y = minimum.y; y < maximum.y; ++y) {
					for (int z = minimum.z; z < maximum.z; ++z) {
						const BlockPos position{ x, y, z };
						if (box.intersects(AABB::unitAt(position))) {
							visit(position);
						}
					}
				}
			}
		}

		const Context<W>& mContext;
	};
}
