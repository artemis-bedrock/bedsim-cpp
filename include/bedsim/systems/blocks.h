#pragma once

#include "bedsim/body.h"
#include "bedsim/physics.h"
#include "bedsim/systems/context.h"
#include "bedsim/vec.h"

#include <algorithm>
#include <cmath>
#include <optional>

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

		[[nodiscard]] BlockMovement blockUnderAfterMove(const Body& body) const {
			if (body.supportingBlock) {
				return mContext.movement(*body.supportingBlock);
			}

			const auto under = mContext.movement(blockAt(body.position - Vec3{ 0.0f, 0.2f, 0.0f }));
			if (!under.air) {
				return under;
			}

			const auto below = mContext.movement(blockAt(body.position) + faceOffset(Face::Down));
			return below.fenceLike ? below : under;
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
