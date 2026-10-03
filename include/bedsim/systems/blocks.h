#pragma once

#include "bedsim/body.h"
#include "bedsim/physics.h"
#include "bedsim/systems/context.h"
#include "bedsim/vec.h"

#include <algorithm>

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

		[[nodiscard]] Traversal traversal(const Body& body, const BlockMovement& inside) const {
			if (inside.traversal == Traversal::None && body.supportingBlock && mContext.movement(*body.supportingBlock).traversal == Traversal::Scaffolding) {
				return Traversal::Scaffolding;
			}

			return inside.traversal;
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
