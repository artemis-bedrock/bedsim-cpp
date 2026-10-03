#pragma once

#include "bedsim/body.h"
#include "bedsim/systems/collision.h"

namespace bedsim::systems {
	struct PoseSnapshot {
		std::optional<RetainedBox> retainedBox;
		Vec3 size{};
		bool sneaking{};
		bool crawling{};
		bool swimming{};
		float swimAmount{};
		bool gliding{};
		int riptideTicks{};
		bool riptideCollision{};

		[[nodiscard]] static PoseSnapshot capture(const Body& body) {
			return {
				.retainedBox = body.retainedBox,
				.size = body.size,
				.sneaking = body.sneaking,
				.crawling = body.crawling,
				.swimming = body.swimming,
				.swimAmount = body.swimAmount,
				.gliding = body.gliding,
				.riptideTicks = body.riptideTicks,
				.riptideCollision = body.riptideCollision
			};
		}

		void restore(Body& body) const {
			body.retainedBox = retainedBox;
			body.size = size;
			body.sneaking = sneaking;
			body.crawling = crawling;
			body.swimming = swimming;
			body.swimAmount = swimAmount;
			body.gliding = gliding;
			body.riptideTicks = riptideTicks;
			body.riptideCollision = riptideCollision;
		}
	};

	template <World W>
	class Poses {
	public:
		explicit Poses(const Collision<W>& collision)
			: mCollision(collision) { }

		[[nodiscard]] bool restoreUprightPose(Body& body, const bool collisionsAvailable) const {
			if (!collisionsAvailable) {
				return false;
			}

			const auto standing = mCollision.canFitHeight(body, body.standingHeight);
			if (!standing.known) {
				return false;
			}

			if (standing.fits) {
				body.sneaking = false;
				body.crawling = false;
				body.size.y = body.standingHeight;
				return true;
			}

			const auto sneaking = mCollision.canFitHeight(body, body.sneakingHeight);
			if (!sneaking.known) {
				return false;
			}

			if (sneaking.fits) {
				body.sneaking = true;
				body.crawling = false;
				body.size.y = body.sneakingHeight;
				return true;
			}

			body.sneaking = false;
			body.crawling = true;
			body.size.y = body.crawlingHeight;
			return true;
		}

		[[nodiscard]] bool restoreUprightPose(Body& body) const {
			return restoreUprightPose(body, mCollision.poseCollisionsAvailable(body));
		}

		[[nodiscard]] bool stopGliding(Body& body) const {
			body.gliding = false;
			if (body.swimPose() || body.riptideTicks > 0) {
				return true;
			}

			return restoreUprightPose(body);
		}

		[[nodiscard]] bool stopRiptideOnBlockCollision(Body& body) const {
			if (body.riptideTicks <= 0 || (!body.collideX && !body.collideZ)) {
				return true;
			}

			body.riptideTicks = 0;
			body.riptideCollision = false;
			if (body.gliding || body.swimPose()) {
				return true;
			}

			return restoreUprightPose(body);
		}

	private:
		const Collision<W>& mCollision;
	};
}
