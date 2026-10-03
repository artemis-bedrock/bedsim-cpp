#pragma once

#include "bedsim/body.h"
#include "bedsim/constants.h"
#include "bedsim/physics.h"
#include "bedsim/systems/context.h"
#include "bedsim/vec.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace bedsim::systems {
	struct Fit {
		bool fits{};
		bool known{};
	};

	template <World W>
	class Collision {
	public:
		explicit Collision(const Context<W>& context)
			: mContext(context) { }

		[[nodiscard]] CollisionContext collisionContext(const Body& body) const {
			return {
				.position = body.position,
				.sneaking = body.sneaking,
				.descending = body.pressingDescend,
				.wantDown = body.wantDown,
				.leatherBoots = mContext.wearingLeatherBoots()
			};
		}

		void nearbyBoxes(const Body& body, const AABB& area, std::vector<AABB>& out) const {
			out.clear();
			if constexpr (HasMovementCollisions<W>) {
				mContext.world().movementCollisions(area, collisionContext(body), out);
			} else if constexpr (HasCollisionQuery<W>) {
				mContext.world().collisions(area, out);
			} else {
				gatherCollisions(area, out);
			}

			std::erase_if(out, [](const AABB& box) {
				return box.hasZeroVolume();
			});
		}

		[[nodiscard]] bool hasNearbyBoxes(const Body& body, const AABB& area) const {
			auto& probe = mContext.scratch().probe;
			nearbyBoxes(body, area, probe);
			return !probe.empty();
		}

		[[nodiscard]] bool poseCollisionsAvailable(const Body& body) const {
			return mContext.areaLoaded(mContext.boundingBox(body));
		}

		[[nodiscard]] bool movementSweepLoaded(const Body& body) const {
			const auto sweep = mContext.boundingBox(body).extend(body.velocity);
			return mContext.areaLoaded(movementProbeArea(sweep));
		}

		[[nodiscard]] Fit canFitHeight(const Body& body, const float height) const {
			Body standing = body;
			standing.size.y = height;
			standing.sneaking = false;
			standing.swimming = false;
			standing.gliding = false;
			standing.riptideTicks = 0;
			standing.swimWaterContact = false;
			standing.swimWaterGraceTicks = 0;
			standing.pressingDescend = false;
			standing.wantDown = false;
			const auto box = mContext.boundingBox(standing);
			if (!mContext.areaLoaded(box)) {
				return { .fits = false, .known = false };
			}

			return { .fits = !hasNearbyBoxes(standing, box), .known = true };
		}

		void prepareCollisionBox(Body& body) const {
			const bool useSlideOffset = mContext.options().useSlideOffset;
			const auto dimensions = body.collisionDimensions();
			const float offset = body.slideYOffset(useSlideOffset);
			const auto& retained = body.retainedBox;
			const bool sameDimensions = retained && retained->dimensions == dimensions && retained->yOffset == offset;
			if (sameDimensions && retained->position == body.position) {
				return;
			}

			auto box = body.collisionBoxAt(body.position, useSlideOffset);
			if (retained && !sameDimensions) {
				body.rememberCollisionBox(box, useSlideOffset);
				return;
			}

			if (!mContext.areaLoaded(box)) {
				return;
			}

			auto& probe = mContext.scratch().probe;
			nearbyBoxes(body, box, probe);
			box = recoverRoundedContacts(box, body.position, probe);
			body.rememberCollisionBox(box, useSlideOffset);
		}

		[[nodiscard]] bool tryCollisions(Body& body) const {
			prepareCollisionBox(body);
			const auto& options = mContext.options();
			const bool useSlideOffset = options.useSlideOffset;

			bool completedStep = false;
			const auto startBox = body.boundingBox(useSlideOffset);
			const auto velocity = body.velocity;
			auto& boxes = mContext.scratch().boxes;
			nearbyBoxes(body, startBox.extend(velocity), boxes);

			const bool oneWay = body.stuckInCollider;
			const auto y = clipAll(boxes, startBox, { 0.0f, velocity.y, 0.0f }, oneWay);
			auto box = startBox.translate(y.velocity);
			mContext.debug("Y-collision non-step={} /w penetration={} (oneWay={})", y.velocity, y.penetration, oneWay);

			const auto x = clipAll(boxes, box, { velocity.x, 0.0f, 0.0f }, oneWay, y.penetration);
			box = box.translate(x.velocity);
			mContext.debug("(X) hz-collision non-step={} /w penetration={} (oneWay={})", x.velocity, x.penetration, oneWay);

			const auto z = clipAll(boxes, box, { 0.0f, 0.0f, velocity.z }, oneWay, x.penetration);
			box = box.translate(z.velocity);
			mContext.debug("(Z) hz-collision non-step={} /w penetration={} (oneWay={})", z.velocity, z.penetration, oneWay);

			auto collided = y.velocity + x.velocity + z.velocity;
			const auto& penetration = z.penetration;
			const auto collisionPosition = box.bottomCenter();
			mContext.debug("endCollisionVel={} endCollisionPos={}", collided, collisionPosition);

			const bool penetrated = dot(penetration, penetration) >= 9.999999999999999e-12f;
			body.stuckInCollider = body.penetratedLastFrame && penetrated;
			body.penetratedLastFrame = penetrated;

			const bool blockedX = velocity.x != collided.x;
			const bool blockedY = velocity.y != collided.y;
			const bool blockedZ = velocity.z != collided.z;
			const bool grounded = body.onGround || (blockedY && velocity.y < 0.0f);
			if (grounded && (blockedX || blockedZ)) {
				const auto stepProbe = startBox.extend(velocity).extendTowards(Face::Up, kStepHeight);
				if (!mContext.areaLoaded(stepProbe)) {
					return false;
				}

				const auto step = autoStep(startBox, velocity, boxes, oneWay);
				mContext.debug("auto-step collision boxes={}/{}", step.boxCount, boxes.size());
				mContext.debug("stepYVel={}", step.up);
				mContext.debug("stepXVel={}", step.x);
				mContext.debug("stepZVel={}", step.z);
				mContext.debug("inverseYStepVel={}", step.down);

				const bool stepBlocked = hasNearbyBoxes(body, step.box);
				const auto stepPosition = step.box.bottomCenter();
				mContext.debug("endStepVel={} endStepPos={}", step.velocity, stepPosition);

				if (!stepBlocked && horizontalLengthSquared(collided) < horizontalLengthSquared(step.velocity)) {
					const float stepDistance = length(stepPosition - body.client.position);
					const float collisionDistance = length(collisionPosition - body.client.position);
					if (options.ignoreClientStepTiebreaker || collisionDistance > options.positionCorrectionThreshold || stepDistance <= collisionDistance) {
						collided = step.velocity;
						box = step.box;
						completedStep = true;
						if (useSlideOffset) {
							auto slide = body.slideOffset * kSlideOffsetMultiplier;
							slide.y += step.velocity.y;
							body.slideOffset = slide;
						}

						mContext.debug("step successful");
					} else {
						mContext.debug("step failed (client rejection) [clientPos={} collisionPos={} stepPos={}]", body.client.position, collisionPosition, stepPosition);
					}
				} else {
					mContext.debug("step failed");
				}
			}

			auto end = box.bottomCenter();
			if (useSlideOffset) {
				if (completedStep) {
					end.y -= body.slideOffset.y;
				} else {
					body.slideOffset = {};
				}
			}

			body.sprintMovementBlocked = sprintMovementBlocked(velocity, end - body.position);
			body.setPosition(end);
			body.rememberCollisionBox(box, useSlideOffset);

			const bool collidedY = std::abs(velocity.y - collided.y) >= 1e-5f;
			body.collideX = std::abs(velocity.x - collided.x) >= 1e-5f;
			body.collideY = collidedY;
			body.collideZ = std::abs(velocity.z - collided.z) >= 1e-5f;
			body.onGround = (collidedY && velocity.y < 0.0f) || (body.onGround && !collidedY && velocity.y == 0.0f);
			if (!checkSupportingBlock(body, velocity)) {
				return false;
			}

			body.setVelocity(collided);
			mContext.debug("clientVel={} clientPos={}", body.client.movement, body.client.position);
			mContext.debug("finalVel={} finalPos={}", collided, body.position);
			mContext.debug("(client) hzCollision={} yCollision={}", body.client.horizontalCollision, body.client.verticalCollision);
			mContext.debug("(server) xCollision={} yCollision={} zCollision={}", body.collideX, body.collideY, body.collideZ);
			return true;
		}

		[[nodiscard]] bool avoidEdge(Body& body) const {
			if (!body.sneaking || !body.onGround || body.velocity.y > 0.0f) {
				mContext.debug("avoidEdge: conditions not met (sneaking={} onGround={} yVel={})", body.sneaking, body.onGround, body.velocity.y);
				return true;
			}

			constexpr float edgeBoundary = 0.025f;
			constexpr int maxIterations = 1000;
			const auto previous = body.velocity;
			const auto box = mContext.boundingBox(body).grow({ -edgeBoundary, 0.0f, -edgeBoundary });
			const float drop = -kStepHeight * 1.01f;
			float x = body.velocity.x;
			float z = body.velocity.z;
			if (!mContext.areaLoaded(box.extend({ x, drop, z }))) {
				return false;
			}

			int iteration = 0;
			for (; iteration < maxIterations && x != 0.0f && !hasNearbyBoxes(body, box.translate({ x, drop, 0.0f })); ++iteration) {
				x = stepTowardsZero(x);
			}

			if (iteration == maxIterations) {
				x = 0.0f;
			}

			for (iteration = 0; iteration < maxIterations && z != 0.0f && !hasNearbyBoxes(body, box.translate({ 0.0f, drop, z })); ++iteration) {
				z = stepTowardsZero(z);
			}

			if (iteration == maxIterations) {
				z = 0.0f;
			}

			for (iteration = 0; iteration < maxIterations && x != 0.0f && z != 0.0f && !hasNearbyBoxes(body, box.translate({ x, drop, z })); ++iteration) {
				x = stepTowardsZero(x);
				z = stepTowardsZero(z);
			}

			if (iteration == maxIterations) {
				x = 0.0f;
				z = 0.0f;
			}

			body.setVelocity({ x, body.velocity.y, z });
			mContext.debug("(avoidEdge): oldVel={} newVel={}", previous, body.velocity);
			return true;
		}

	private:
		[[nodiscard]] static float horizontalLengthSquared(const Vec3& vector) {
			return vector.x * vector.x + vector.z * vector.z;
		}

		[[nodiscard]] static float stepTowardsZero(const float value) {
			constexpr float offset = 0.05f;
			if (value < offset && value >= -offset) {
				return 0.0f;
			}

			return value > 0.0f ? value - offset : value + offset;
		}

		void gatherCollisions(const AABB& area, std::vector<AABB>& out) const {
			auto& local = mContext.scratch().blockBoxes;
			const auto minimum = blockAt(area.min);
			const auto maximum = blockAt(area.max);
			for (int x = minimum.x; x <= maximum.x; ++x) {
				for (int y = minimum.y - 1; y <= maximum.y; ++y) {
					for (int z = minimum.z; z <= maximum.z; ++z) {
						const BlockPos position{ x, y, z };
						local.clear();
						mContext.blockCollisions(position, local);
						for (const auto& shape : local) {
							const auto placed = shape.translate(Vec3{ position });
							if (placed.intersects(area)) {
								out.push_back(placed);
							}
						}
					}
				}
			}
		}

		[[nodiscard]] bool checkSupportingBlock(Body& body, const Vec3& velocity) const {
			if (!body.onGround) {
				body.supportingBlock.reset();
				return true;
			}

			auto area = mContext.boundingBox(body).extendTowards(Face::Down, 1e-3f);
			if (!mContext.areaLoaded(area)) {
				body.supportingBlock.reset();
				return false;
			}

			findSupportingBlock(body, area);
			if (body.supportingBlock) {
				return true;
			}

			area = area.translate({ -velocity.x, 0.0f, -velocity.z });
			if (!mContext.areaLoaded(area)) {
				return false;
			}

			findSupportingBlock(body, area);
			return true;
		}

		void findSupportingBlock(Body& body, const AABB& area) const {
			if constexpr (HasSupportQuery<W>) {
				body.supportingBlock = mContext.world().supportingBlock(area, collisionContext(body));
			} else {
				body.supportingBlock.reset();
				auto& local = mContext.scratch().blockBoxes;
				const auto center = Vec3{ blockAt(body.position) } + Vec3{ 0.5f };
				float closest = std::numeric_limits<float>::max() - 1.0f;
				const auto minimum = blockAt(area.min);
				const auto maximum = blockCeiling(area.max);
				for (int y = minimum.y; y <= maximum.y; ++y) {
					for (int x = minimum.x; x <= maximum.x; ++x) {
						for (int z = minimum.z; z <= maximum.z; ++z) {
							const BlockPos position{ x, y, z };
							local.clear();
							mContext.blockCollisions(position, local);
							for (const auto& shape : local) {
								if (shape.hasZeroVolume() || !area.intersects(shape.translate(Vec3{ position }))) {
									continue;
								}

								const auto offset = Vec3{ position } - center;
								if (const float distance = dot(offset, offset); distance < closest) {
									closest = distance;
									body.supportingBlock = position;
								}

								break;
							}
						}
					}
				}
			}
		}

		const Context<W>& mContext;
	};
}
