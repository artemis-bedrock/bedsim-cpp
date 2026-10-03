#pragma once

#include "bedsim/types.h"
#include "bedsim/vec.h"

#include <cmath>

namespace bedsim {
	inline constexpr float kIntersectionEpsilon = 1e-5f;

	struct AABB {
		Vec3 min{};
		Vec3 max{};

		[[nodiscard]] bool operator==(const AABB&) const = default;

		[[nodiscard]] static AABB unitAt(const BlockPos& position) {
			const Vec3 minimum{ position };
			return { minimum, minimum + Vec3{ 1.0f } };
		}

		[[nodiscard]] static AABB ordered(const Vec3& first, const Vec3& second) {
			return { bedsim::min(first, second), bedsim::max(first, second) };
		}

		[[nodiscard]] AABB translate(const Vec3& offset) const {
			return { min + offset, max + offset };
		}

		[[nodiscard]] AABB grow(const Vec3& amount) const {
			return { min - amount, max + amount };
		}

		[[nodiscard]] AABB grow(const float amount) const {
			return grow(Vec3{ amount });
		}

		[[nodiscard]] AABB extend(const Vec3& movement) const {
			AABB result = *this;
			for (int axis = 0; axis < 3; ++axis) {
				if (movement[axis] < 0.0f) {
					result.min[axis] += movement[axis];
				} else if (movement[axis] > 0.0f) {
					result.max[axis] += movement[axis];
				}
			}

			return result;
		}

		[[nodiscard]] AABB extendTowards(const Face face, const float amount) const {
			AABB result = *this;
			switch (face) {
			case Face::Down:
				result.min.y -= amount;
				break;
			case Face::Up:
				result.max.y += amount;
				break;
			case Face::North:
				result.min.z -= amount;
				break;
			case Face::South:
				result.max.z += amount;
				break;
			case Face::West:
				result.min.x -= amount;
				break;
			case Face::East:
				result.max.x += amount;
				break;
			}

			return result;
		}

		[[nodiscard]] bool intersects(const AABB& other, const float epsilon = kIntersectionEpsilon) const {
			return other.max.x - min.x > epsilon && max.x - other.min.x > epsilon &&
				other.max.y - min.y > epsilon && max.y - other.min.y > epsilon &&
				other.max.z - min.z > epsilon && max.z - other.min.z > epsilon;
		}

		[[nodiscard]] bool hasZeroVolume() const {
			for (int axis = 0; axis < 3; ++axis) {
				if (!std::isfinite(min[axis]) || !std::isfinite(max[axis]) || min[axis] >= max[axis]) {
					return true;
				}
			}

			return false;
		}

		[[nodiscard]] Vec3 bottomCenter() const {
			return { (min.x + max.x) * 0.5f, min.y, (min.z + max.z) * 0.5f };
		}
	};
}
