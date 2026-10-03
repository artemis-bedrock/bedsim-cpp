#pragma once

#include "bedsim/constants.h"
#include "bedsim/vec.h"

#include <cstdint>
#include <utility>

namespace bedsim {
	enum class Face : uint8_t {
		Down,
		Up,
		North,
		South,
		West,
		East
	};

	enum class Effect : uint8_t {
		JumpBoost,
		Levitation,
		SlowFalling,
		Weaving
	};

	enum class Enchantment : uint8_t {
		DepthStrider,
		SoulSpeed,
		SwiftSneak,
		Riptide
	};

	enum class GameMode : int32_t {
		Survival = 0,
		Creative = 1,
		Adventure = 2,
		SurvivalSpectator = 3,
		CreativeSpectator = 4,
		Default = 5,
		Spectator = 6
	};

	enum class LiquidKind : uint8_t {
		Water,
		Lava
	};

	struct Liquid {
		LiquidKind kind{};
		int depth{ 8 };
		bool falling{};
	};

	enum class BubbleDirection : uint8_t {
		Up,
		Down
	};

	[[nodiscard]] constexpr BlockPos faceOffset(const Face face) {
		switch (face) {
		case Face::Down:
			return { 0, -1, 0 };
		case Face::Up:
			return { 0, 1, 0 };
		case Face::North:
			return { 0, 0, -1 };
		case Face::South:
			return { 0, 0, 1 };
		case Face::West:
			return { -1, 0, 0 };
		case Face::East:
			return { 1, 0, 0 };
		}

		std::unreachable();
	}

	[[nodiscard]] constexpr Face oppositeFace(const Face face) {
		return static_cast<Face>(std::to_underlying(face) ^ 1);
	}

	[[nodiscard]] constexpr Face faceTowards(const BlockPos& from, const BlockPos& to) {
		const auto delta = to - from;
		if (delta.y < 0) {
			return Face::Down;
		}

		if (delta.y > 0) {
			return Face::Up;
		}

		if (delta.z < 0) {
			return Face::North;
		}

		if (delta.z > 0) {
			return Face::South;
		}

		return delta.x < 0 ? Face::West : Face::East;
	}

	[[nodiscard]] inline Vec3 clampToBlockRange(const Vec3& value) {
		return clamp(value, Vec3{ kMinimumBlockCoordinate }, Vec3{ kMaximumBlockCoordinate });
	}

	[[nodiscard]] inline BlockPos blockAt(const Vec3& position) {
		return BlockPos{ clampToBlockRange(floor(position)) };
	}

	[[nodiscard]] inline BlockPos blockCeiling(const Vec3& position) {
		return BlockPos{ clampToBlockRange(ceil(position)) };
	}
}
