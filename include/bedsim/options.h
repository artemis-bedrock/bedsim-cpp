#pragma once

#include "bedsim/constants.h"

#include <cstdint>

namespace bedsim {
	enum class SimulationMode : uint8_t {
		Authoritative,
		Permissive,
		Passive
	};

	enum class SprintTiming : uint8_t {
		Modern,
		Legacy
	};

	struct Options {
		SimulationMode mode{};

		float positionCorrectionThreshold{};
		float velocityCorrectionThreshold{};

		bool useSlideOffset{};
		SprintTiming sprintTiming{};

		bool limitAllVelocity{};
		float limitAllVelocityThreshold{};

		bool ignoreClientStepTiebreaker{};
		bool requireLiquidLayer{};
		int64_t swimWaterGraceTicks{};
		bool upstreamImpulseClamping{};

		[[nodiscard]] constexpr int64_t resolvedSwimWaterGraceTicks() const {
			if (swimWaterGraceTicks < 0) {
				return 0;
			}

			return swimWaterGraceTicks == 0 ? kSwimWaterGraceTicks : swimWaterGraceTicks;
		}
	};

	[[nodiscard]] constexpr Options predictionOptions() {
		return {
			.mode = SimulationMode::Passive,
			.ignoreClientStepTiebreaker = true
		};
	}
}
