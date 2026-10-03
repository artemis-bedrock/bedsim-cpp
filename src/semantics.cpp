#include "bedsim/semantics.h"

#include <algorithm>
#include <array>
#include <string_view>

namespace bedsim {
	struct FrictionRule {
		std::string_view name;
		float friction{};
	};

	static constexpr std::array kFrictionRules{
		FrictionRule{ "minecraft:ice", 0.98f },
		FrictionRule{ "minecraft:frosted_ice", 0.98f },
		FrictionRule{ "minecraft:packed_ice", 0.98f },
		FrictionRule{ "minecraft:blue_ice", 0.989f },
		FrictionRule{ "minecraft:slime", 0.8f },
		FrictionRule{ "minecraft:honey_block", 0.8f }
	};

	static constexpr std::array<std::string_view, 7> kClimbable{
		"minecraft:ladder",
		"minecraft:vine",
		"minecraft:cave_vines",
		"minecraft:cave_vines_body_with_berries",
		"minecraft:cave_vines_head_with_berries",
		"minecraft:twisting_vines",
		"minecraft:weeping_vines"
	};

	[[nodiscard]] static bool isFenceLike(const std::string_view name) {
		return name.ends_with("_fence") || name.ends_with("fence_gate") || name.ends_with("_wall");
	}

	BlockMovement vanillaMovement(const std::string_view name) {
		BlockMovement movement{
			.air = name == "minecraft:air",
			.climbable = std::ranges::contains(kClimbable, name),
			.cobweb = name == "minecraft:web",
			.honey = name == "minecraft:honey_block",
			.fenceLike = isFenceLike(name)
		};

		if (const auto rule = std::ranges::find(kFrictionRules, name, &FrictionRule::name); rule != kFrictionRules.end()) {
			movement.groundFriction = rule->friction;
		}

		if (name == "minecraft:soul_sand") {
			movement.accelerationFriction = kSoulSandAccelerationFriction;
			movement.soulSpeedNeutralizesFriction = true;
		}

		if (name == "minecraft:slime") {
			movement.bounce = Bounce::Slime;
		} else if (name == "minecraft:bed") {
			movement.bounce = Bounce::Bed;
		} else if (name == "minecraft:sweet_berry_bush") {
			movement.inside = Inside::SweetBerryBush;
		} else if (name == "minecraft:powder_snow") {
			movement.inside = Inside::PowderSnow;
			movement.traversal = Traversal::PowderSnow;
		} else if (name == "minecraft:scaffolding") {
			movement.traversal = Traversal::Scaffolding;
		}

		return movement;
	}
}
