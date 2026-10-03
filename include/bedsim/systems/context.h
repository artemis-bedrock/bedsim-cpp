#pragma once

#include "bedsim/aabb.h"
#include "bedsim/body.h"
#include "bedsim/format.h"
#include "bedsim/options.h"
#include "bedsim/physics.h"
#include "bedsim/scratch.h"
#include "bedsim/semantics.h"
#include "bedsim/world.h"

#include <format>
#include <optional>
#include <utility>
#include <vector>

namespace bedsim::systems {
	template <World W>
	class Context {
	public:
		Context(const W& world, const Options& options, Scratch& scratch)
			: mWorld(world), mOptions(options), mScratch(scratch) { }

		[[nodiscard]] const W& world() const {
			return mWorld;
		}

		[[nodiscard]] const Options& options() const {
			return mOptions;
		}

		[[nodiscard]] Scratch& scratch() const {
			return mScratch;
		}

		template <typename... Args>
		void debug(std::format_string<Args...> format, Args&&... args) const {
			if constexpr (HasDebugSink<W>) {
				mWorld.debug(std::format(format, std::forward<Args>(args)...));
			}
		}

		[[nodiscard]] AABB boundingBox(const Body& body) const {
			return body.boundingBox(mOptions.useSlideOffset);
		}

		[[nodiscard]] BlockMovement movement(const BlockPos& position) const {
			const BlockMovement movement{ mWorld.movement(position) };
			if constexpr (HasMovementFallback<W>) {
				return movement.validated(mWorld.fallbackMovement(position));
			} else {
				return movement.validated();
			}
		}

		[[nodiscard]] bool isAir(const BlockPos& position) const {
			return movement(position).air;
		}

		void blockCollisions(const BlockPos& position, std::vector<AABB>& out) const {
			mWorld.blockCollisions(position, out);
		}

		[[nodiscard]] bool areaLoaded(const AABB& area) const {
			return isMovementRangeValid(area) && mWorld.isLoaded(area);
		}

		[[nodiscard]] std::optional<Liquid> liquid(const BlockPos& position) const {
			if constexpr (HasLiquids<W>) {
				return mWorld.liquid(position);
			} else {
				return std::nullopt;
			}
		}

		[[nodiscard]] std::optional<int> effectAmplifier(const Effect effect) const {
			if constexpr (HasEffects<W>) {
				return mWorld.effectAmplifier(effect);
			} else {
				return std::nullopt;
			}
		}

		[[nodiscard]] static constexpr bool hasEquipment() {
			return HasEquipment<W>;
		}

		[[nodiscard]] int enchantmentLevel(const Enchantment enchantment) const {
			if constexpr (HasEquipment<W>) {
				return mWorld.enchantmentLevel(enchantment);
			} else {
				return 0;
			}
		}

		[[nodiscard]] bool wearingLeatherBoots() const {
			if constexpr (HasEquipment<W>) {
				return mWorld.wearingLeatherBoots();
			} else {
				return false;
			}
		}

		[[nodiscard]] bool hasElytra() const {
			if constexpr (HasElytra<W>) {
				return mWorld.hasElytra();
			} else {
				return false;
			}
		}

	private:
		const W& mWorld;
		const Options& mOptions;
		Scratch& mScratch;
	};
}
