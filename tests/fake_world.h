#pragma once

#include "bedsim/simulator.h"

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace bedsim::test {
	struct BlockPosLess {
		[[nodiscard]] bool operator()(const BlockPos& left, const BlockPos& right) const {
			return std::tie(left.x, left.y, left.z) < std::tie(right.x, right.y, right.z);
		}
	};

	inline const std::vector<AABB> kFullBlock{ AABB{ {}, Vec3{ 1.0f } } };

	struct FakeBlock {
		std::string name{ "minecraft:air" };
		std::vector<AABB> shapes;
		std::optional<Liquid> liquid;
		std::optional<BubbleDirection> bubble;
	};

	struct FakeWorld {
		std::map<BlockPos, FakeBlock, BlockPosLess> blocks;
		std::string fill{ "minecraft:air" };
		std::map<Effect, int> effects;
		std::map<Enchantment, int> enchantments;
		bool leatherBoots{};
		bool elytra{};
		std::function<bool(const AABB&)> loaded;

		FakeWorld& set(const BlockPos& position, FakeBlock block) {
			blocks[position] = std::move(block);
			return *this;
		}

		FakeWorld& solid(const BlockPos& position, const std::string& name = "minecraft:stone") {
			return set(position, { .name = name, .shapes = kFullBlock });
		}

		FakeWorld& passable(const BlockPos& position, const std::string& name) {
			return set(position, { .name = name });
		}

		FakeWorld& water(const BlockPos& position, const int depth = 8, const bool falling = false) {
			return set(position, { .name = "minecraft:water", .liquid = Liquid{ .kind = LiquidKind::Water, .depth = depth, .falling = falling } });
		}

		FakeWorld& lava(const BlockPos& position, const int depth = 8, const bool falling = false) {
			return set(position, { .name = "minecraft:lava", .liquid = Liquid{ .kind = LiquidKind::Lava, .depth = depth, .falling = falling } });
		}

		FakeWorld& floor(const int y, const int radius, const std::string& name = "minecraft:stone") {
			for (int x = -radius; x <= radius; ++x) {
				for (int z = -radius; z <= radius; ++z) {
					solid({ x, y, z }, name);
				}
			}

			return *this;
		}

		[[nodiscard]] const FakeBlock* find(const BlockPos& position) const {
			const auto block = blocks.find(position);
			return block != blocks.end() ? &block->second : nullptr;
		}

		void blockCollisions(const BlockPos& position, std::vector<AABB>& out) const {
			if (const auto* block = find(position)) {
				out.insert(out.end(), block->shapes.begin(), block->shapes.end());
			}
		}

		[[nodiscard]] BlockMovement movement(const BlockPos& position) const {
			const auto* block = find(position);
			return vanillaMovement(block ? block->name : fill);
		}

		[[nodiscard]] bool isLoaded(const AABB& area) const {
			return !loaded || loaded(area);
		}

		[[nodiscard]] std::optional<Liquid> liquid(const BlockPos& position) const {
			const auto* block = find(position);
			return block ? block->liquid : std::nullopt;
		}

		[[nodiscard]] std::optional<BubbleDirection> bubbleColumn(const BlockPos& position) const {
			const auto* block = find(position);
			return block ? block->bubble : std::nullopt;
		}

		[[nodiscard]] std::optional<int> effectAmplifier(const Effect effect) const {
			const auto found = effects.find(effect);
			return found != effects.end() ? std::optional{ found->second } : std::nullopt;
		}

		[[nodiscard]] int enchantmentLevel(const Enchantment enchantment) const {
			const auto found = enchantments.find(enchantment);
			return found != enchantments.end() ? found->second : 0;
		}

		[[nodiscard]] bool wearingLeatherBoots() const {
			return leatherBoots;
		}

		[[nodiscard]] bool hasElytra() const {
			return elytra;
		}
	};

	struct BoxWorld {
		std::vector<AABB> boxes;
		bool loaded{ true };

		void blockCollisions(const BlockPos&, std::vector<AABB>&) const { }

		void collisions(const AABB& area, std::vector<AABB>& out) const {
			for (const auto& box : boxes) {
				if (area.intersects(box)) {
					out.push_back(box);
				}
			}
		}

		[[nodiscard]] BlockMovement movement(const BlockPos&) const {
			return vanillaMovement("minecraft:air");
		}

		[[nodiscard]] bool isLoaded(const AABB&) const {
			return loaded;
		}
	};

	template <typename Base>
	struct LoggedWorld : Base {
		std::vector<std::string>* logs{};

		void debug(const std::string_view line) const {
			logs->emplace_back(line);
		}
	};

	[[nodiscard]] inline Body baseBody() {
		return Body{};
	}

	static_assert(World<FakeWorld> && HasLiquids<FakeWorld> && HasEffects<FakeWorld> && HasEquipment<FakeWorld> && HasElytra<FakeWorld> && HasBubbleColumns<FakeWorld>);
	static_assert(!HasCollisionQuery<FakeWorld> && !HasMovementCollisions<FakeWorld> && !HasSupportQuery<FakeWorld> && !HasLiquidFlow<FakeWorld>);
	static_assert(World<BoxWorld> && HasCollisionQuery<BoxWorld> && !HasEffects<BoxWorld> && !HasLiquids<BoxWorld>);
	static_assert(!HasDebugSink<FakeWorld> && HasDebugSink<LoggedWorld<FakeWorld>>);
}
