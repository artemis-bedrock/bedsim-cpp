#pragma once

#include "bedsim/aabb.h"
#include "bedsim/semantics.h"
#include "bedsim/types.h"
#include "bedsim/vec.h"

#include <concepts>
#include <optional>
#include <string_view>
#include <vector>

namespace bedsim {
	struct CollisionContext {
		Vec3 position{};
		bool sneaking{};
		bool descending{};
		bool wantDown{};
		bool leatherBoots{};
	};

	template <typename T>
	concept World = requires(const T& world, const BlockPos& position, const AABB& area, std::vector<AABB>& out) {
		world.blockCollisions(position, out);
		{ world.movement(position) } -> std::convertible_to<BlockMovement>;
		{ world.isLoaded(area) } -> std::same_as<bool>;
	};

	template <typename T>
	concept HasMovementFallback = requires(const T& world, const BlockPos& position) {
		{ world.fallbackMovement(position) } -> std::convertible_to<BlockMovement>;
	};

	template <typename T>
	concept HasCollisionQuery = requires(const T& world, const AABB& area, std::vector<AABB>& out) {
		world.collisions(area, out);
	};

	template <typename T>
	concept HasMovementCollisions = requires(const T& world, const AABB& area, const CollisionContext& context, std::vector<AABB>& out) {
		world.movementCollisions(area, context, out);
	};

	template <typename T>
	concept HasSupportQuery = requires(const T& world, const AABB& area, const CollisionContext& context) {
		{ world.supportingBlock(area, context) } -> std::same_as<std::optional<BlockPos>>;
	};

	template <typename T>
	concept HasClimbableContact = requires(const T& world, const AABB& box) {
		{ world.climbableContact(box) } -> std::same_as<bool>;
	};

	template <typename T>
	concept HasLiquids = requires(const T& world, const BlockPos& position) {
		{ world.liquid(position) } -> std::same_as<std::optional<Liquid>>;
	};

	template <typename T>
	concept HasLiquidFlow = requires(const T& world, const BlockPos& position, Face face) {
		{ world.liquidFaceClosed(position, face) } -> std::same_as<bool>;
		{ world.liquidFlowBarrier(position) } -> std::same_as<bool>;
	};

	template <typename T>
	concept HasBubbleColumns = requires(const T& world, const BlockPos& position) {
		{ world.bubbleColumn(position) } -> std::same_as<std::optional<BubbleDirection>>;
	};

	template <typename T>
	concept HasBubbleColumnSurface = requires(const T& world, const BlockPos& position) {
		{ world.bubbleColumnSurface(position) } -> std::same_as<std::optional<bool>>;
	};

	template <typename T>
	concept HasEffects = requires(const T& world, Effect effect) {
		{ world.effectAmplifier(effect) } -> std::same_as<std::optional<int>>;
	};

	template <typename T>
	concept HasEquipment = requires(const T& world, Enchantment enchantment) {
		{ world.enchantmentLevel(enchantment) } -> std::same_as<int>;
		{ world.wearingLeatherBoots() } -> std::same_as<bool>;
	};

	template <typename T>
	concept HasDebugSink = requires(const T& world, std::string_view message) {
		world.debug(message);
	};

	template <typename T>
	concept HasElytra = requires(const T& world) {
		{ world.hasElytra() } -> std::same_as<bool>;
	};
}
