#pragma once

#include "bedsim/aabb.h"
#include "bedsim/vec.h"

#include <format>

template <>
struct std::formatter<bedsim::Vec2> {
	constexpr auto parse(std::format_parse_context& context) {
		return context.begin();
	}

	auto format(const bedsim::Vec2& value, std::format_context& context) const {
		return std::format_to(context.out(), "[{} {}]", value.x, value.y);
	}
};

template <typename T>
struct std::formatter<bedsim::Vector3<T>> {
	constexpr auto parse(std::format_parse_context& context) {
		return context.begin();
	}

	auto format(const bedsim::Vector3<T>& value, std::format_context& context) const {
		return std::format_to(context.out(), "[{} {} {}]", value.x, value.y, value.z);
	}
};

template <>
struct std::formatter<bedsim::AABB> {
	constexpr auto parse(std::format_parse_context& context) {
		return context.begin();
	}

	auto format(const bedsim::AABB& value, std::format_context& context) const {
		return std::format_to(context.out(), "{{{} {}}}", value.min, value.max);
	}
};
