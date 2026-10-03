#pragma once

#include <algorithm>
#include <cmath>

namespace bedsim {
	struct Vec2 {
		float x{};
		float y{};

		constexpr Vec2() = default;

		constexpr explicit Vec2(const float value)
			: x(value), y(value) { }

		constexpr Vec2(const float x, const float y)
			: x(x), y(y) { }

		[[nodiscard]] constexpr bool operator==(const Vec2&) const = default;

		[[nodiscard]] constexpr float& operator[](const int index) {
			return index == 0 ? x : y;
		}

		[[nodiscard]] constexpr float operator[](const int index) const {
			return index == 0 ? x : y;
		}

		constexpr Vec2& operator+=(const Vec2& other) {
			x += other.x;
			y += other.y;
			return *this;
		}

		constexpr Vec2& operator-=(const Vec2& other) {
			x -= other.x;
			y -= other.y;
			return *this;
		}

		constexpr Vec2& operator*=(const float scalar) {
			x *= scalar;
			y *= scalar;
			return *this;
		}

		constexpr Vec2& operator/=(const float scalar) {
			x /= scalar;
			y /= scalar;
			return *this;
		}
	};

	template <typename T>
	struct Vector3 {
		T x{};
		T y{};
		T z{};

		constexpr Vector3() = default;

		constexpr explicit Vector3(const T value)
			: x(value), y(value), z(value) { }

		constexpr Vector3(const T x, const T y, const T z)
			: x(x), y(y), z(z) { }

		template <typename U>
		constexpr explicit Vector3(const Vector3<U>& other)
			: x(static_cast<T>(other.x)), y(static_cast<T>(other.y)), z(static_cast<T>(other.z)) { }

		[[nodiscard]] constexpr bool operator==(const Vector3&) const = default;

		[[nodiscard]] constexpr T& operator[](const int index) {
			if (index == 0) {
				return x;
			}

			return index == 1 ? y : z;
		}

		[[nodiscard]] constexpr T operator[](const int index) const {
			if (index == 0) {
				return x;
			}

			return index == 1 ? y : z;
		}

		constexpr Vector3& operator+=(const Vector3& other) {
			x += other.x;
			y += other.y;
			z += other.z;
			return *this;
		}

		constexpr Vector3& operator-=(const Vector3& other) {
			x -= other.x;
			y -= other.y;
			z -= other.z;
			return *this;
		}

		constexpr Vector3& operator*=(const Vector3& other) {
			x *= other.x;
			y *= other.y;
			z *= other.z;
			return *this;
		}

		constexpr Vector3& operator*=(const T scalar) {
			x *= scalar;
			y *= scalar;
			z *= scalar;
			return *this;
		}

		constexpr Vector3& operator/=(const T scalar) {
			x /= scalar;
			y /= scalar;
			z /= scalar;
			return *this;
		}
	};

	using Vec3 = Vector3<float>;
	using BlockPos = Vector3<int>;

	[[nodiscard]] constexpr Vec2 operator-(const Vec2& value) {
		return { -value.x, -value.y };
	}

	[[nodiscard]] constexpr Vec2 operator+(const Vec2& first, const Vec2& second) {
		return { first.x + second.x, first.y + second.y };
	}

	[[nodiscard]] constexpr Vec2 operator-(const Vec2& first, const Vec2& second) {
		return { first.x - second.x, first.y - second.y };
	}

	[[nodiscard]] constexpr Vec2 operator*(const Vec2& first, const Vec2& second) {
		return { first.x * second.x, first.y * second.y };
	}

	[[nodiscard]] constexpr Vec2 operator*(const Vec2& value, const float scalar) {
		return { value.x * scalar, value.y * scalar };
	}

	[[nodiscard]] constexpr Vec2 operator*(const float scalar, const Vec2& value) {
		return { scalar * value.x, scalar * value.y };
	}

	[[nodiscard]] constexpr Vec2 operator/(const Vec2& value, const float scalar) {
		return { value.x / scalar, value.y / scalar };
	}

	template <typename T>
	[[nodiscard]] constexpr Vector3<T> operator-(const Vector3<T>& value) {
		return { -value.x, -value.y, -value.z };
	}

	template <typename T>
	[[nodiscard]] constexpr Vector3<T> operator+(const Vector3<T>& first, const Vector3<T>& second) {
		return { first.x + second.x, first.y + second.y, first.z + second.z };
	}

	template <typename T>
	[[nodiscard]] constexpr Vector3<T> operator-(const Vector3<T>& first, const Vector3<T>& second) {
		return { first.x - second.x, first.y - second.y, first.z - second.z };
	}

	template <typename T>
	[[nodiscard]] constexpr Vector3<T> operator*(const Vector3<T>& first, const Vector3<T>& second) {
		return { first.x * second.x, first.y * second.y, first.z * second.z };
	}

	template <typename T>
	[[nodiscard]] constexpr Vector3<T> operator*(const Vector3<T>& value, const T scalar) {
		return { value.x * scalar, value.y * scalar, value.z * scalar };
	}

	template <typename T>
	[[nodiscard]] constexpr Vector3<T> operator*(const T scalar, const Vector3<T>& value) {
		return { scalar * value.x, scalar * value.y, scalar * value.z };
	}

	template <typename T>
	[[nodiscard]] constexpr Vector3<T> operator/(const Vector3<T>& first, const Vector3<T>& second) {
		return { first.x / second.x, first.y / second.y, first.z / second.z };
	}

	template <typename T>
	[[nodiscard]] constexpr Vector3<T> operator/(const Vector3<T>& value, const T scalar) {
		return { value.x / scalar, value.y / scalar, value.z / scalar };
	}

	[[nodiscard]] constexpr float dot(const Vec2& first, const Vec2& second) {
		return first.x * second.x + first.y * second.y;
	}

	[[nodiscard]] constexpr float dot(const Vec3& first, const Vec3& second) {
		return first.x * second.x + first.y * second.y + first.z * second.z;
	}

	[[nodiscard]] inline float length(const Vec2& value) {
		return std::sqrt(dot(value, value));
	}

	[[nodiscard]] inline float length(const Vec3& value) {
		return std::sqrt(dot(value, value));
	}

	[[nodiscard]] constexpr Vec2 min(const Vec2& first, const Vec2& second) {
		return { std::min(first.x, second.x), std::min(first.y, second.y) };
	}

	[[nodiscard]] constexpr Vec3 min(const Vec3& first, const Vec3& second) {
		return { std::min(first.x, second.x), std::min(first.y, second.y), std::min(first.z, second.z) };
	}

	[[nodiscard]] constexpr Vec2 max(const Vec2& first, const Vec2& second) {
		return { std::max(first.x, second.x), std::max(first.y, second.y) };
	}

	[[nodiscard]] constexpr Vec3 max(const Vec3& first, const Vec3& second) {
		return { std::max(first.x, second.x), std::max(first.y, second.y), std::max(first.z, second.z) };
	}

	[[nodiscard]] constexpr Vec2 clamp(const Vec2& value, const Vec2& minimum, const Vec2& maximum) {
		return min(max(value, minimum), maximum);
	}

	[[nodiscard]] constexpr Vec3 clamp(const Vec3& value, const Vec3& minimum, const Vec3& maximum) {
		return min(max(value, minimum), maximum);
	}

	[[nodiscard]] inline Vec3 floor(const Vec3& value) {
		return { std::floor(value.x), std::floor(value.y), std::floor(value.z) };
	}

	[[nodiscard]] inline Vec3 ceil(const Vec3& value) {
		return { std::ceil(value.x), std::ceil(value.y), std::ceil(value.z) };
	}
}
