#ifndef CYCLONE_CORE_H
#define CYCLONE_CORE_H

#include <cmath>

#include <iostream>

namespace cyclone {
	using real = double;

	class Vector3 {
	public:
		real x;
		real y;
		real z;

		constexpr Vector3() : x(0), y(0), z(0), padding(0) {}

		constexpr Vector3(real x, real y, real z) : x(x), y(y), z(z), padding(0) {}

		//dot procuct
		[[nodiscard]] real operator*(const Vector3& other) const {
			return x * other.x + y * other.y + z * other.z;
		}

		//cross product
		[[nodiscard]] Vector3 operator^(const Vector3& other) const {
			return Vector3(
				y * other.z - z * other.y,
				z * other.x - x * other.z,
				x * other.y - y * other.x
			);
		}

		//operatior overloads for vector arithmetic
		void operator*=(const real value) {
			x *= value;
			y *= value;
			z *= value;
		}

		Vector3 operator*(const real value) const {
			return Vector3(x * value, y * value, z * value);
		}


		Vector3 operator+(const Vector3& other) const {
			return Vector3(x + other.x, y + other.y, z + other.z);
		}

		Vector3 operator-(const Vector3& other) const {
			return Vector3(x - other.x, y - other.y, z - other.z);
		}

		void operator+=(const Vector3& other) {
			x += other.x;
			y += other.y;
			z += other.z;
		}

		void operator-=(const Vector3& other) {
			x -= other.x;
			y -= other.y;
			z -= other.z;
		}

		//newtons third law useful for equal or opposite forces
		void invert() {
			x = -x;
			y = -y;
			z = -z;
		}

		//calculation of the magnitude of the vector
		[[nodiscard]] real magnitude() const {
			return std::sqrt(x * x + y * y + z * z);
		}


		//sqrt is expensive so this is useful for comparisons
		[[nodiscard]] real squareMagnitude() const {
			return x * x + y * y + z * z;
		}


		//we use multiplication by the inverse of the magnitude to avoid 
		// division
		void normalize() {
			real mag = magnitude();
			if (mag > 0) {
				*this *= (1.0 / mag);
			}
		}


	private:
		real padding; // align at 32 bytes
	};
}


#endif // !CYCLONE_CORE_H
