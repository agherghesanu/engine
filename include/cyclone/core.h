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

    // QUATERNION (For Orientation)
    class Quaternion {
    public:
        union {
            struct { real r, i, j, k; };
            real data[4];
        };

        Quaternion() : r(1), i(0), j(0), k(0) {}
        Quaternion(real r, real i, real j, real k) : r(r), i(i), j(j), k(k) {}

        // Normalizes the quaternion to unit length (required for rotation)
        void normalize() {
            real d = r * r + i * i + j * j + k * k;
            if (d == 0) { r = 1; return; }
            d = 1.0 / std::sqrt(d);
            r *= d; i *= d; j *= d; k *= d;
        }

        // Multiplies quaternions (combines rotations)
        void operator*=(const Quaternion& multiplier) {
            Quaternion q = *this;
            r = q.r * multiplier.r - q.i * multiplier.i - q.j * multiplier.j - q.k * multiplier.k;
            i = q.r * multiplier.i + q.i * multiplier.r + q.j * multiplier.k - q.k * multiplier.j;
            j = q.r * multiplier.j + q.j * multiplier.r + q.k * multiplier.i - q.i * multiplier.k;
            k = q.r * multiplier.k + q.k * multiplier.r + q.i * multiplier.j - q.j * multiplier.i;
        }

        // Adds a scaled vector (angular velocity) to the orientation
        void addScaledVector(const Vector3& vector, real scale) {
            Quaternion q(0, vector.x * scale, vector.y * scale, vector.z * scale);
            q *= *this;
            r += q.r * 0.5;
            i += q.i * 0.5;
            j += q.j * 0.5;
            k += q.k * 0.5;
        }

        void rotateByVector(const Vector3& vector) {
            Quaternion q(0, vector.x, vector.y, vector.z);
            (*this) *= q;
        }
    };

    // MATRIX 3 (For Inertia Tensors)
    class Matrix3 {
    public:
        real data[9];

        Matrix3() {
            data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = data[6] = data[7] = data[8] = 0;
        }

        Matrix3(real c0, real c1, real c2, real c3, real c4, real c5, real c6, real c7, real c8) {
            data[0] = c0; data[1] = c1; data[2] = c2;
            data[3] = c3; data[4] = c4; data[5] = c5;
            data[6] = c6; data[7] = c7; data[8] = c8;
        }

        // Transform a vector by this matrix
        Vector3 operator*(const Vector3& vector) const {
            return Vector3(
                vector.x * data[0] + vector.y * data[1] + vector.z * data[2],
                vector.x * data[3] + vector.y * data[4] + vector.z * data[5],
                vector.x * data[6] + vector.y * data[7] + vector.z * data[8]
            );
        }

        // Multiply matrix by matrix
        Matrix3 operator*(const Matrix3& o) const {
            return Matrix3(
                data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
                data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
                data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],

                data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
                data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
                data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],

                data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6],
                data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7],
                data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8]
            );
        }

        // Sets the matrix to be the inverse inertia tensor of a cuboid
        void setInverseInertiaTensor(const Vector3& halfSizes, real mass) {
            real squares = mass / 12.0;
            setInertiaTensorCoeffs(
                squares * (halfSizes.y * halfSizes.y + halfSizes.z * halfSizes.z),
                squares * (halfSizes.x * halfSizes.x + halfSizes.z * halfSizes.z),
                squares * (halfSizes.x * halfSizes.x + halfSizes.y * halfSizes.y)
            );
            invert(); 
        }

        void setInertiaTensorCoeffs(real ix, real iy, real iz, real ixy = 0, real ixz = 0, real iyz = 0) {
            data[0] = ix; data[1] = -ixy; data[2] = -ixz;
            data[3] = -ixy; data[4] = iy; data[5] = -iyz;
            data[6] = -ixz; data[7] = -iyz; data[8] = iz;
        }

        void invert() {
            real t4 = data[0] * data[4]; real t6 = data[0] * data[5]; real t8 = data[1] * data[3];
            real t10 = data[2] * data[3]; real t12 = data[1] * data[6]; real t14 = data[2] * data[6];
            // Calculate the determinant
            real t16 = (t4 * data[8] - t6 * data[7] - t8 * data[8] + t10 * data[7] + t12 * data[5] - t14 * data[4]);
            if (t16 == (real)0.0f) return; 
            real t17 = 1 / t16;

            data[0] = (data[4] * data[8] - data[5] * data[7]) * t17;
            data[1] = -(data[1] * data[8] - data[2] * data[7]) * t17;
            data[2] = (data[1] * data[5] - data[2] * data[4]) * t17;
            data[3] = -(data[3] * data[8] - data[5] * data[6]) * t17;
            data[4] = (data[0] * data[8] - t14) * t17;
            data[5] = -(data[0] * data[5] - t10) * t17;
            data[6] = (data[3] * data[7] - data[4] * data[6]) * t17;
            data[7] = -(data[0] * data[7] - t12) * t17;
            data[8] = (t4 - t8) * t17;
        }

        
        void setOrientation(const Quaternion& q) {
            data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
            data[1] = 2 * q.i * q.j - 2 * q.k * q.r;
            data[2] = 2 * q.i * q.k + 2 * q.j * q.r;
            data[3] = 2 * q.i * q.j + 2 * q.k * q.r;
            data[4] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
            data[5] = 2 * q.j * q.k - 2 * q.i * q.r;
            data[6] = 2 * q.i * q.k - 2 * q.j * q.r;
            data[7] = 2 * q.j * q.k + 2 * q.i * q.r;
            data[8] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
        }
    };

    // MATRIX 4 (For Transform / Rendering)
    class Matrix4 {
    public:
        real data[12]; 

        Matrix4() {
            data[1] = data[2] = data[3] = data[4] = data[6] = data[7] = data[8] = data[9] = data[11] = 0;
            data[0] = data[5] = data[10] = 1;
        }

        // Combine Position and Orientation into a Transform Matrix
        void setOrientationAndPos(const Quaternion& q, const Vector3& pos) {
            data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
            data[1] = 2 * q.i * q.j - 2 * q.k * q.r;
            data[2] = 2 * q.i * q.k + 2 * q.j * q.r;
            data[3] = pos.x;

            data[4] = 2 * q.i * q.j + 2 * q.k * q.r;
            data[5] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
            data[6] = 2 * q.j * q.k - 2 * q.i * q.r;
            data[7] = pos.y;

            data[8] = 2 * q.i * q.k - 2 * q.j * q.r;
            data[9] = 2 * q.j * q.k + 2 * q.i * q.r;
            data[10] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
            data[11] = pos.z;
        }

        // transform a vector (Orientation + Scale only, no translation)
        Vector3 transformDirection(const Vector3& vector) const {
            return Vector3(
                vector.x * data[0] + vector.y * data[1] + vector.z * data[2],
                vector.x * data[4] + vector.y * data[5] + vector.z * data[6],
                vector.x * data[8] + vector.y * data[9] + vector.z * data[10]
            );
        }

        // transform a position (Orientation + Position)
        Vector3 transform(const Vector3& vector) const {
            return Vector3(
                vector.x * data[0] + vector.y * data[1] + vector.z * data[2] + data[3],
                vector.x * data[4] + vector.y * data[5] + vector.z * data[6] + data[7],
                vector.x * data[8] + vector.y * data[9] + vector.z * data[10] + data[11]
            );
        }
    };

}


#endif // !CYCLONE_CORE_H
