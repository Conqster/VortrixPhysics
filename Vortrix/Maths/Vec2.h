#pragma once

#include "Core.h"


namespace vx
{

	class alignas(8) Vec2
	{
	public:
		Vec2() = default;
		Vec2(float _x, float _y) : x(_x), y(_y){}
		explicit Vec2(float scalar) : x(scalar), y(scalar){}
		Vec2(const Vec2& rhs) = default;
		explicit Vec2(__m128 vec);

		/// @return x component value
		VX_INLINE float X() const { return x; }
		/// @return y component value
		VX_INLINE float Y() const { return y; }

		/// set x component
		VX_INLINE void SetX(float v) { x = v; }
		/// set y component
		VX_INLINE void SetY(float v) { y = v; }

		VX_INLINE void SetXY(float _x, float _y) { x = _x; y = _y; }


		/// Component accessor by index
		/// @param i Index (0 = x, 1 = y)
		/// @return reference to component
		VX_INLINE float& operator[](uint32_t i);
		/// Component accessor by index (const)
		/// @param i Index (0 = x, 1 = y)
		/// @return const reference to component
		VX_INLINE float const& operator[](uint32_t i) const;

		/// Set both components to zero
		VX_INLINE void ToZero();
		/// Vector with all zero 
		VX_INLINE static Vec2 Zero() { return Vec2(0.0f); }
		/// Vector with all one 
		VX_INLINE static Vec2 One() { return Vec2(1.0f); }
		/// Vector [1,0] 
		VX_INLINE static Vec2 Right() { return Vec2(1.0f, 0.0f); }
		/// Vector [0,1] 
		VX_INLINE static Vec2 Up() { return Vec2(0.0f, 1.0f); }

		/// Component-wise absolute value
		VX_INLINE Vec2 Abs() const;
		/// Component-wise sign 
		VX_INLINE Vec2 Sign() const;
		/// Check if any component is NaN
		/// @return true if any component is NaN
		VX_INLINE bool IsNaN() const;
		/// Chech if vector approx zero
		/// @return tolerance allowed absolute error
		VX_INLINE bool IsZero(float tolerance = 1e-6f) const;
		/// Approximate equality check
		/// @return rhs Vector to compare
		/// @param tolerance_sq Square tolerance
		VX_INLINE bool IsApprox(const Vec2& rhs, float tolerance_sq = 1e-12f) const;
		/// Check if vector is normalised
		/// @param tolerance allowed length error
		VX_INLINE bool IsNormalised(float tolerance = 1e-6f) const;

		/// Comparison
		VX_INLINE bool operator == (const Vec2& rhs) const;
		VX_INLINE bool operator != (const Vec2& rhs) const { return !(*this == rhs); }

		/// smallest component value
		VX_INLINE float MinComponent() const;
		/// Largest component value
		VX_INLINE float MaxComponent() const;

		/// @return a vector from the smallest component of lhs & rhs vectors
		VX_INLINE static Vec2 Min(const Vec2& lhs, const Vec2& rhs);
		/// @return a vector from the largest component of lhs & rhs vectors
		VX_INLINE static Vec2 Max(const Vec2& lhs, const Vec2& rhs);
		/// Clamp each component between min & max
		/// @param v in vector
		/// @return v in range [min, max]
		VX_INLINE static Vec2 Clamp(const Vec2& v, const Vec2& min, const Vec2& max);

		/// Dot product
		VX_INLINE float Dot(const Vec2& rhs) const;
		/// The unsigned angle (radians) between this and to
		/// @param this vector from which the angular difference is measured.
		/// @param to vector to which the angular difference is measured.
		/// @return unsigned angle (radians) between the two vectors
		VX_INLINE float Angle(const Vec2& to) const;
		/// The signed angle (radians) between this and to
		/// @param this vector from which the angular difference is measured.
		/// @param to vector to which the angular difference is measured.
		/// @return signed angle (radians) between the two vectors
		VX_INLINE float SignedAngle(const Vec2& to) const;

		///@return the squared length (magnitude) of the vector
		VX_INLINE float LengthSq() const;
		///@return the length (magnitude) of the vector
		VX_INLINE float Length() const;

		/// normlise this vector
		/// @return normalise copy of this 
		/// of length 1.
		VX_INLINE Vec2 Normalised() const;
		/// normlise this vector in place
		/// @return reference to this
		VX_INLINE Vec2& Normalise();

		/// Negate Vec2
		/// @return negate copy of this
		VX_INLINE Vec2 Inverted() const;
		/// Negate in place 
		/// @return reference to this
		VX_INLINE Vec2& Invert();
		/// Compute a perpendicular vector to the vector 
		/// @return perpendicular vector to this.
		VX_INLINE Vec2 Perpendicular() const;

		/// Projects this vector onto nor
		/// assume nor is of length 1.
		/// @param nor Normalised direction
		/// @returns (this · nor) nor
		VX_INLINE Vec2 Project(const Vec2& nor) const;
		/// Reject this vector onto nor
		/// assume nor is of length 1.
		/// @param nor Normalised direction
		/// @return (this · nor) nor - this
		VX_INLINE Vec2 Reject(const Vec2& nor) const;
		/// Reflect this vector about a normal
		/// assume nor is of length 1.
		/// @param nor Normalised direction
		/// @return R = V - 2 * V.Dot(N) * N
		VX_INLINE Vec2 Reflect(const Vec2& nor) const;
		/// Linear interpolation
		/// @param from Start vector 
		/// @param to End vector
		/// @param t range [0, 1]
		VX_INLINE static Vec2 Lerp(const Vec2& from, const Vec2& to, float t);

		///Component wise Square root
		VX_INLINE Vec2 Sqrt() const;
		///Component wise Square root (in place)
		VX_INLINE Vec2& SqrtAssign();

		/// swizzle (shhuffle) components
		template<Axis Swizzle_X, Axis Swizzle_Y>
		VX_INLINE [[nodiscard]] Vec2 Swizzle() const;

		template<int X, int Y>
		VX_INLINE [[nodiscard]] static Vec2 Swizzle(const vx::Vec2& a, const vx::Vec2& b)
		{
			VX_ASSERT(X >= 0 && X < 4, "");
			VX_ASSERT(Y >= 0 && Y < 4, "");

			float buffer[4] = { a[0], a[1], b[0], b[1] };
			return Vec2(buffer[X], buffer[Y]);
		}

		/// Add 2 vector float component wise
		VX_INLINE Vec2 operator+(const Vec2& rhs) const;
		/// Add 2 vector float component wise
		VX_INLINE Vec2& operator+=(const Vec2& rhs);
		/// subtract 2 vector float component wise
		VX_INLINE Vec2 operator-(const Vec2& rhs) const;
		/// subtract 2 vector float component wise
		VX_INLINE Vec2& operator -=(const Vec2& rhs);
		/// multiply vector with float
		VX_INLINE Vec2 operator*(const float scalar) const;
		/// multiply vector with float
		VX_INLINE friend Vec2 operator*(const float lhs, const Vec2& rhs);
		/// multiply vector with float
		VX_INLINE Vec2& operator*=(const float scalar);
		/// divide vector with float
		VX_INLINE Vec2 operator/(const float scalar) const;
		/// divide float with the vector
		VX_INLINE friend Vec2 operator/(const float lhs, const Vec2& rhs);
		/// divide vector with float
		VX_INLINE Vec2& operator/=(const float scalar);
		/// unary negation
		VX_INLINE Vec2 operator -() const;

		///component wise multiply
		VX_INLINE Vec2 operator*(const Vec2& rhs) const;
		///component wise multiply
		VX_INLINE Vec2& operator*=(const Vec2& rhs);
		/// coponent wise divide
		VX_INLINE Vec2 operator/(const Vec2& rhs) const;
		/// coponent wise divide
		VX_INLINE Vec2& operator/=(const Vec2& rhs);

		VX_INLINE friend std::ostream& operator<<(std::ostream& os, const Vec2& v)
		{
			os << "Vec2(" << v.x << ", " << v.y << ")";
			return os;
		}


#ifdef VX_SIMD_SSE
		/// Store SIMD vector into this Vec2
		/// Uses lower two lanes
		VX_INLINE void Store(__m128 v)
		{
			_mm_storel_pi(reinterpret_cast<__m64*>(this), v);
		}
#endif // VX_SIMD_SSE


		///////NEW NEW 

		/// rotate complex multiple 
		/// x' = x * cos(angle) - y * sin(angle)
		/// y' = x * sin(angle) + y * cos(angle)
		/// 
		/// cos_angle = cos(angle), sin_angle = sin(angle)
		/// 
		VX_INLINE Vec2 Rotated(float cos_angle, float sin_angle)
		{
			float _x = x * cos_angle - y * sin_angle;
			float _y = x * sin_angle + y * cos_angle;
			return { _x, _y };
		}

		VX_INLINE Vec2 Rotated(float angle)
		{
			return Rotated(VxCos(angle), VxSin(angle));
		}

		/// rotate (x,y) by cos_angle & sin_angle counter clockwise
		VX_INLINE void Rotate(float cos_angle, float sin_angle)
		{
			float _x = x* cos_angle - y * sin_angle;
			float _y = x* sin_angle + y * cos_angle;
			x = _x;
			y = _y;
		}
		VX_INLINE void Rotate(float angle)
		{
			return Rotate(VxCos(angle), VxSin(angle));
		}



	private:

		/// Get SIMD represenation (x, y, in lowe lanes)
		VX_INLINE __m128 SimdValue() const;
		union
		{
			struct { float x, y; };
			//for iterative support
			float mFloats[2];
		};
	};

	static_assert(std::is_trivial_v<Vec2>);

}
#include "Vec2.inl"