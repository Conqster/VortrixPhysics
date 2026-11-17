#pragma once

#include <string>
#include <type_traits>
#include <iostream>

//Detect ToChar 
template<typename T, typename = void> 
struct Has_ToChar : std::false_type{};

template<typename T> 
struct Has_ToChar < T, std::void_t<
	decltype(ToChar(std::declval<T>(), std::declval<char*>(), size_t{})) >> : std::true_type{};

//Detect stack string
//template<typename>
//struct Is_Stack_String : std::false_type {};
//
//template<size_t N>
//struct Is_Stack_String<StackString<N>> : std::true_type{};

#include "Vortrix/Maths/Vec3.h"

namespace vx {


	/// not null-terminated 
	/// but null termination is abled when .Data() last element '\0'
	template<size_t N = 256>
	class StackString
	{
	public:
		StackString() = default;
		~StackString() = default; //default value not heap allocation

		StackString(const StackString&) = default;
		StackString& operator=(const StackString&) = default;
		StackString(StackString&&) noexcept = default;
		StackString& operator=(StackString&&) noexcept = default;


		explicit StackString(const char* chars)
		{
			if(chars)
				Append(chars, strlen(chars));
		}

		template<size_t M>
		explicit StackString(const char (&chars)[M])
		{
			Append(chars, M - 1);
		}





		void Append(const char* src, size_t len)
		{
			size_t to_copy = (len < Remaining()) ? len : Remaining();
			memcpy(mData + mCount, src, to_copy);
			mCount += to_copy;
			mData[mCount] = '\0';
		}

		void Append(char c)
		{
			if (Remaining() > 0)
				mData[mCount++] = c;
			mData[mCount] = '\0';
		}

		size_t Length() const { return mCount; }
		size_t Capacity() const { return N; }
		size_t Remaining() const { return N - mCount; }


		bool Empty() const { return mCount == 0; }

		void Clear() { mCount = 0; mData[0] = '\0'; }

		const char* Data() const { return mData; }

		char* BufferHead() { return &mData[0];}

		template<size_t M>
		StackString& operator << (const StackString<M>& other)
		{
			Append(other.Data(), other.Length());
			return *this;
		}

		StackString& operator << (const char* chars)
		{
			if (chars)
				Append(chars, strlen(chars));
			return *this;
		}
		template<size_t M>
		StackString& operator << (const char(&chars)[M])
		{
			Append(chars, M - 1);
			return *this;
		}
		StackString& operator << (const std::string& str)
		{
			Append(str.data(), str.size());
			return *this;
		}


		//need to support StackString 
		template<typename T>
		StackString& operator << (const T& value)
		{
			char tmp[64];
			int len = 0;
			if constexpr (std::is_integral_v<T>)
				len = snprintf(tmp, sizeof(tmp), "%lld", (long long)value);
			else if constexpr (std::is_floating_point_v<T>)
				len = snprintf(tmp, sizeof(tmp), "%g", (double)value);
			else if constexpr (Has_ToChar<T>::value)
				len = ToChar(value, tmp, sizeof(tmp));
			else
			{
				//static_assert(sizeof(T) == 0, "Unsuppoted type for StackString <<");
				len = 0;
			}

			Append(tmp, len);
			return *this;
		}
	private:
		char mData[N]{};
		size_t mCount = 0;
		static_assert(N > 1);
	};



	///ToStackString<>("%.2f", 2.12345f)
	template<size_t N = 16, typename T>
	StackString<N> ToStackString(char const* const format, const T& value)
	{
		char tmp[N];
		int len = 0;
		if constexpr (std::is_integral_v<T>)
			len = snprintf(tmp, sizeof(tmp), format, (long long)value);
		else if constexpr (std::is_floating_point_v<T>)
			len = snprintf(tmp, sizeof(tmp), format, (double)value);
		else if constexpr (std::is_convertible_v<T, const char*>)
		{
			static_assert(sizeof(T) == 0, "Unsuppoted type for ToStackString ");
		}
		StackString<N> result;
		result.Append(tmp, len);
		return result;
	}


	//	template<size_t N>
	//VX_INLINE StackString<N>& operator<<(StackString<N>& buff, const Vec3& v)
	//{
	//	char buffer[64];
	//	constexpr const char* format = "{x: %.2f, y: %.2f, z: %.2f}";
	//	int len = snprintf(buffer, sizeof(buffer), format, v[0], v[1], v[2]);
	//	if (len > 0)
	//		buff.Append(buffer);
	//	return buff;
	//}



	template<size_t N = 16, typename ...Args>
	static vx::StackString<N> ConvertToString(const char* format, Args&&... args)
	{
		char buffer[N];
		snprintf(buffer, sizeof(buffer), format, std::forward<Args>(args)...);
		return vx::StackString<N>(buffer);
	}

	template<size_t N = 16>
	static vx::StackString<N> ToString(float v, const char* format = "%.2f")
	{
		return ConvertToString<N>(format, v);
	}

	template<size_t N = 16>
	static vx::StackString<N> ToString(double v, const char* format = "%.2f")
	{
		return ConvertToString<N>(format, v);
	}

	template<size_t N = 16>
	static vx::StackString<N> ToString(int v, const char* format = "%.d")
	{
		return ConvertToString<N>(format, v);
	}

	template<size_t N = 32>
	static vx::StackString<N> ToString(Vec3 v, const char* format = "[%.2f,%.2f,%.2f]")
	{
		return ConvertToString<N>(format, v.X(), v.Y(), v.Z());
	}
}