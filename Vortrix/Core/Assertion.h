#pragma once

#include "Core.h"


#if defined(VX_DEBUG) || defined(VX_DEV) || defined(VX_REL_ASAN)
#define VX_CORE_ENABLE_ASSERTS
#include <iostream>
#endif // defined(VX_DEBUG) || defined(VX_DEV) || defined(VX_REL_ASAN)


namespace vx
{
	//lvl 
	// 0 -> none
	// 1 -> Warn
	// 2 -> error
	//
	using VxAssertFailedFunction = void(*)(const char* expression, const char* message, 
		const unsigned int lvl, const char* file, unsigned int line, const char* func);
	extern VxAssertFailedFunction VxAssertionFailedFunc;

#if defined(VX_CORE_ENABLE_ASSERTS)
	/// if we are in ASan mode, downgrade lvl2 (crash) to lvl1 (warning)
	#if defined(VX_REL_ASAN)
		#define VX_ASSERT_VAL(lvl) 1
	#else
		#define VX_ASSERT_VAL(lvl) lvl
	#endif // defined(VX_REL_ASAN)


	namespace internals {
		static void DefaultAssertHandler(const char* expr, const char* message,
			const unsigned int lvl, const char* file, unsigned int line, const char* func);
	}

	inline void VxSetAssertFailedFunctionHandler(VxAssertFailedFunction handler)
	{
		VxAssertionFailedFunc = handler;
	}

#define VX_ASSERT_IMPL(expr, msg, lvl) \
	do { if(!(expr)) { \
			constexpr int actual_lvl = VX_ASSERT_VAL(lvl); \
			if (vx::VxAssertionFailedFunc) vx::VxAssertionFailedFunc(#expr, msg, actual_lvl, __FILE__, __LINE__, __FUNCTION__); \
			else \
			std::cerr << "[ASSERTION FAILED] (no output function hander): " << \
			#expr << ".\n"; \
			if(actual_lvl == 2) \
			VX_DEBUG_BREAK(); \
	 } } while (0)


#define VX_ASSERT_RET_IMPL(expr, msg, lvl, ...) \
	do { if(!(expr)) { \
			constexpr int actual_lvl = VX_ASSERT_VAL(lvl); \
			if (vx::VxAssertionFailedFunc) vx::VxAssertionFailedFunc(#expr, msg, actual_lvl, __FILE__, __LINE__, __FUNCTION__); \
			else \
			std::cerr << "[ASSERTION FAILED] (no output function hander): " << \
			#expr << ".\n"; \
			if(actual_lvl == 2) VX_DEBUG_BREAK(); \
			return __VA_ARGS__; \
	 } } while (0)



#define VX_ASSERT_INTERNAL_WITH_MSG(expr, msg) VX_ASSERT_IMPL(expr, msg, 2)
#define VX_ASSERT_INTERNAL_NO_MSG(expr) VX_ASSERT_IMPL(expr, nullptr, 2)

#define VX_ASSERT_INTERNAL_WARN_WITH_MSG(expr, msg) VX_ASSERT_IMPL(expr, msg, 1)
#define VX_ASSERT_INTERNAL_WARN_NO_MSG(expr) VX_ASSERT_IMPL(expr, nullptr, 1)


//selector when a macro takes 1 or 2 arguments
#define VX_SELECT_2(__1, __2, TARGET_MACRO, ...) TARGET_MACRO
//selector when a macro takes 2 or 3 arguments
#define VX_SELECT_3(__1, __2, __3, TARGET_MACRO, ...) TARGET_MACRO


#define VX_ASSERT(...) EXPAND_MACRO(VX_SELECT_2(__VA_ARGS__, VX_ASSERT_INTERNAL_WITH_MSG, VX_ASSERT_INTERNAL_NO_MSG)(__VA_ARGS__) )
#define VX_ASSERT_WARN(...) EXPAND_MACRO(VX_SELECT_2(__VA_ARGS__, VX_ASSERT_INTERNAL_WARN_WITH_MSG, VX_ASSERT_INTERNAL_WARN_NO_MSG)(__VA_ARGS__) )





#define VX_ASSERT_INTERNAL_WARN_RET_WITH_MSG(expr, ret_val, msg) VX_ASSERT_RET_IMPL(expr, msg, 1, ret_val)
#define VX_ASSERT_INTERNAL_WARN_RET_NO_MSG(expr, ret_val) VX_ASSERT_RET_IMPL(expr, nullptr, 1, ret_val)

#define VX_ASSERT_INTERNAL_WARN_VOID_WITH_MSG(expr, msg) VX_ASSERT_RET_IMPL(expr, msg, 1, )
#define VX_ASSERT_INTERNAL_WARN_VOID_NO_MSG(expr) VX_ASSERT_RET_IMPL(expr, nullptr, 1, )

/// params( expression, fail return value, warning message)
#define VX_ASSERT_WARN_RETURN(...) EXPAND_MACRO(VX_SELECT_3(__VA_ARGS__, VX_ASSERT_INTERNAL_WARN_RET_WITH_MSG, VX_ASSERT_INTERNAL_WARN_RET_NO_MSG)(__VA_ARGS__) )
/// params( expression, warning message)
#define VX_ASSERT_WARN_VOID(...) EXPAND_MACRO(VX_SELECT_2(__VA_ARGS__, VX_ASSERT_INTERNAL_WARN_VOID_WITH_MSG, VX_ASSERT_INTERNAL_WARN_VOID_NO_MSG)(__VA_ARGS__) )


#else

#define VX_ASSERT(...)	((void)0)
#define VX_ASSERT_WARN(...)	((void)0)
inline void VxSetAssertFailedFunctionHandler(VxAssertFailedFunction handler) {}

//selector when a macro takes 1 or 2 arguments
#define VX_SELECT_2(__1, __2, TARGET_MACRO, ...) TARGET_MACRO
#define VX_SELECT_3(__1, __2, __3, TARGET_MACRO, ...) TARGET_MACRO

//#define VX_ASSERT_RET_IMPL(expr, msg, ...) \
//	do { if(!(expr)) { \
//			return __VA_ARGS__; \
//	 } } while (0)

//#define VX_ASSERT_RET_IMPL(expr, msg, ...) \
//			return __VA_ARGS__ \

#define VX_ASSERT_RET_IMPL(expr, msg, ...) \

#define VX_ASSERT_WARN_RET_WITH_MSG(expr, ret_val, msg) VX_ASSERT_RET_IMPL(expr, msg, ret_val)
#define VX_ASSERT_WARN_RET_NO_MSG(expr, ret_val) VX_ASSERT_RET_IMPL(expr, nullptr, ret_val)

#define VX_ASSERT_WARN_VOID_WITH_MSG(expr, msg) VX_ASSERT_RET_IMPL(expr, msg, )
#define VX_ASSERT_WARN_VOID_NO_MSG(expr) VX_ASSERT_RET_IMPL(expr, nullptr, )

#define VX_ASSERT_WARN_RETURN(...) EXPAND_MACRO(VX_SELECT_3(__VA_ARGS__, VX_ASSERT_WARN_RET_WITH_MSG, VX_ASSERT_WARN_RET_NO_MSG)(__VA_ARGS__) )
#define VX_ASSERT_WARN_VOID(...) EXPAND_MACRO(VX_SELECT_2(__VA_ARGS__, VX_ASSERT_WARN_VOID_WITH_MSG, VX_ASSERT_WARN_VOID_NO_MSG)(__VA_ARGS__) )
#endif // VX_CORE_ENABLE_ASSERTS

} //namesapce vx

