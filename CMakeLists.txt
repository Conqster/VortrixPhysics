cmake_minimum_required(VERSION 3.18)

#generate project
project(VortrixPhysics VERSION 0.1.0 LANGUAGES CXX)

message(STATUS "CMake C++ compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMake C++ compiler ver: ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "CMake system name: ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMake host system processor: ${CMAKE_HOST_SYSTEM_PROCESSOR}")

option(ENABLE_ALL_WARNINGS "Enable all warning" ON)
# When turning this option on, the library will be compiled with debug symbols
option(GENERATE_DEBUG_SYMBOLS "Generate debug symbols" ON)
# When turning this on, the library will be compiled with C++ exceptions enabled.
# This adds some overhead and Vortrix doesn't use exceptions so by default it is off.
option(CPP_EXCEPTIONS_ENABLED "Enable C++ exceptions" OFF)

option(PROFILE_SIM "Profile simulation informations" ON)
option(PROFILE_BROAD "Profile Broadphase" ON)
option(PROFILE_NARROW "Profile Narrowphase" ON)
option(PROFILE_SOLVER "Profile Constraint Solver" ON)

option(USE_TRACY_PROFILER "Enable Tracy Profiler" ON)

option(USE_SSE "Enable SSE Optimisation" ON)
option(USE_AVX2 "Enable AVX2 & FMA Optimisation" ON) #only use for fmadd... at the moment

#later
#option(BUILD_TESTS "build unit tests" ON)
#option(BUILD_PLAYGROUND "build playground" ON)

option(VX_BUILD_SHARED_LIBS "Compile Vortrix as shared dynamic library" %{BUILD_SHARED_LIBS})

#utilities
option(USE_IMGUI "Enable ImGui for Sample Framework Editing" ON)


if(BUILD_SHARED_LIBS)
	message(STATUS "BUILD_SHARED_LIBS is on by default compiling as DLL")
else()
	message(STATUS "BUILD_SHARED_LIBS is off by default compiling as LIB")
endif()

if(MSVC)
	set(CMAKE_CONFIGURATION_TYPES "Debug;Development;Release;Distribution;ReleaseASan" CACHE STRING "" FORCE)
	

	#/EHsc for std::vector
	#Base flags
	set(VX_GLOBAL_FLAGS "/Zc:__cplusplus /nologo /Zc:inline /fp:fast /fp:except- /Gm- /GR- /EHsc")
	
	set(VX_GLOBAL_FLAGS "${VX_GLOBAL_FLAGS} /Zc:sizedDealloc") 

	
	if(ENABLE_ALL_WARNINGS)
		#  a lot i do not know, about compiler warning 
		#could not use flags /Wall /WX
		#as it break when treating all warnings as Error 
		#
		#in future need to look into suppressing some complier warning 
		#for now keep all warning and as warnings
		#set(VX_GLOBAL_FLAGS "${VX_GLOBAL_FLAGS} /Wall /WX") 
		set(VX_GLOBAL_FLAGS "${VX_GLOBAL_FLAGS} /Wall") 
	endif()
	
	#if(GENERATE_DEBUG_SYMBOLS)
	#	set(VX_GLOBAL_FLAGS "${VX_GLOBAL_FLAGS} /Zi")
	#endif()
	
	#if (NOT CPP_EXCEPTIONS_ENABLED)
		# Remove any existing compiler flag that enables exceptions
	#	string(REPLACE "/EHsc" "" VX_GLOBAL_FLAGS ${CMAKE_CXX_FLAGS})

		# Disable warning about STL and compiler-generated types using noexcept when exceptions are disabled
	#	set(VX_GLOBAL_FLAGS "${VX_GLOBAL_FLAGS} /wd4577")
	#else()
		# Enable exceptions
	#	set(VX_GLOBAL_FLAGS "${VX_GLOBAL_FLAGS} /EHsc")
	#endif()
	
	#common optimisation block
	set(PHYSICS_OPTS "${VX_GLOBAL_FLAGS} /O2 /Oi /Ot /DNDEBUG")
	
	set(CMAKE_CXX_FLAGS_DEBUG "/Zi /MP /nologo /Wall /MP /GS /fp:fast /fp:except- /Gm- /RTC1" CACHE STRING "" FORCE)
	set(CMAKE_EXE_LINKER_FLAGS_DEBUG "/DEBUG:FULL " CACHE STRING "" FORCE)
		 
	#Development (Release speed + uProf/VTune profiling)
	set(CMAKE_CXX_FLAGS_DEVELOPMENT "${PHYSICS_OPTS} /Zi /GS" CACHE STRING "" FORCE)
	set(CMAKE_EXE_LINKER_FLAGS_DEVELOPMENT "/DEBUG /OPT:REF /OPT:ICF" CACHE STRING "" FORCE)
	
	#Release Speed with no symbol but some profiling benchmarking support
	set(CMAKE_CXX_FLAGS_RELEASE "${PHYSICS_OPTS} /GS- /Gy /GL" CACHE STRING "" FORCE)
	set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/OPT:REF /OPT:ICF" CACHE STRING "" FORCE)
	
	#Distribution with Link Time Code Gen
	set(CMAKE_CXX_FLAGS_DISTRIBUTION "${PHYSICS_OPTS} /GS- /Gy /GL" CACHE STRING "" FORCE)
	set(CMAKE_EXE_LINKER_FLAGS_DISTRIBUTION "/LTCG /OPT:REF /OPT:ICF" CACHE STRING "" FORCE)
	
	#Release Speed with no symbol but some profiling benchmarking support
	set(CMAKE_CXX_FLAGS_RELEASEASAN "${PHYSICS_OPTS} /fsanitize=address /Zi" CACHE STRING "" FORCE)
	set(CMAKE_EXE_LINKER_FLAGS_RELEASEASAN  "/DEBUG" CACHE STRING "" FORCE)

endif()


set(VX_ROOT_DIR ${CMAKE_SOURCE_DIR})

# Vortrix Library 
include(${VX_ROOT_DIR}/Vortrix/Vortrix.cmake)

# Sample framework executable
set(VX_TARGET_SAMPLE_EXE VortrixPhysicsSample)
set(VX_SAMPLE_FRAMEWORK_DIR ${VX_ROOT_DIR}/SampleFramework)
include(${VX_SAMPLE_FRAMEWORK_DIR}/VxSampleFramework.cmake)

# shared between Vortrix library & sample framework
add_compile_definitions(
	$<$<CONFIG:Debug>:VX_DEBUG>
	$<$<CONFIG:Development>:VX_DEV>
	$<$<CONFIG:Release>:VX_REL>
	$<$<CONFIG:Distribution>:VX_DIST>
	$<$<CONFIG:ReleaseASan>:VX_REL_ASAN>
	
	#outputs Debug; Release etc
	VX_BUILD_NAME="$<CONFIG>"
	
	#string print outputs DEBUG; DEV, REL etc
	$<$<CONFIG:Debug>:VX_BUILD_STR="DEBUG">
	$<$<CONFIG:Development>:VX_BUILD_STR="DEV">
	$<$<CONFIG:Release>:VX_BUILD_STR="REL">
	$<$<CONFIG:Distribution>:VX_BUILD_STR="DIST">
	$<$<CONFIG:ReleaseASan>:VX_BUILD_STR="REL_ASAN">
	)
	



set_target_properties(${VX_TARGET_SAMPLE_EXE} PROPERTIES 
						#VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
						#VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
						VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${VX_TARGET_SAMPLE_EXE}>"
						VS_STARTUP_PROJECT TRUE)
						
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${VX_TARGET_SAMPLE_EXE})


#Build Events - Post-Build Event
#Command Line
#	xcopy /y /e /d "$(SolutionDir)assets" "$(OutDir)assets\"
#	xcopy /y /d "$(SolutionDir)external\glew\glew-2.1.0\bin\Release\x64\*.dll" "$(OutDir)"


get_filename_component(MSVC_DIR "${CMAKE_CXX_COMPILER}" DIRECTORY)
set(ASAN_DLL "${MSVC_DIR}/clang_rt.asan_dynamic-x86_64.dll")

#copy require data to output directory
add_custom_command(TARGET ${VX_TARGET_SAMPLE_EXE} POST_BUILD
	#assets 
	COMMAND ${CMAKE_COMMAND} -E copy_directory
	"${PROJECT_SOURCE_DIR}/assets/"
	"$<TARGET_FILE_DIR:${VX_TARGET_SAMPLE_EXE}>/assets"
	
	
	#dlls
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
	"${PROJECT_SOURCE_DIR}/external/glew/glew-2.1.0/bin/Release/x64/glew32.dll"
	"$<TARGET_FILE_DIR:${VX_TARGET_SAMPLE_EXE}>"
	
	
	
	#copy Asan dll
	COMMAND 
	$<$<CONFIG:ReleaseASan>:${CMAKE_COMMAND}>
	$<$<CONFIG:ReleaseASan>:-E>
	$<$<CONFIG:ReleaseASan>:copy_if_different>
	$<$<CONFIG:ReleaseASan>:${ASAN_DLL}>
	$<$<CONFIG:ReleaseASan>:$<TARGET_FILE_DIR:${VX_TARGET_SAMPLE_EXE}>>
	
	COMMENT "Depolying assets and DLLs to output directory"
	)


