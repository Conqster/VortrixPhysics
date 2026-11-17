#ISO c++ 17 standard required for features like (std::hash, std::unorderedmap, std::filesystem, etc)
#target_compile_features(crrt_compiler_flags INTERFACE cxx_std_17) 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


file(GLOB_RECURSE VX_APP_FILES
								${VX_SAMPLE_FRAMEWORK_DIR}/*.h
								${VX_SAMPLE_FRAMEWORK_DIR}/*.inl
								${VX_SAMPLE_FRAMEWORK_DIR}/*.cpp)
#for code organisation 
source_group(TREE ${VX_SAMPLE_FRAMEWORK_DIR} PREFIX "SampleFramework" FILES ${VX_APP_FILES})
message(STATUS "VX_SAMPLE_FRAMEWORK_DIR: ${VX_SAMPLE_FRAMEWORK_DIR}")

set(VX_SAMPLE_FRAMEWORK_FILES ${VX_APP_FILES})

#required imgui code
if(USE_IMGUI)
	set(VX_EXTERNAL_IMGUI_DIR ${VX_ROOT_DIR}/external/imgui)
	file(GLOB_RECURSE IMGUI_FILES 
						${VX_EXTERNAL_IMGUI_DIR}/*.h 
						${VX_EXTERNAL_IMGUI_DIR}/*.c 
						${VX_EXTERNAL_IMGUI_DIR}/*.cpp)
						
						
set(VX_SAMPLE_FRAMEWORK_FILES ${VX_SAMPLE_FRAMEWORK_FILES} ${IMGUI_FILES})	
source_group(TREE ${VX_EXTERNAL_IMGUI_DIR} PREFIX "UI-ImGui" FILES ${IMGUI_FILES})					
endif()



add_executable(${VX_TARGET_SAMPLE_EXE} ${VX_SAMPLE_FRAMEWORK_FILES})

target_compile_definitions(${VX_TARGET_SAMPLE_EXE} PUBLIC APP_ASSERT_DIR="${PROJECT_SOURCE_DIR}/assets/")

# link libraries
target_link_libraries(${VX_TARGET_SAMPLE_EXE} PRIVATE Vortrix)
set(SAMPLE_EXT_LIBS opengl32.lib glfw3.lib glew32.lib)
target_link_libraries(${VX_TARGET_SAMPLE_EXE} PUBLIC ${SAMPLE_EXT_LIBS})

#additional include directory 
#	project directory	
# 	/external
# 	/external/glfw/include
# 	/external/glew/glew-2.1.0/include
target_include_directories(${VX_TARGET_SAMPLE_EXE} PUBLIC
												${PROJECT_SOURCE_DIR}
												${PROJECT_SOURCE_DIR}/external
												${VX_ROOT_DIR}
												${PROJECT_SOURCE_DIR}/external/glfw/include
												${PROJECT_SOURCE_DIR}/external/glew/glew-2.1.0/include)
												
#Linker 
#Additional Library directory
#	glew
# 	glfw
# Additional dependencies
#	opengl32.lib
#	glfw3.lib	
#	glew32.lib

#could use "link_directories" 
target_link_directories(${VX_TARGET_SAMPLE_EXE} PUBLIC
											${PROJECT_SOURCE_DIR}/external/glew/glew-2.1.0/lib/Release/x64
											${PROJECT_SOURCE_DIR}/external/glfw/lib-vc2022)

					
target_compile_definitions(${VX_TARGET_SAMPLE_EXE} PRIVATE
					PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

