#include <SampleFramework/SampleFramework.h>

#include "Shader.h"
#include <fstream>

#include <string>
#include "ErrorAssertion.h"
#include "SampleFramework/Display/ApplicationWindow.h"

bool Shader::Create(const std::string_view name, const std::string_view  ver, const std::string_view  frag, const std::string_view  geo, bool force_debug)
{
	mName = std::string(name);
	std::string vertex_code = ReadFile(ver, force_debug);
	std::string fragment_code = ReadFile(frag, force_debug);
	bool has_geometry_shader = !geo.empty();
	std::string geometry_code = (has_geometry_shader) ? ReadFile(geo, force_debug) : "";


	
	mID = glCreateProgram();

	std::string debug_defines;
	if(force_debug)
	{
		bool is_render_doc = !ApplicationWindow::SupportsBindless();
		std::string defines = "#version 430\n";
		debug_defines = is_render_doc ? "#define RENDERDOC_DEBUG 1\n" : "#define RENDERDOC_DEBUG 0\n";
		debug_defines = defines + debug_defines;
	}

	GLuint vertex_shader = CompileShader(GL_VERTEX_SHADER, vertex_code, debug_defines);
	GLuint frag_shader = CompileShader(GL_FRAGMENT_SHADER, fragment_code, debug_defines);
	GLuint geo_shader = (has_geometry_shader) ? CompileShader(GL_GEOMETRY_SHADER, geometry_code, debug_defines) : 0;

	glAttachShader(mID, vertex_shader);
	glAttachShader(mID, frag_shader);
	if (has_geometry_shader)
		glAttachShader(mID, geo_shader);

	glLinkProgram(mID);
	glValidateProgram(mID);

	GLint result = 0;
	GLchar eLog[1024] = { 0 };
	glGetProgramiv(mID, GL_VALIDATE_STATUS, &result);

	if (!result)
	{
		glGetProgramInfoLog(mID, sizeof(eLog), NULL, eLog);
		VX_LOG_WARN("[ERROR VALIDATING PROGRAM {", mName, ")]: ", eLog);


		//need to destroy shader if compiled 
		glDeleteShader(vertex_shader);
		glDeleteShader(frag_shader);
		if (has_geometry_shader)
			glDeleteShader(geo_shader);

		return false;
	}

	glDeleteShader(vertex_shader);
	glDeleteShader(frag_shader);
	if (has_geometry_shader)
		glDeleteShader(geo_shader);

	glObjectLabel(GL_PROGRAM, mID, mName.size(), mName.c_str());

	ReflectUniforms();
	return true;
}

void Shader::Bind() const
{
	GLCall(glUseProgram(mID));
}

void Shader::UnbindProgram()
{
	GLCall(glUseProgram(0));
}

void Shader::SetUniformMat4(const char* name, const vx::Mat44& matrix)
{
	glUniformMatrix4fv(GetUniformLocation(name), 1, GL_FALSE, matrix.Data());
}

void Shader::SetUniform1i(const char* name, int value)
{
	glUniform1i(GetUniformLocation(name), value);
}

void Shader::SetUniform1f(const char* name, float value)
{
	glUniform1f(GetUniformLocation(name), value);
}

void Shader::SetUniformVec2(const char* name, const vx::Vec2& value)
{
	glUniform2f(GetUniformLocation(name), value.X(), value.Y());
}

void Shader::SetUniformVec3(const char* name, const vx::Vec3& value)
{
	glUniform3f(GetUniformLocation(name), value.X(), value.Y(), value.Z());
}

void Shader::SetUniformVec4(const char* name, const vx::Vec4& value)
{
	glUniform4f(GetUniformLocation(name), value.X(), value.Y(), value.Z(), value.W());
}

void Shader::Clear()
{
	//TO-DO: not too sure but a program could accually been 0
	if (mID != 0)
	{
		glDeleteProgram(mID);
		mID = 0;
	}

	cacheUniformLocations.clear();
	mUniformNameBuffer.clear();
}

std::string Shader::ReadFile(const std::string_view shader_file, bool remove_ver)
{
	std::string content;
	std::ifstream fileStream(shader_file.data(), std::ios::in);
	if (!fileStream.is_open())
	{
		VX_LOG_WARN("[Reading Shader File {", mName,"}]: Failed to read %s, file doesn't exist.\n", shader_file);
		return "";
	}

	std::string line = "";
	//while (!fileStream.eof())
	//{
	//	std::getline(fileStream, line);
	//	content.append(line + "\n");
	//}
	//fileStream.close();

	bool resolved = !remove_ver; //<-- quick, if not remove then auto resolved
	while (std::getline(fileStream, line))
	{
		/// when to chane 
		/// !resolved
		/// remove
		/// then find
		if (!resolved && line.find("#version") != std::string::npos)
			resolved = true;
		else
			content.append(line + "\n");
	}
	fileStream.close();

	return content;
}



unsigned int Shader::CompileShader(GLenum shader_type, const std::string& source, const std::string& debug_defines)
{
	GLuint shaderid = glCreateShader(shader_type);

	if (debug_defines.empty())
	{
		const char* src = source.c_str();
		glShaderSource(shaderid, 1, &src, nullptr);
	}
	else
	{
		const char* src[] = { debug_defines.c_str(), source.c_str()};
		glShaderSource(shaderid, 2, src, nullptr);

	}
	
	glCompileShader(shaderid);

	GLint result = 0;
	GLchar eLog[1024] = { 0 };
	glGetShaderiv(shaderid, GL_COMPILE_STATUS, &result);

	if (!result)
	{
		//TO-DO: Need to fix the debug message (for easy debugging)
		//current issue: misspelt a data type (sampler as sample) wrong error message
		//Error message was generic syntax error, unexpected IDENTIFIER, expecting
		// LEFT_BRACE or COMMA or SEMICOLON.
		glGetShaderInfoLog(shaderid, sizeof(eLog), NULL, eLog);
		//printf("[SHADER]: Couldn't create a %s, %sSOURCE: %s\n", LogShaderTypeName2(type), eLog, GetShaderFilePath(type));
		//printf("[SHADER]: Couldn't create a shader, \n%sFile: \n%s\n", eLog, source.c_str());
		const char* _type;
		switch (shader_type)
		{
			case GL_VERTEX_SHADER: _type = "Vertex";
				break;
			case GL_FRAGMENT_SHADER: _type = "Fragment";
				break;
			case GL_GEOMETRY_SHADER: _type = "Geometry";
				break;
			case GL_COMPUTE_SHADER: _type = "Compute"; 
				break;
			default: _type = "Unknown Shader type";
		}

		//VX_WARN("[SHADER]: Couldn't create a shader {", _type, "}, ", eLog, "File source: ", src);
		const char* debug_src = (debug_defines.empty()) ? "N/A" : debug_defines.c_str();
		VX_LOG_WARN("[SHADER:", mName, "]: Couldn't create a shader {", _type, "}, ", eLog, "\nShader Debug Defines: \n", debug_src, "\nFile source: \n", source.c_str());
		
		exit(-1);
	}

	return shaderid;
}

GLint Shader::GetUniformLocation(std::string_view name)
{
	//if (cacheUniformLocations.find(name) != cacheUniformLocations.end())
	//	return cacheUniformLocations[name];

	auto it = cacheUniformLocations.find(name);
	if (it != cacheUniformLocations.end())
		return it->second;

	VX_LOG_WARN("[SHADER UNIFORM (WARNING) program {", mName, "}]: uniform '", name, "' doesn't exist!!!!!");
	return -1;
	//int location = glGetUniformLocation(mID, name.data());

	//VX_ASSERT_WARN(location != -1, (std::string("[SHADER UNIFORM (WARNING) program {") + mName + "}]: uniform '%s' doesn't exist!!!!!" + name.data()).c_str());

	//if (location != -1)
	//	cacheUniformLocations[name.data()] = location;


	//return location;
}

void Shader::ReflectUniforms()
{
	GLint count;
	glGetProgramiv(mID, GL_ACTIVE_UNIFORMS, &count);

	mUniformNameBuffer.reserve(count);
	cacheUniformLocations.reserve(count);

	char name_buff[128];//changing to StackString

	for (int i = 0; i < count; ++i)
	{
		GLsizei length;
		glGetActiveUniformName(mID, i, sizeof(name_buff), &length, name_buff);
		name_buff[length] = '\0';

		//string permanetly inside the shader
		//string view to stable uniform names preventing per frame allocation
		std::string_view view = mUniformNameBuffer.emplace_back(name_buff);

		GLint loc = glGetUniformLocation(mID, name_buff);
		cacheUniformLocations.emplace(view, loc);
	}
}
