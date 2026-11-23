#include "DebugGizmosRenderer.h"
#include "SampleFramework/Camera.h"


#include "SampleFramework/Renderer/Framebuffer.h"
#include <array>

#include "Vortrix/Geometry/AABB.h"

#include "SampleFramework/Display/ApplicationWindow.h"

#include "Vortrix/Core/Profiler.h"

#include "Vortrix/Core/Colours.h"

bool DebugGizmosRendererImpl::Init(ApplicationWindow* window)
{
	mActiveWindow = window;
	//bool success = SetShader(nullptr);

	vx::Ref<Shader> line_shader = vx::MakeRef<Shader>();
	bool success = line_shader->Create("debug-line-shader",
		"assets/shaders/debug/batchLines.vert", //vertex shader
		"assets/shaders/debug/batchLines.frag"); //fragment shader
	vx::Ref<Shader> tri_shader = vx::MakeRef<Shader>();
	success |= tri_shader->Create("debug-line-shader",
		"assets/shaders/debug/DebugBatchTriangle.vert", //vertex shader
		"assets/shaders/debug/DebugBatchTriangle.frag"); //fragment shader

	mTriVertexGrp.Generate(sizeof(Triangle) / 3, tri_shader);
	mLineVertexGrp.Generate(sizeof(Line), line_shader);



	mCameraUBO.Generate(sizeof(GizmosCameraData));
	mCameraUBO.Bind(0);



	
		//New Line Segment
	std::vector<GPUVertexAttribute> line_vertex_attributes = {
		{0, 3, GL_FLOAT, false, offsetof(Line, from)},
		///using Colour (uint32_t underlying type)
		/// size -> 4 (channel count) 4 
		/// type -> unsigned bytes (uint8_t) * 4 -> uint32_t 
		/// normalise true 0 - 255 -> 0.0 - 1.0f
		{1, 4, GL_UNSIGNED_BYTE, true, offsetof(Line, fromColour)},
		{2, 3, GL_FLOAT, false, offsetof(Line, to)},
		{3, 4, GL_UNSIGNED_BYTE, true, offsetof(Line, toColour)},
		{4, 1, GL_FLOAT, false, offsetof(Line, thickness)}
	};
	
	std::vector<GPUVertexAttributeDivisor> line_vertex_attribute_divisors =
	{
		{0, 1},
		{1, 1},
		{2, 1},
		{3, 1},
		{4, 1},
	};

	//New Triangle Segment
	std::vector<GPUVertexAttribute> tri_vertex_attributes = {
	{0, 3, GL_FLOAT, false, offsetof(VertexData, position)},
	{1, 3, GL_FLOAT, false, offsetof(VertexData, normal)},
	{2, 4, GL_UNSIGNED_BYTE, true, offsetof(VertexData, colour)},
	};
	

	mLineVertexGrp.BindLayout(
		line_vertex_attributes.data(), 
		line_vertex_attributes.size(), 
		line_vertex_attribute_divisors.data(),
		line_vertex_attribute_divisors.size());

	mTriVertexGrp.BindLayout(
		tri_vertex_attributes.data(),
		tri_vertex_attributes.size(),
		nullptr, 0);


    return success;
}


void DebugGizmosRendererImpl::DrawLine(const vx::Vec3& v0, const vx::Vec3& v1, const vx::Colour& colour)
{
	if (!this)
	{
		VX_LOG_WARN("Failed to use debug gizmos renderer, memory error.");
		return;
	}

	Line line = {
	v0.ToFloat3(), colour,
	v1.ToFloat3(), colour,
	mLineWidth
	};

	mLineVertexGrp.bBufferDirty = true;
	mLineBatches.push_back(line);
}

void DebugGizmosRendererImpl::DrawWireTriangle(const vx::Vec3& v1, const vx::Vec3& v2, const vx::Vec3& v3, const vx::Colour& colour)
{
	if (!this)
	{
		VX_LOG_WARN("Failed to use debug gizmos renderer, memory error.");
		return;
	}
	DrawLine(v1, v2, colour);
	DrawLine(v1, v3, colour);
	DrawLine(v2, v3, colour);
}

void DebugGizmosRendererImpl::DrawSolidTriangle(const vx::Vec3& v0, const vx::Vec3& v1, const vx::Vec3& v2, const vx::Colour& colour)
{
	if (!this)
	{
		VX_LOG_WARN("Failed to use debug gizmos renderer, memory error.");
		return;
	}

	vx::Float3 nor = ComputeTriangleNormals(v0, v1, v2).ToFloat3();
	Triangle tri = {
		{v0.ToFloat3(), nor, colour},
		{v1.ToFloat3(), nor, colour},
		{v2.ToFloat3(), nor, colour},
	};
	mTriVertexGrp.bBufferDirty = true;
	mTriangleBatches.push_back(tri);
}

void DebugGizmosRendererImpl::DrawSolidWireTriangle(const vx::Vec3& v0, const vx::Vec3& v1, const vx::Vec3& v2, const vx::Colour& colour)
{
	DrawSolidTriangle(v0, v1, v2, colour);
	DrawWireTriangle(v0, v1, v2, colour);
}

void DebugGizmosRendererImpl::DrawWireSphereDiscs(const vx::Vec3& center, float radius, const vx::Colour col, int segments)
{
	DrawWireDisc(center, radius, vx::kVxTau, col, vx::Vec3::Right(), vx::Vec3::Up(), segments);
	DrawWireDisc(center, radius, vx::kVxTau, col, vx::Vec3::Forward(), vx::Vec3::Right(), segments);
	DrawWireDisc(center, radius, vx::kVxTau, col, vx::Vec3::Up(), vx::Vec3::Forward(), segments);
}

void DebugGizmosRendererImpl::DrawWireDisc(const vx::Vec3& center, float radius, const vx::Colour col, const vx::Vec3& plane_axis_x, const vx::Vec3& plane_axis_y, int segments)
{
	DrawWireDisc(center, radius, vx::kVxTau, col, plane_axis_x, plane_axis_y, segments);
}

void DebugGizmosRendererImpl::DrawHalfWireDisc(const vx::Vec3& center, float radius, const vx::Colour col, const vx::Vec3& plane_axis_x, const vx::Vec3& plane_axis_y, int segments)
{
	DrawWireDisc(center, radius, vx::kVxPi, col, plane_axis_x, plane_axis_y, segments);
}

void DebugGizmosRendererImpl::DrawWireDisc(const vx::Vec3& center, float radius, float ratio, const vx::Colour col, const vx::Vec3& plane_axis_x, const vx::Vec3& plane_axis_y, int segments)
{
	if (!this)
	{
		VX_LOG_WARN("Failed to use debug gizmos renderer, memory error.");
		return;
	}
	float step = ratio / segments;
	vx::Vec3 prev = center + plane_axis_x * radius;

	vx::Vec3 x = plane_axis_x.Normalised();
	vx::Vec3 y = plane_axis_y.Reject(x).Normalised();
	//y.Normalise();

	//for (int i = 1; i <= segments; ++i)
	//{
	//	float angle = i * step;
	//	vx::Vec3 curr = center +
	//		(x * vx::VxCos(angle) +
	//			y * vx::VxSin(angle)) *
	//		radius;
	//	DrawLine(prev, curr, col);
	//	prev = curr;
	//}

	float c = vx::VxCos(step);
	float s = vx::VxSin(step);

	vx::Vec2 dir{ 1.0f, 0.0f };
	for (int i = 1; i <= segments; ++i)
	{
		dir.Rotate(c, s);
		vx::Vec3 curr = center + (x * (dir.X() * radius) +
								  y * (dir.Y() * radius));
		DrawLine(prev, curr, col);
		prev = curr;
	}

}

void DebugGizmosRendererImpl::DrawArrow(const vx::Vec3& v0, const vx::Vec3& v1, const vx::Vec3& base_axis, float width, float height, vx::Colour col)
{
	if (!this)
	{
		VX_LOG_WARN("Failed to use debug gizmos renderer, memory error.");
		return;
	}

	DrawArrowCone(v0, v1, width, height, width, 12, col);
	//DrawCone(v0, v1, width, 12, col);
	return;
	//line 
	DrawLine(v0, v1, col);

	VX_ASSERT_WARN(base_axis.IsNormalised(), "Base axis needs to be normalised");

	vx::Vec3 dir = (v0 - v1).Normalised();

	vx::Vec3 n0 = dir.Cross(base_axis).Normalised();

	//arrow head base
	vx::Vec3 base = v0 - (dir * height);

	//local points	
	DrawLine(v0, base + n0 * width, col);
	DrawLine(v0, base - n0 * width, col);

	//DrawLine(base + n0 * width, base - n0 * width, col);
	DrawLine(v0 + n0 * width, v0 - n0 * width, col);
}

void DebugGizmosRendererImpl::DrawCone(const vx::Vec3& apex, const vx::Vec3& base_center, float radius, int segments, vx::Colour col)
{
	vx::Vec3 axis = (apex - base_center).Normalised();

	vx::Vec3 tangent = axis.NormalisedPerpendicular();
	vx::Vec3 bitangent = axis.Cross(tangent).Normalised();

	float angle_step = vx::kVxTau / segments;
	//std::vector<vx::Vec3> base_ring;
	//base_ring.reserve(segments);

	//for (int i = 0; i < segments; ++i)
	//{
	//	float a = i * angle_step;
	//	vx::Vec3 p = base_center + (tangent * vx::VxCos(a) + bitangent * vx::VxSin(a)) * radius;
	//	base_ring.push_back(p);
	//}
	//for (int i = 0; i < segments; ++i)
	//{
	//	int nxt = (i + 1) % segments;
	//	DrawSolidTriangle(apex, base_ring[i], base_ring[nxt], col);
	//	DrawSolidTriangle(apex, base_ring[nxt], base_ring[i], col);
	//}

	float c = vx::VxCos(angle_step);
	float s = vx::VxSin(angle_step);
	vx::Vec2 dir = { 1.0f, 0.0f };
	vx::Vec3 prev = base_center + (tangent * dir.X() + bitangent * dir.Y()) * radius;
	for (int i = 1; i <= segments; ++i)
	{
		dir.Rotate(c, s);
		vx::Vec3 curr = base_center + (tangent * dir.X() + bitangent * dir.Y()) * radius;
		DrawSolidTriangle(apex, prev, curr, col);
		DrawSolidTriangle(apex, curr, prev, col);
		prev = curr;
	}
}

void DebugGizmosRendererImpl::DrawArrowCone(const vx::Vec3& start, const vx::Vec3& end, float shaft_radius, float head_height, float head_radius, int cone_segment, vx::Colour col)
{
	//shadft
	DrawLine(start, end, col);
	//cone base
	vx::Vec3 dir = (end - start).Normalised();
	vx::Vec3 cone_base = end - dir * head_height;
	DrawCone(end, cone_base, head_radius, cone_segment, col);
}

void DebugGizmosRendererImpl::DrawBox(const std::array<vx::Vec3, 8>& corners, const vx::Colour& col, bool wireframe)
{
	if(wireframe)
	{
		static const std::array<uint32_t, 24> edge_indices = {
			0, 1, 1, 2, 2, 3, 3, 0,
			4, 5, 5, 6, 6, 7, 7, 4,
			0, 4, 1, 5, 2, 6, 3, 7
		};

		for (size_t i = 0; i < edge_indices.size(); i += 2)
			DrawLine(corners[edge_indices[i]], corners[edge_indices[i + 1]], col);
		return;
	}

	static const uint32_t faces[6][4] =
	{
		{0, 3, 2, 1},
		{4, 5, 6, 7},
		{0, 1, 5, 4},
		{3, 7, 6, 2},
		{0, 4, 7, 3},
		{1, 2, 6, 5}
	};

	for (int f = 0; f < 6; ++f)
	{
		const uint32_t* v = faces[f];
		DrawSolidTriangle(corners[v[0]], corners[v[1]], corners[v[2]], col);
		DrawSolidTriangle(corners[v[0]], corners[v[2]], corners[v[3]], col);
	}
}

void DebugGizmosRendererImpl::DrawAABB(const vx::Vec3& min, const vx::Vec3& max, const vx::Colour& col, bool wireframe)
{
	const std::array<vx::Vec3, 8> corners = { 
		vx::Vec3(min.X(), min.Y(), min.Z()),
		vx::Vec3(max.X(), min.Y(), min.Z()),
		vx::Vec3(max.X(), max.Y(), min.Z()),
		vx::Vec3(min.X(), max.Y(), min.Z()),
		vx::Vec3(min.X(), min.Y(), max.Z()),
		vx::Vec3(max.X(), min.Y(), max.Z()),
		vx::Vec3(max.X(), max.Y(), max.Z()),
		vx::Vec3(min.X(), max.Y(), max.Z())
	};

	DrawBox(corners, col, wireframe);
}

void DebugGizmosRendererImpl::DrawAABB(const vx::AABB& aabb, const vx::Colour& col, bool wireframe)
{
	DrawAABB(aabb.mMin, aabb.mMax, col, wireframe);
}

template<size_t Sector, size_t Stack, bool Wireframe>
void DebugGizmosRendererImpl::DrawSphere(const vx::Vec3& center, float radius, vx::Colour col)
{
	constexpr size_t sector_count = Sector;
	constexpr size_t stack_count = Stack;

	///adapted Songho.ca sphere: https://www.songho.ca/opengl/gl_sphere.html
	constexpr float sector_step = vx::kVxTau / float(sector_count);
	constexpr float stack_step = vx::kVxPi / float(stack_count);

	float sector_angle, stack_angle;

	std::array<vx::Vec3, (sector_count + 1) * (stack_count + 1)> vertices;
	size_t vert_idx = 0;

	for (size_t i = 0; i <= stack_count; ++i)
	{
		stack_angle = vx::kVxPi / 2 - i * stack_step;        // starting from pi/2 to -pi/2
		float xy = radius * vx::VxCos(stack_angle);             // r * cos(u)
		float z = radius * vx::VxSin(stack_angle);              // r * sin(u)

		// add (sectorCount+1) vertices per stack
		// first and last vertices have same position and normal, but different tex coords
		for (size_t j = 0; j <= sector_count; ++j)
		{
			sector_angle = j * sector_step;           // starting from 0 to 2pi

			// vertex position (x, y, z)
			float x = xy * vx::VxCos(sector_angle);             // r * cos(u) * cos(v)
			float y = xy * vx::VxSin(sector_angle);             // r * cos(u) * sin(v)

			vertices[vert_idx++] = center + vx::Vec3(x, y, z);
		}
	}



	for (int i = 0; i < stack_count; ++i)
	{
		int k1 = i * (sector_count + 1);     // beginning of current stack
		int k2 = k1 + sector_count + 1;      // beginning of next stack

		for (int j = 0; j < sector_count; ++j, ++k1, ++k2)
		{
			vx::Vec3 v0 = vertices[k1];
			vx::Vec3 v1 = vertices[k1 + 1];
			vx::Vec3 v2 = vertices[k2 + 1];
			vx::Vec3 v3 = vertices[k2];
			
			if constexpr (Wireframe)
			{
				DrawWireTriangle(v0, v2, v1, col);
				DrawWireTriangle(v0, v3, v2, col);
			}
			else
			{
				DrawSolidTriangle(v0, v2, v1, col);
				DrawSolidTriangle(v0, v3, v2, col);
			}
		}
	}
}

template void DebugGizmosRendererImpl::DrawSphere<8, 6, false>(const Vec3&, float, vx::Colour);
template void DebugGizmosRendererImpl::DrawSphere<8, 6, true>(const Vec3&, float, vx::Colour);
template void DebugGizmosRendererImpl::DrawSphere<16, 12, false>(const Vec3&, float, vx::Colour);
template void DebugGizmosRendererImpl::DrawSphere<4, 4, false>(const Vec3&, float, vx::Colour);

//void DebugGizmosRendererImpl::DrawAABB(const vx::AABB& aabb, const vx::Colour& col, bool wireframe)
//{
//}

void DebugGizmosRendererImpl::DrawAACross(const vx::Vec3& pos, const Colour* colour, int colour_count, float scale)
{
	DrawCross(pos, vx::Vec3::Right(), vx::Vec3::Up(), vx::Vec3::Forward(), colour, colour_count, scale);
}

void DebugGizmosRendererImpl::DrawCross(const vx::Vec3& pos, const vx::Vec3& rt, const vx::Vec3& up, const vx::Vec3& fwd, const vx::Colour* colour, int colour_count, float scale)
{
	int col_count = (colour) ? colour_count : 0;
	vx::Colour cols[3] =
	{
		(col_count > 0) ? (--col_count, colour[0]) : vx::Colour::sMagenta,
		(col_count > 0) ? (--col_count, colour[1]) : cols[0],
		(col_count > 0) ? (--col_count, colour[2]) : cols[1]// vx::Colour::sMagenta
	};

	//scale disp
	vx::Vec3 drt = rt * scale;
	vx::Vec3 dup = up * scale;
	vx::Vec3 dfwd = fwd * scale;

	DrawLine(pos - drt, pos+drt, cols[0]);
	DrawLine(pos - dup, pos+dup, cols[1]);
	DrawLine(pos - dfwd, pos+dfwd, cols[2]);
}

void DebugGizmosRendererImpl::DrawBasis(const vx::Vec3& pos, const vx::Vec3& rt, const vx::Vec3& up,
	const vx::Vec3& fwd, float arrow_width, float arrow_height, bool b_draw_plane, uint32_t cone_segment)
{
	//DrawArrow(pos + rt, pos, up, arrow_width, arrow_height, vx::Colour::sRed);
	//DrawArrow(pos + up, pos, fwd, arrow_width, arrow_height, vx::Colour::sGreen);
	//DrawArrow(pos + fwd, pos, rt, arrow_width, arrow_height, vx::Colour::sBlue);

	DrawArrowCone(pos, pos + rt, arrow_width, arrow_height, arrow_width, cone_segment, vx::Colour::sRed);
	DrawArrowCone(pos, pos + up, arrow_width, arrow_height, arrow_width, cone_segment, vx::Colour::sGreen);
	DrawArrowCone(pos, pos + fwd, arrow_width, arrow_height, arrow_width, cone_segment, vx::Colour::sBlue);

	if(b_draw_plane)
	{
		const float one_third = 1.0f / 3.0f;
		vx::Colour colours[3] = { vx::Colour::sRed, vx::Colour::sGreen, vx::Colour::sBlue };
		vx::Vec3 basis[3] = { rt, up, fwd };

		for (uint32_t i = 0; i < 3; ++i)
		{
			//local space
			vx::Vec3 pt0 = basis[i] * one_third;
			vx::Vec3 pt1 = basis[(i + 1) % 3] * one_third;

			vx::Vec3 peak = pos + pt0 + pt1;

			//transform
			pt0 += pos;
			pt1 += pos;

			DrawLine(pt0, peak, colours[i]);
			DrawLine(pt1, peak, colours[(i + 1) % 3]);
		}
	}
}

void DebugGizmosRendererImpl::DrawBasis(const vx::Mat44 transform, float arrow_width, float arrow_height, bool b_draw_plane, uint32_t cone_segment)
{
	DrawBasis(transform.GetTranslation(), transform.GetAxisX(), transform.GetAxisY(), transform.GetAxisZ(), arrow_width, arrow_height, b_draw_plane, cone_segment);
}

//template<template T>
template<DebugGizmosRendererImpl::EVertexPrimitiveMode T>
void DebugGizmosRendererImpl::Upload(VertexGroup<T>& target_vertex_buff, uint32_t vertices_per_shape, size_t buffer_shape_count, const void* buffer_data)
{
	//GLCall(glBindVertexArray(target_vertex_buff.VAO));
	//GLCall(glBindBuffer(GL_ARRAY_BUFFER, target_vertex_buff.VBO));

	uint32_t total_vertices = static_cast<uint32_t>(buffer_shape_count * vertices_per_shape);
	GLsizeiptr bytes_needed = total_vertices * target_vertex_buff.stride;
	if (target_vertex_buff.maxVertex < total_vertices)
	{
		target_vertex_buff.maxVertex = total_vertices;
		//GLCall(glBufferData(GL_ARRAY_BUFFER, bytes_needed, nullptr, GL_DYNAMIC_DRAW));
		GLCall(glNamedBufferData(target_vertex_buff.VBO, bytes_needed, nullptr, GL_DYNAMIC_DRAW));
	}

	if (total_vertices > 0)
		//GLCall(glBufferSubData(GL_ARRAY_BUFFER, 0, bytes_needed, buffer_data));
		GLCall(glNamedBufferSubData(target_vertex_buff.VBO, 0, bytes_needed, buffer_data));

	target_vertex_buff.bBufferDirty = false;

}


void DebugGizmosRendererImpl::UploadIfDirty()
{
	if (mLineVertexGrp.bBufferDirty) Upload(mLineVertexGrp, 1, mLineBatches.size(), mLineBatches.data());
	if (mTriVertexGrp.bBufferDirty) Upload(mTriVertexGrp, 3, mTriangleBatches.size(), mTriangleBatches.data());
}

void DebugGizmosRendererImpl::ExecuteDraws()
{
	VX_VARIABLE_PROFILE_FUNCTION();
	if (mLineBatches.size() <= 0 && mTriangleBatches.size() <= 0)
		return;

	EndCurrentDrawCommand();
	UploadIfDirty();

	//mShader->Bind();
	GLCall(glBindVertexArray(mLineVertexGrp.VAO));
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, mLineVertexGrp.VBO));
	//bind this uniform buffer to slot, as other system might have bind to 
	//require slot 
	mCameraUBO.Bind(0);

	for (const auto& draw_cmd : mDrawCommands)
		InternalExecuteDraw(draw_cmd);

	mLineBatches.clear();
	mTriangleBatches.clear();
	mDrawCommands.clear();

	//unbind any framebuffer (Reset)
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glBindVertexArray(0);
	//unbind shader
	glUseProgram(0);
}

DrawCommand* DebugGizmosRendererImpl::PushDrawCommand(DrawCommand cmd)
{
	EndCurrentDrawCommand();

	cmd.lineBuffRange.start = mLineBatches.size();
	cmd.triBuffRange.start = mTriangleBatches.size();
	mDrawCommands.push_back(cmd);
	return &mDrawCommands.back();
}

DrawCommand* DebugGizmosRendererImpl::PushDrawCommand(vx::Mat44 proj, vx::Mat44 view, IRenderTarget* render_target)
{
	return PushDrawCommand({ proj, view, 0, 0, 0, 0, render_target });
}

bool DebugGizmosRendererImpl::EndCurrentDrawCommand()
{
	if (mDrawCommands.empty()) return false;

	auto& last_cmd = mDrawCommands.back();

	last_cmd.lineBuffRange.count = mLineBatches.size() - last_cmd.lineBuffRange.start;
	last_cmd.triBuffRange.count = mTriangleBatches.size() - last_cmd.triBuffRange.start;

	return true;
}

void DebugGizmosRendererImpl::RemoveDrawCommand(DrawCommand& cmd)
{
	const DrawCommand* _addr = &cmd;
	VX_ASSERT(_addr >= mDrawCommands.data() && _addr < mDrawCommands.data() + mDrawCommands.size());

	//has address
	std::swap(cmd, mDrawCommands.back());
	mDrawCommands.pop_back();
}

bool DebugGizmosRendererImpl::ExecuteDraw(IRenderTarget* render_target)
{
	DrawCommand* cmd = nullptr;

	int cmd_idx = -1;
	for (auto it = mDrawCommands.begin(); it != mDrawCommands.end(); ++it)
	{
		cmd_idx++;
		if (it->renderTarget == render_target)
		{
			cmd = it._Ptr;
			break;
		}
	}

	return (cmd) ? InternalExecuteDraw(*cmd) : false;
}

bool DebugGizmosRendererImpl::ExecuteDraw(const DrawCommand& cmd)
{

	UploadIfDirty();
	//mShader->Bind();
	GLCall(glBindVertexArray(mLineVertexGrp.VAO));
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, mLineVertexGrp.VBO));

	mCameraUBO.Bind(0);

	bool status = InternalExecuteDraw(cmd);
	//unbind any framebuffer (Reset)
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glBindVertexArray(0);
	//unbind shader
	glUseProgram(0);
	return status;
}

bool DebugGizmosRendererImpl::InternalExecuteDraw(const DrawCommand& cmd)
{
	if (mLineBatches.size() <= 0 && mTriangleBatches.size() <= 0) return false;

	int first_line_vertex = static_cast<int>(cmd.lineBuffRange.start);
	int line_vertex_count = static_cast<int>(cmd.lineBuffRange.count);
	int first_tri_vertex = static_cast<int>(cmd.triBuffRange.start * 3);
	int tri_vertex_count = static_cast<int>(cmd.triBuffRange.count * 3);

	if (line_vertex_count == 0 && tri_vertex_count == 0) return true; //no fail, just empty

	mCameraUBO.SetBufferData(&cmd.cameraData);
	//mShader->Bind();
	//mShader->SetUniformMat4("uProj", cmd.cameraData.proj);
	//mShader->SetUniformMat4("uView", cmd.cameraData.view);

	vx::Vec2 view_resolution;
	if (cmd.renderTarget)
	{
		cmd.renderTarget->Bind();
		//use main attachment size
		auto main_att = cmd.renderTarget->GetAttachment(0);
		VX_ASSERT(main_att != nullptr);
		view_resolution = vx::Vec2(main_att->Width(), main_att->Height());
	}
	else
	{
		VX_ASSERT(mActiveWindow != nullptr);
		view_resolution = vx::Vec2(mActiveWindow->Width(), mActiveWindow->Height());
		GLCall(glBindFramebuffer(GL_FRAMEBUFFER, 0));
	}

	if (line_vertex_count != 0)
	{
		mLineVertexGrp.shader->Bind();
		mLineVertexGrp.Bind();
		mLineVertexGrp.shader->SetUniformVec2("uResolution", view_resolution);

		//glDrawArrays(GL_LINES, first_line_vertex, line_vertex_count);
		mLineVertexGrp.DrawArrayInstancedBase(first_line_vertex,
			mLineVertexGrp.GetVertexCountDivisor() - 1, line_vertex_count);
	}

	//hack for now rare to draw triangle
	if (tri_vertex_count != 0)
	{
		//glDisable(GL_CULL_FACE);
		//mShader->SetUniform1i("uHasNormal", bTriFrameHasNor);
		mTriVertexGrp.shader->Bind();
		mTriVertexGrp.Bind();
		//glDrawArrays(GL_TRIANGLES, first_tri_vertex, tri_vertex_count);
		mTriVertexGrp.DrawArray(first_tri_vertex, tri_vertex_count);
		mLineVertexGrp.shader->Bind();
		mLineVertexGrp.Bind();
		//mShader->SetUniform1i("uHasNormal", false);
		//glEnable(GL_CULL_FACE);
	}

	return true;
}


bool DebugGizmosRendererImpl::Flush(IRenderTarget* render_target)
{
	DrawCommand* cmd = nullptr;
	int cmd_idx = 0;

	for (auto it = mDrawCommands.begin(); it != mDrawCommands.end(); ++it, ++cmd_idx)
		if (it->renderTarget == render_target)
		{
			cmd = &(*it);
			break;
		}

	if (!cmd) return false;

	UploadIfDirty();
	//mShader->Bind();
	GLCall(glBindVertexArray(mLineVertexGrp.VAO));
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, mLineVertexGrp.VBO));

	mCameraUBO.Bind(0);
	InternalExecuteDraw(*cmd);
	//remove
	RemoveDrawCommand(mDrawCommands[cmd_idx]);

	//unbind any framebuffer (Reset)
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glBindVertexArray(0);
	//unbind shader
	glUseProgram(0);

	return true;
}

