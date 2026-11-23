#include "EditorImgui.h"

#include <GL/glew.h>
#include <glfw/glfw3.h>

#include <external/imgui/imgui.h>
#include <external/imgui/imgui_impl_glfw.h>
#include <external/imgui/imgui_impl_opengl3.h>

#include <SampleFramework/SampleFramework.h>
#include "Vortrix/Particles/Particle.h"
#include "Vortrix/Dynamics/Body/Body.h"

#include "Vortrix/Collision/Shapes/Shape.h"

#include "Renderer/Texture.h"

#include "Vortrix/Dynamics/Body/BodyManager.h"
#include "Vortrix/PhysicsWorld.h"

#include "Vortrix/Dynamics/Constraints/DistanceConstraint.h"
#include "Vortrix/Dynamics/Constraints/PointConstraint.h"
//
//template<typename Enum>
//extern bool UICombo(const char* label, Enum& value, const char* items_separated_by_zeros, int height_in_items = -1)
//{
//
//}

vx::Colour EditorImGui::kAccent = vx::Colour(0.3f, 0.75f, 0.8f, 1.0f);
vx::Colour EditorImGui::kSecondaryAccent = vx::Colour(.102f, .42f, .478f, 1.0f);
ImGuiTreeNodeFlags EditorImGui::kTreeNodeFlags = ImGuiTreeNodeFlags_Framed;


EditorImGui::ScopePushItemWidth::ScopePushItemWidth(float width)
{
	ImGui::PushItemWidth(width);
}

EditorImGui::ScopePushItemWidth::~ScopePushItemWidth()
{
	ImGui::PopItemWidth();
}


bool EditorImGui::SliderInt(const char* label, int* v, int v_min, int v_max, const char* format, ImGuiSliderFlags flags)
{
	ScopePushItemWidth size(kSliderWidth);
	return ImGui::SliderScalar(label, ImGuiDataType_S32, v, &v_min, &v_max, format, flags);
}

bool EditorImGui::SliderFloat(const char* label, float* v, float v_min, float v_max, const char* format, ImGuiSliderFlags flags)
{
	ScopePushItemWidth size(kSliderWidth);
	return ImGui::SliderScalar(label, ImGuiDataType_Float, v, &v_min, &v_max, format, flags);
}

bool EditorImGui::DragFloat(const char* label, float* v, float v_speed, float v_min, float v_max, const char* format, ImGuiSliderFlags flags)
{
	ScopePushItemWidth size(kSliderWidth);
	return ImGui::DragScalar(label, ImGuiDataType_Float, v, v_speed, &v_min, &v_max, format, flags);
}

bool EditorImGui::DragInt(const char* label, int* v, float v_speed, int v_min, int v_max, const char* format, ImGuiSliderFlags flags)
{
	ScopePushItemWidth size(kSliderWidth);
	return ImGui::DragScalar(label, ImGuiDataType_S32, v, v_speed, &v_min, &v_max, format, flags);
}

void EditorImGui::TextColoured(const Colour col, const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vx::Vec4 c = col;
	ImGui::TextColoredV(ImVec4(c[0], c[1], c[2], c[3]), fmt, args);
	va_end(args);
}

void EditorImGui::TextColourWrapped(const vx::Colour col, const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	ImGui::PushStyleColor(ImGuiCol_Text, col);
	ImGui::TextWrappedV(fmt, args);
	ImGui::PopStyleColor();
	va_end(args);
}

void EditorImGui::ResolvePanelPosSize(float target_width, float target_height, vx::Vec2& o_pos, vx::Vec2& o_size, 
	EPanelScreenAnchor screen_anchor, vx::Vec2 max_ratio_screen)
{
	VX_ASSERT(!mFrameWorkableScreenSize.IsZero());

	float frame_screen_width = mFrameWorkableScreenSize[0];
	float frame_screen_height = mFrameWorkableScreenSize[1];


	vx::Vec2 workable_pos_origin = vx::Vec2(0.0f, mFrameMenuItemY);//menu item y size 25
	vx::Vec2 workable_win_size = vx::Vec2(frame_screen_width, frame_screen_height - mFrameMenuItemY);//menu item y size 25

	//top left 
	float screen_padding = 5.0f;

	///test remove side paddingd 
	workable_win_size -= vx::Vec2(2 * screen_padding);

	max_ratio_screen = vx::Vec2::Clamp(max_ratio_screen, vx::Vec2::Zero(), vx::Vec2::One());

	float panel_width = vx::VxMin(target_width, workable_win_size[0] * max_ratio_screen[0]);

	float panel_height = vx::VxMin(target_height, workable_win_size[1] * max_ratio_screen[1]);

	vx::Vec2 panel_size(panel_width, panel_height);;




	vx::Vec2 pos_offset;
	/// top right consider primary simulation panel
	/// use the approraite corrd 
	/// for this instamce top right 
	/// then consider the appropriate panel 
	vx::Vec2 target_panel_pos(1550.0f, 35.0f);//its top left
	//pos_offset[0] -= vx::VxAbs(target_panel_pos[0] - frame_screen_width);

	const bool consider_target_panel = true;

	///for now, target panel is to the right (primary panel)
	if (consider_target_panel)
	{
		frame_screen_width = vx::VxMin(target_panel_pos[0], frame_screen_width);

	}

	switch (screen_anchor)
	{
	case EPanelScreenAnchor::TopLeft:
		pos_offset = vx::Vec2(screen_padding);
		break;
	case EPanelScreenAnchor::BottomLeft:
	{
		const float disp_alongY = (frame_screen_height - mFrameMenuItemY) +// move point downward along Y & consider the menu height size
			-(screen_padding + panel_size[1]);///take into account the bottom padding and panel size
		pos_offset = vx::Vec2(screen_padding, disp_alongY);
	}
	break;
	case EPanelScreenAnchor::TopRight:
	{
		float disp_alongX = (frame_screen_width)+ // move point downward along X
			-(screen_padding + panel_size[0]);///take into account the bottom padding and panel size
		pos_offset = vx::Vec2(disp_alongX, screen_padding);
	}
	break;
	case EPanelScreenAnchor::BottomRight:
		vx::Vec2 disp = vx::Vec2(frame_screen_width, frame_screen_height - mFrameMenuItemY) - (vx::Vec2(screen_padding) + panel_size);
		pos_offset = disp;
		break;
	}

	vx::Vec2 panel_pos = workable_pos_origin + pos_offset;

	o_pos = panel_pos;
	o_size = panel_size;
}

bool EditorImGui::BeginPanel(const char* name, bool* p_open, ImGuiWindowFlags flags)
{
	return ImGui::Begin(name, p_open, flags);
}

bool EditorImGui::BeginPanel(const char* name, bool* p_open, float target_width, float target_height,
	EPanelScreenAnchor screen_anchor, vx::Vec2 max_ratio_screen, ImGuiWindowFlags flags)
{

	vx::Vec2 pos;
	vx::Vec2 size;
	ResolvePanelPosSize(target_width, target_height, pos, size, screen_anchor, max_ratio_screen);
	ImGui::SetNextWindowPos(ImVec2(pos[0], pos[1]), ImGuiCond_Appearing);
	ImGui::SetNextWindowSize(ImVec2(size[0], size[1]), ImGuiCond_Appearing);

	return ImGui::Begin(name, p_open, flags);
}

void EditorImGui::EndPanel()
{
	ImGui::End();
}

void EditorImGui::Initialise(GLFWwindow* glfw_win)
{
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;


	//ImGui Renderer
	ImGui_ImplGlfw_InitForOpenGL(glfw_win, true);
	ImGui_ImplOpenGL3_Init("#version 400");

	io.Fonts->AddFontFromFileTTF("assets/fonts/Inter_18pt-Regular.ttf", 18.0f);


	ImGuiStyle* style = &ImGui::GetStyle();
	style->WindowRounding = 4.0f;
	style->FrameRounding = 5.0f;
	style->GrabRounding = 6.0f;

	//style->ScaleAllSizes()

	////Style 
	style->WindowPadding = ImVec2(10.0f, 10.0f);
	style->FramePadding = ImVec2(8.0f, 4.0f);
	style->CellPadding = ImVec2(6.0f, 4.0f);
	style->ItemSpacing = ImVec2(8.0f, 7.0f);
	style->ItemInnerSpacing = ImVec2(7.0f, 4.0f);
	style->IndentSpacing = 18.0f;
	style->ScrollbarSize = 12.0f;
	style->GrabMinSize = 10.0f;

	style->WindowBorderSize = 1.0f;
	style->FrameBorderSize = 0.0f;
	style->PopupBorderSize = 1.0f;
	style->TabBorderSize = 0.0f;
	style->SeparatorTextBorderSize = 1.0f;

	style->WindowRounding = 4.0f;
	style->ChildRounding = 4.0f;
	style->PopupRounding = 4.0f;
	style->FrameRounding = 3.0f;
	style->GrabRounding = 3.0f;
	style->ScrollbarRounding = 3.0f;
	style->TabRounding = 3.0f;

	style->WindowTitleAlign = ImVec2(0.0f, 0.5f);




	///COLOUR SCHEME
	ImVec4* colours = style->Colors;

	colours[ImGuiCol_WindowBg] = ImVec4(0.11f, 0.11f, 0.141f, 0.95);

	colours[ImGuiCol_Border] = ImVec4(0.0, 0.0, 0.0, 0.55);

	colours[ImGuiCol_FrameBg] = ImVec4(0.140f, 0.140f, 0.19f, 1.000f);
	colours[ImGuiCol_FrameBgHovered] = ImVec4(.2, 0.2, 0.24, 1.000f);
	colours[ImGuiCol_FrameBgActive] = ImVec4(.2, 0.28, 0.38, 1.0f);

	colours[ImGuiCol_CheckMark] = ImVec4(0.3, 0.75, 0.8, 1.0);

	colours[ImGuiCol_Button] = ImVec4(0.22f, 0.23f, 0.30f, 1.0f);
	colours[ImGuiCol_ButtonHovered] = ImVec4(0.28f, 0.3f, 0.34f, 1.0f);
	colours[ImGuiCol_ButtonActive] = ImVec4(0.0, 0.302f, 0.349f, 0.4f);


	//colours[ImGuiCol_Header] = ImVec4(0.0, 0.3, 0.35, 0.4);
	colours[ImGuiCol_Header] = ImVec4(0.24, 0.25, 0.35, 0.4);
	colours[ImGuiCol_HeaderHovered] = ImVec4(.3, .32, .45, .8);
	colours[ImGuiCol_HeaderActive] = ImVec4(.28, .38, .52, 1.0);


	//colours[ImGuiCol_Tab] = ImVec4(0.0, 0.302, 0.349, 0.4);
	colours[ImGuiCol_Tab] = ImVec4(0.16, .17, .2, 1.);
	//colours[ImGuiCol_TabHovered] = ImVec4(0.251, 0.702, 0.702, 1.0);
	colours[ImGuiCol_TabHovered] = ImVec4(.24, .26, .3, 1.0);
	//colours[ImGuiCol_TabActive] = ImVec4(0.1, .42, 0.48, 1.0);
	colours[ImGuiCol_TabActive] = ImVec4(.102, .42, .478, 1.0);


	colours[ImGuiCol_SeparatorHovered] = ImVec4(0.25, 0.48, 0.62, 0.78);
	colours[ImGuiCol_SeparatorActive] = ImVec4(0.25, 0.48, 0.62, 1.0);

	colours[ImGuiCol_ResizeGrip] = ImVec4(0.25, 0.48, 0.62, 0.2);
	colours[ImGuiCol_ResizeGripHovered] = ImVec4(0.25, 0.48, 0.62, 0.67);
	colours[ImGuiCol_ResizeGripActive] = ImVec4(0.25, 0.48, 0.62, 0.95);

	colours[ImGuiCol_SliderGrab] = ImVec4(.102, .42, .478, 1.0);
	colours[ImGuiCol_SliderGrabActive] = ImVec4(0.302, .749, 0.8, 1.0);

	colours[ImGuiCol_TitleBg] = ImVec4(0.11f, 0.11f, 0.141f, 0.95);
	colours[ImGuiCol_TitleBgActive] = ImVec4(0.12f, 0.16f, 0.20f, 0.95);
	colours[ImGuiCol_TitleBgCollapsed] = ImVec4(0.11f, 0.11f, 0.141f, 0.95);

	colours[ImGuiCol_NavHighlight] = ImVec4(0.3, 0.75, 0.8, 1.0);

	
}


void EditorImGui::BeginNewFrame(float screen_width, float screen_height)
{
	mFrameWorkableScreenSize.SetXY(screen_width, screen_height);

	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}

void EditorImGui::RenderFrame()
{
	VX_VARIABLE_PROFILE_FUNCTION();
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void EditorImGui::Shutdown()
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}

bool EditorImGui::UIBlockingMouseInput() const
{
	ImGuiIO& io = ImGui::GetIO();
	return io.WantCaptureMouse;
}

bool EditorImGui::UIBlockingInput()
{
	ImGuiIO& io = ImGui::GetIO();
	return io.WantCaptureMouse || io.WantCaptureKeyboard;
}

bool EditorImGui::BeginMainMenu() const
{
	return ImGui::BeginMainMenuBar();
}

void EditorImGui::EndMainMenu()
{
	ImGui::EndMainMenuBar();
	ImVec2 size = ImGui::GetItemRectSize();
	mFrameMenuItemY = size.y;
}

void EditorImGui::DrawParticlesOverlayItems(std::vector<vx::Particles::Particle>& particles) const
{
	auto draw_particle_prop = [](vx::Particles::Particle& particle) {
		ImGui::DragFloat3("mPosition: ", &particle.mPosition[0], 0.1f);
		ImGui::SliderFloat("mDamping", &particle.mDamping, 0.0f, 1.0f);
		ImGui::SliderFloat("mMass", &particle.mMass, 0.0f, 200.0f, "%.1f");
		ImGui::SliderFloat("mRadius", &particle.mRadius, 0.0f, 1.0f);
		if (ImGui::TreeNode("Properties state"))
		{
			ImGui::Text("mAcceleration: %s", particle.mAcceleration.ToString().c_str());
			ImGui::Text("mAccumulatedForce: %s", particle.mAccumlatedForce.ToString().c_str());
			ImGui::Text("mVelocity: %s", particle.mVelocity.ToString().c_str());
			ImGui::TreePop();
		}
	};

	uint32 count = static_cast<uint32>(particles.size());
	for (size_t i = 0; i < count; ++i)
	{
		auto& p = particles[i];
		ImGui::PushID(&p);
		ImGui::SeparatorText(("Particle " + std::to_string(i)).c_str());
		draw_particle_prop(p);
		ImGui::PopID();
	}
}


/// bodyEulerAngle
	/// not using vec3 as its vector 4 underthehood
	/// hence 16 bytes where 4 bytes is waste 
	/// the 4 bytes could be used for flags etc
struct BodyEulerAngle
{
	union
	{
		struct { float pitch, yaw, roll; };
		float angles[3];
	};

	bool localSpace = true;
	bool bDirty = false;

	vx::Vec3 ToVec3()
	{
		return vx::Vec3(pitch, yaw, roll);
	}

	void FromVec3(const vx::Vec3& vec)
	{
		pitch = vec.X();
		yaw = vec.Y();
		roll = vec.Z();
	}

	//for ui 
	float& operator[](uint32_t i)
	{
		VX_ASSERT(i < 3, "out of bounds");
		return angles[i];
	}
	float const& operator[](uint32_t i) const
	{
		VX_ASSERT(i < 3, "out of bounds");
		return angles[i];
	}

};
template<vx::EMotionType Type>
void EditorImGui::DrawBodyOverlayDetailsImpl(vx::Body& body, vx::BodyDebug& body_debug_info)
{


	
	static std::vector<BodyEulerAngle> cache_body_euler;

	if (body.ID().Idx() >= cache_body_euler.size())
	{
		cache_body_euler.resize(body.ID().Idx() + 1);
		vx::Vec3 angle = vx::RadToDeg(body.Orientation().GetEulerAngles());
		cache_body_euler[body.ID().Idx()].FromVec3(angle);
	}

	auto& euler = cache_body_euler[body.ID().Idx()];

	Vec3 p = body.Position();
	ImGui::Text("Island Idx: %d", body.GetIslandIndex());
	ImGui::Text("Active Body Idx: %d", body.GetIndexInActiveBodies());
	if (ImGui::DragFloat3("Position ", &p[0], 0.01f))
	{
		///world -space
		if (!euler.localSpace)
			body.SetPosition(p);
		else
		{
			vx::Vec3 old_pos = body.Position();
			vx::Vec3 delta = p - old_pos;
			delta = body.Orientation().InverseRotate(delta);
			body.SetPosition(old_pos + delta);
			//ImGui::TextColored(ImVec4(1, 0, 0, 1), "This causes position ui output missmatch in Local space");
		}
	}

	if (body.IsAwake())
	{
		vx::Vec3 angle = vx::RadToDeg(body.Orientation().GetEulerAngles());
		euler.FromVec3(angle);
	}

	ImGui::Checkbox("Local Space", &euler.localSpace);
	BodyEulerAngle old_angle = euler;
	bool change = ImGui::DragFloat3("Euler", &euler[0], 0.25f);
	if (change)
	{
		vx::Vec3 angle_dt = euler.ToVec3() - old_angle.ToVec3();

		for (uint32_t i = 0; i < 3; ++i)
			angle_dt[i] = fmod(angle_dt[i] + 180.0f, 360.0f) - 180.0f;

		vx::Quat dq = vx::Quat::FromEulerAngle(vx::DegToRad(angle_dt));

		//local - space 
		if (euler.localSpace)
			body.SetOrientation((body.Orientation() * dq).Normalised());
		//world - space 
		else
			body.SetOrientation((dq * body.Orientation()).Normalised());
	}

	if constexpr (Type == EMotionType::Dynamic)
	{
		ImGui::SliderFloat("Linear Damping", &body.mLinearDamping, 0.0f, 1.0f);
		ImGui::SliderFloat("Angular Damping", &body.mAngularDamping, 0.0f, 1.0f);
		ImGui::DragFloat("Max Squared Linear Speed", &body.mMaxLinearVelocity);
		ImGui::DragFloat("Max Squared Angular Speed", &body.mMaxAngularVelocity);
	}

	ImGui::SliderFloat("Friction Coefficent", &body.mFriction, 0.0f, 1.0f);
	ImGui::SliderFloat("Restitution Coefficent", &body.mRestitution, 0.0f, 1.0f);

	if constexpr (Type == EMotionType::Dynamic)
	{
		if (ImGui::Button("Clear Forces", ImVec2(100, 0)))
			body.ClearAccumulatedForces();
		ImGui::SameLine();
		if (ImGui::Button("Clear Velocities", ImVec2(120, 0)))
			body.ClearVelocities();
		static vx::Vec3 wakeup_impluse = vx::Vec3::Forward() * 500;
		ImGui::DragFloat3("Wakeup Impluse (Nm)", &wakeup_impluse[0], 0.1);
		if (ImGui::Button("Wakeup", ImVec2(80, 0)))
			body.WakeUp(wakeup_impluse);
		static vx::Vec3 apply_force = vx::Vec3::Forward() * 10.0f;
		static vx::StackString<32> apply_force_button_label("Apply Force 10N");
		if (ImGui::DragFloat3("Force (N)", &apply_force[0], 0.1f))
		{
			apply_force_button_label.Clear();
			apply_force_button_label << "Apply Force " << apply_force.Length() << "N";
		}
		if(ImGui::Button(apply_force_button_label.Data()))
			body.AddForce(apply_force);
		EDynamicsDofs _dof = body.GetAllowedDynamicsDof();
		DrawAllowedDofFlags(_dof);
		body.SetAllowedDynamicsDof(_dof);
	}

	//ImGui::SliderFloat("mMass", &body.mMass, 0.0f, 200.0f, "%.1f");
	//ImGui::SliderFloat("mRadius", &body.mRadius, 0.0f, 1.0f);
	if (ImGui::TreeNode("Step phase"))
	{
		EditorImGui::DrawBodyFlags(body_debug_info.simulationStats.phase);
		ImGui::TreePop();
	}
	if (ImGui::TreeNode("Properties state"))
	{
		ImGui::Text("Body Type %s", (body.IsDynamic()) ? "Dynamic" : "Static");
		ImGui::Text("Awake: %s", (body.mAwake) ? "true" : "false");
		//ImGui::Text("Acceleration: %s", body.mAcceleration.ToString().c_str());
		vx::Vec3 body_orientation_euler = body.Orientation().GetEulerAngles();
		ImGui::SeparatorText("Transformation");
		ImGui::Text("Euler Angles: %s degrees", vx::RadToDeg(body_orientation_euler).ToString().c_str());
		ImGui::Text("UI Cache Euler Angles: %s degrees", euler.ToVec3().ToString().c_str());
		ImGui::Text("Orientation Quat: %s ", body.Orientation().ToString().c_str());

		ImGui::SeparatorText("Motion Dynamics");
		ImGui::Text("Linear Velocity: %s", body.mLinearVelocity.ToString().c_str());
		ImGui::Text("Linear Speed: %s m/s", std::to_string(body.mLinearVelocity.Length()).c_str());
		ImGui::Text("Angular Velocity: %s", body.mAngularVelocity.ToString().c_str());
		ImGui::Text("Angular Speed: %s rad/s", std::to_string(body.mAngularVelocity.Length()).c_str());
		ImGui::Text("Force Accumulated: %s", body.mForceAccumulated.ToString().c_str());
		ImGui::Text("Torque Accumulated: %s", body.mTorqueAccumulated.ToString().c_str());
		ImGui::Text("Sleep Timer: %f", body.mSleepTimer);
		auto& body_stat = body_debug_info.simulationStats;
		ImGui::Text("Max Attained Linear Speed Squared: %f(m/s)^2", body_stat.maxAttainedLinearVelocitySq);
		ImGui::Text("Max Angular Speed Squared: %f(rad/s)^2", body_stat.maxAttainedAngularVelocitySq);

		ImGui::SeparatorText("Mass Properties");
		ImGui::Text("Mass: %f kg.", body.Mass());
		ImGui::Text("Diagonal Inverse Inertia: %s", body.GetLocalInvInertiaDiagonal().ToString().c_str());
		ImGui::TreePop();
	}
	if (ImGui::TreeNode("Debug Shape Properies"))
	{
		auto shape = body.GetShape();
		ImGui::Text("Type: %s", shape->ShapeTypeName());
		ImGui::Text("Density: %f", shape->Density());
		switch (shape->Type())
		{
		case EShapeType::Sphere: ImGui::Text("Radius %f", shape->HalfExtents().X());
			break;
		case EShapeType::Box:
		case EShapeType::Capsule:
		case EShapeType::Plane:
			ImGui::Text("Half Size %s", shape->HalfExtents().ToString().c_str());
			break;
		default:
			break;
		}

		MassProperties mp = shape->GetMassProperties();
		ImGui::Text("Mass: %f", mp.mass);
		ImGui::Text("Inertia Tensor: %s", mp.inertialTensorDiagonal.ToString().c_str());
		if constexpr (Type == vx::EMotionType::Dynamic)
			ImGui::Text("Inv Inertia Tensor: %s", (Vec3::LoadFloat3Raw(mp.inertialTensorDiagonal)).Reciprocal().ToString().c_str());

		ImGui::TreePop();
	}
}

void EditorImGui::DrawBodyOverlayDetails(vx::BodyID body_id, vx::PhysicsWorld* physics_world)
{
	using Draw_Body_Detail_Func = void(EditorImGui::*)(vx::Body&, vx::BodyDebug&);
	static const Draw_Body_Detail_Func draw_body_table[3] =
	{
		&EditorImGui::DrawBodyOverlayDetailsImpl<EMotionType::Static>,
		nullptr, //kinematic
		&EditorImGui::DrawBodyOverlayDetailsImpl<EMotionType::Dynamic>
	};

	VX_ASSERT(body_id.IsValid());


	auto& body = physics_world->GetBodyManager().GetBody(body_id);
	(this->*draw_body_table[static_cast<int>(body.MotionType())])(
		body, physics_world->GetBodyManager().GetBodyDebugInfo(body_id));

	if (ImGui::Button("Delete"))
		physics_world->RemoveBody(body_id);
}

void EditorImGui::DrawBodiesOverlayItems(vx::BodyManager& body_manager, vx::PhysicsWorld* physics_world)
{
	std::vector<vx::Body>& bodies = body_manager.GetBodies();

	//funcion map 
	using Draw_Body_Detail_Func = void(EditorImGui::*)(vx::Body&, vx::BodyDebug&);
	static const Draw_Body_Detail_Func draw_body_table[3] =
	{
		&EditorImGui::DrawBodyOverlayDetailsImpl<EMotionType::Static>,
		nullptr, //kinematic
		&EditorImGui::DrawBodyOverlayDetailsImpl<EMotionType::Dynamic>
	};

	uint32 count = static_cast<uint32>(bodies.size());
	for (size_t i = 0; i < count; ++i)
	{
		auto& p = bodies[i];

		vx::BodyID id = p.ID();
		if (!id.IsValid())
			continue;

		ImGui::PushID(&p);
		ImGui::SeparatorText(body_manager.GetBodyDebugName(id));
		//draw_body_prop(p);
		(this->*draw_body_table[static_cast<int>(p.MotionType())])(p, body_manager.GetBodyDebugInfo(p));
		if (ImGui::Button("Delete"))
			physics_world->RemoveBody(id);
		ImGui::PopID();
	}
}

void EditorImGui::DrawDistanceConstraintOverlayUniqueProps(vx::DistanceConstraint& constraint)
{
	Vec3 _p = constraint.LocalAnchorA();
	if (ImGui::DragFloat3("Local Anchor A", &_p[0], 0.01f))
		constraint.SetLocalAnchorA(_p);
	_p = constraint.LocalAnchorB();
	ImGui::DragFloat3("Local Anchor B", &_p[0], 0.01f);
	constraint.SetLocalAnchorB(_p);

	float min_dist = constraint.MinDistance();
	float max_dist = constraint.MaxDistance();
	bool updated_dist = ImGui::DragFloat("Min Distance", &min_dist, 0.1f);
	updated_dist |= ImGui::DragFloat("Max Distance", &max_dist, 0.1f);
	if (updated_dist)
		constraint.SetDistance(min_dist, max_dist);

	ImGui::Text("Accumulated Lambda: %f", constraint.GetAccumulatedLambda());

	if(ImGui::TreeNode("Spring Setting"))
	{
		float v = constraint.SpringFrequency();
		if (ImGui::SliderAngle("mFrequency [Hz:Rad/sec]", &v, 0.0f))
			constraint.SetSpringFrequency(v);

		v = constraint.SpringDampingRatio();
		if (ImGui::DragFloat("Damping Ratio", &v, 0.01f))
			constraint.SetSpringDampingRatio(v);

		ImGui::TreePop();
	}
}

void EditorImGui::DrawPointConstraintOverlayUniqueProps(vx::PointConstraint& constraint)
{
	Vec3 _p = constraint.LocalAnchorA();
	if (ImGui::DragFloat3("Local Anchor A", &_p[0], 0.01f))
		constraint.SetLocalAnchorA(_p);
	_p = constraint.LocalAnchorB();
	ImGui::DragFloat3("Local Anchor B", &_p[0], 0.01f);
	constraint.SetLocalAnchorB(_p);

	ImGui::Checkbox("Has Velocity Bias", &constraint.mHasVelocityBias);
	ImGui::DragFloat("Error treshold", &constraint.mErrorTreshold);

	ImGui::Text("Accumulated Lambda: %s", constraint.GetAccumulatedLambda().ToString().c_str());
}

void EditorImGui::DrawConstraintsOverlayItems(vx::BodyManager& body_manager, std::vector<vx::Constraint*>& constraints)
{
	vx::uint32 count = 0;
	for(int i = 0; i < constraints.size(); ++i)
	{
		vx::Constraint* c = constraints[i];
		ImGui::PushID(c);
		vx::StackString<32> text;
		text << ++count << " Type: " << c->TypeName();
		ImGui::SeparatorText(text.Data());
		ImGui::Text("Idx: %d", c->ConstraintIdx());
		//two body constraint 
		Body* bA = c->BodyA();
		Body* bB = c->BodyB();

		if(bA && bB)
		{
			ImGui::Text("Body A: [%s].\nBody B: [%s].",
				body_manager.GetBodyDebugName(*bA),
				body_manager.GetBodyDebugName(*bB));
		}


		switch (c->Type())
		{
		case vx::EConstraintType::Distance:
			DrawDistanceConstraintOverlayUniqueProps(*(vx::DistanceConstraint*)(c));
			break;
		case vx::EConstraintType::Point:
			DrawPointConstraintOverlayUniqueProps(*(vx::PointConstraint*)(c));
			break;
		}
		ImGui::PopID();
	}
}

void EditorImGui::HelpInformation(const char* desc)
{
	ImGui::TextDisabled("(?)");
	if (ImGui::BeginItemTooltip())
	{
		ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
		ImGui::TextUnformatted(desc);
		ImGui::PopTextWrapPos();
		ImGui::EndTooltip();
	}
}

bool EditorImGui::EditQuatWithDrag(vx::Quat& quat, bool& editing, vx::Vec2& pad_size)
{
	ImVec2 drag_panel_size = ImVec2(pad_size.X(), pad_size.Y());
	static float drag_senstitivity = 0.001f;
	static int drag_speed = 5;
	static bool invert_x = false;
	static bool invert_y = false;

	//ImGui::Button("quat_editor", drag_panel_size);
	ImGui::InvisibleButton("##quat_editor", drag_panel_size);

	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	ImVec2 min = ImGui::GetItemRectMin();
	ImVec2 max = ImGui::GetItemRectMax();
	draw_list->AddRect(min, max, vx::Colour(1, 1, 1, 1.0f));

	//mouse drag
	if (ImGui::IsItemActive() && ImGui::IsMouseDragging(0))
	{
		vx::Colour col = vx::Colour(0.0f, 0.0f, 1.0f, 1.0f);
		draw_list->AddRect(min, max, col);
		ImVec2 dt_drag = ImGui::GetMouseDragDelta();

		//dt drag -> rot angle
		float dt_pitch = dt_drag.y * drag_senstitivity * drag_speed;
		float dt_yaw = dt_drag.x * drag_senstitivity * drag_speed;// *(invert_x) ? -1.0f : 1.0f;

		dt_yaw *= (invert_x) ? -1.0f : 1.0f;
		dt_pitch *= (invert_y) ? 1.0f : -1.0f;

		dt_yaw = fmod(dt_yaw + vx::kVxPi, vx::kVxTau) - vx::kVxPi;
		dt_pitch = fmod(dt_pitch + vx::kVxPi, vx::kVxTau) - vx::kVxPi;

		vx::Quat dq = vx::Quat::FromEulerAngle(dt_pitch, dt_yaw, 0.0f);
		quat = (dq * quat).Normalised();

		ImGui::ResetMouseDragDelta();
		editing = true;
	}
	else
		editing = false;
	ImGui::Text("Quat: (%.3f, %.3f, %.3f, %.3f)", quat.X(), quat.Y(), quat.Z(), quat.W());
	ImGui::DragFloat("Drag sensitivity", &drag_senstitivity, 0.1f, 0.0f, 0.0f, "%.5f");
	//ImGui::DragFloat4("Quat", &quat[0], 0.01f);
	ImGui::SliderInt("Drag Speed", &drag_speed, 1, 5);
	ImGui::SetCursorScreenPos(ImVec2(min.x + drag_panel_size.x + 5.0f, min.y));
	ImGui::Checkbox("Invert X", &invert_x);
	ImGui::SameLine();
	ImGui::Checkbox("Invert Y", &invert_y);

	return editing;
}



bool EditorImGui::EditEulerWithDrag(vx::Vec3& euler, bool& editing, vx::Vec2& pad_size, const void* tex)
{
	ImVec2 drag_panel_size = ImVec2(pad_size.X(), pad_size.Y());
	static float drag_senstitivity = 0.5f;
	static int drag_speed = 1;
	static bool invert_x = false;
	static bool invert_y = false;

	//ImGui::Button("quat_editor", drag_panel_size);
	ImGui::InvisibleButton("##euler_editor", drag_panel_size);

	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	ImVec2 min = ImGui::GetItemRectMin();
	ImVec2 max = ImGui::GetItemRectMax();
	draw_list->AddRect(min, max, vx::Colour(1, 1, 1, 1.0f));
	Texture* tex_buff = (Texture*)(tex);
	//Texture* tex_buff = static_cast<Texture*>(tex);
	if (tex_buff && tex_buff->IsValid())
	{
		draw_list->AddImage((ImTextureID)(intptr_t)tex_buff->ID(),
			min, max, ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
	}

	//mouse drag
	if (ImGui::IsItemActive() && ImGui::IsMouseDragging(0))
	{
		vx::Colour col = vx::Colour(0.0f, 0.0f, 1.0f, 1.0f);
		draw_list->AddRect(min, max, col);
		ImVec2 dt_drag = ImGui::GetMouseDragDelta();

		//dt drag -> rot angle
		float dt_pitch = dt_drag.y * drag_senstitivity * drag_speed;
		float dt_yaw = dt_drag.x * drag_senstitivity * drag_speed;// *(invert_x) ? -1.0f : 1.0f;

		dt_yaw *= (invert_x) ? -1.0f : 1.0f;
		dt_pitch *= (invert_y) ? 1.0f : -1.0f;

		euler += vx::Vec3(dt_pitch, dt_yaw, 0.0f);

		ImGui::ResetMouseDragDelta();
		editing = true;
	}
	else
		editing = false;
	ImGui::Text("Euler: %s", euler.ToString().c_str());
	DragFloat("Drag sensitivity", &drag_senstitivity, 0.05f);
	//ImGui::DragFloat4("Quat", &quat[0], 0.01f);
	SliderInt("Drag Speed", &drag_speed, 1, 5);
	ImGui::DragFloat2("Pad Size", &pad_size[0], 0.1f);
	ImGui::SetCursorScreenPos(ImVec2(min.x + drag_panel_size.x + 5.0f, min.y));
	ImGui::Checkbox("Invert X", &invert_x);
	ImGui::SameLine();
	ImGui::Checkbox("Invert Y", &invert_y);

	return editing;
}



bool EditorImGui::ColourEdit3(const char* label, vx::Colour& c)
{
	vx::Vec4 v = c.ToVec4();
	bool changed = ImGui::ColorEdit3(label, &v[0]);
	if (changed)
		c.ToColour(&v[0], true);
	return changed;
}

bool EditorImGui::ColourEdit4(const char* label, vx::Colour& c)
{
	vx::Vec4 v = c.ToVec4();
	bool changed = ImGui::ColorEdit4(label, &v[0]);
	if (changed)
		c.ToColour(&v[0], true);
	return changed;
}

bool EditorImGui::InternalCombo(const char* label, int* current_item, const char* items_separated_by_zeros, int height_in_items)
{
	int items_count = 0;
	const char* p = items_separated_by_zeros;       // FIXME-OPT: Avoid computing this, or at least only when combo is open
	while (*p)
	{
		p += strlen(p) + 1;
		items_count++;
	}


	auto Items_Single_String_Getter = [](void* data, int idx)
	{
		const char* items_separated_by_zeros = (const char*)data;
		int items_count = 0;
		const char* p = items_separated_by_zeros;
		while (*p)
		{
			if (idx == items_count)
				break;
			p += strlen(p) + 1;
			items_count++;
		}
		return *p ? p : nullptr;
	};

	bool value_changed = ImGui::Combo(label, current_item, Items_Single_String_Getter, (void*)items_separated_by_zeros, items_count, height_in_items);
	return value_changed;
}

void EditorImGui::DrawBodyFlags(vx::EBodySimphaseFlags flags)
{

	//auto Flag_Status = [&](const char* label, vx::EBodyDebugFlags bit, ImVec4 active_colour)
	//	{

	//	};


	auto Display_Bit = [&](const char* label, vx::EBodySimphaseFlags bit)
	{
		bool v = vx::Contains(flags, bit);
		ImGui::Checkbox(label, &v);
	};

	ImGui::BeginDisabled();
	Display_Bit("In Broadphase Pair", vx::EBodySimphaseFlags::InBroadphase);
	Display_Bit("In Narrowphase", vx::EBodySimphaseFlags::InNarrowphase);
	Display_Bit("Active Collision", vx::EBodySimphaseFlags::IsColliding);
	Display_Bit("Static Contact", vx::EBodySimphaseFlags::IsTouchingStatic);
	ImGui::EndDisabled();
}

bool EditorImGui::DrawAllowedDofFlags(vx::EDynamicsDofs& flags)
{
	if (ImGui::TreeNode("Allowed Degrees of freedom"))
	{
		int _flags = int(flags);
		bool toggled = ImGui::CheckboxFlags("All", &_flags, int(vx::EDynamicsDofs::All));
		ImGui::Text("Translation: ");
		ImGui::SameLine();
		toggled |= ImGui::CheckboxFlags("X", &_flags, int(vx::EDynamicsDofs::TranslateX));
		ImGui::SameLine();
		toggled |= ImGui::CheckboxFlags("Y", &_flags, int(vx::EDynamicsDofs::TranslateY));
		ImGui::SameLine();
		toggled |= ImGui::CheckboxFlags("Z", &_flags, int(vx::EDynamicsDofs::TranslateZ));
		ImGui::Text("Rotation:     "); //could be bothered to make table; space to align with translation
		ImGui::PushID(&flags);
		ImGui::SameLine();
		toggled |= ImGui::CheckboxFlags("X", &_flags, int(vx::EDynamicsDofs::RotationX));
		ImGui::SameLine();
		toggled |= ImGui::CheckboxFlags("Y", &_flags, int(vx::EDynamicsDofs::RotationY));
		ImGui::SameLine();
		toggled |= ImGui::CheckboxFlags("Z", &_flags, int(vx::EDynamicsDofs::RotationZ));
		ImGui::PopID();

		flags = vx::EDynamicsDofs(_flags);

		ImGui::TreePop();
		return toggled;
	}
	return false;
}

