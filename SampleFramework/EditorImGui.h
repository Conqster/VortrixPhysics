#pragma once

#include <vector>

struct GLFWwindow;

namespace vx{
	namespace Particles {
		class Particle;
	}
}
class Texture;

#include "Vortrix/Core/Colours.h"
#include "Vortrix/Vortrix.h"


#include "Vortrix/Dynamics/Body/EBodyDebugFlags.h"
#include "Vortrix/Dynamics/Body/EDynamicsDofs.h"
#include "Vortrix/Dynamics/Body/BodyID.h"

namespace vx{
	class PhysicsWorld;
	class BodyManager;
	class Body;
	class BodyDebug;
	enum class EMotionType : uint8;

	class Constraint;
	class DistanceConstraint;
	class PointConstraint;
}

typedef int ImGuiTreeNodeFlags;
typedef int ImGuiSliderFlags;
typedef int ImGuiWindowFlags;


/// change EditorGUI/AppGUI
class EditorImGui
{
public:
	static vx::Colour kAccent;// = vx::Colour(0.3f, 0.75f, 0.8f, 1.0f);
	static vx::Colour kSecondaryAccent;// = vx::Colour(.102f, .42f, .478f, 1.0f);

	static  ImGuiTreeNodeFlags kTreeNodeFlags;

	//static constexpr vx::Colour ty = 0;

	static constexpr float kSliderWidth = 100.0f;

	static bool SliderInt(const char* label,
		int* v, int v_min, int v_max, const char* format = "%d",
		ImGuiSliderFlags flags = 0);

	static bool SliderFloat(const char* label,
		float* v, float v_min, float v_max, const char* format = "%.2f",
		ImGuiSliderFlags flags = 0);

	static bool DragFloat(const char* label, float* v,
		float v_speed = 1.0f, float v_min = 0.0f, float v_max = 0.0f,
		const char* format = "%.2f", ImGuiSliderFlags flags = 0);

	// NB: v_speed is float to allow adjusting the drag speed with more precision
	static bool DragInt(const char* label, int* v,
		float v_speed = 1.0f, int v_min = 0, int v_max = 0,
		const char* format = "%.2f", ImGuiSliderFlags flags = 0);

	static void TextColoured(const vx::Colour col, const char* fmt, ...);
	static void TextColourWrapped(const vx::Colour col, const char* fmt, ...);

	struct ScopePushItemWidth {
		ScopePushItemWidth(float width);
		~ScopePushItemWidth();
	};


	enum class EPanelScreenAnchor : vx::uint8
	{
		TopLeft,
		BottomLeft,
		TopRight,
		BottomRight,
	};

	void ResolvePanelPosSize(float target_width, float target_height,
		vx::Vec2& o_pos, vx::Vec2& o_size,
		EPanelScreenAnchor screen_anchor = EPanelScreenAnchor::TopLeft,
		vx::Vec2 max_ratio_screen = vx::Vec2(1.0f / 4));

	static bool BeginPanel(const char* name, bool* p_open, ImGuiWindowFlags flags = 0);
	bool BeginPanel(const char* name, bool* p_open, float target_width, float target_height,
		EPanelScreenAnchor screen_anchor = EPanelScreenAnchor::TopLeft,
		vx::Vec2 max_ratio_screen = vx::Vec2(1.0f / 4),
		ImGuiWindowFlags flags = 0);

	static void EndPanel();


	/// intermidate values
	vx::Vec2 mFrameWorkableScreenSize = vx::Vec2(0);
	float mFrameMenuItemY = 25.0f;

	void Initialise(GLFWwindow* glfw_win);
	void BeginNewFrame(float screen_width, float screen_height);
	//void BeginNewFrame();
	void RenderFrame();
	void Shutdown();

	bool UIBlockingMouseInput() const;
	bool UIBlockingInput();


	bool BeginMainMenu()const;
	void EndMainMenu();

	void DrawParticlesOverlayItems(std::vector<vx::Particles::Particle>& particles) const;

	void DrawBodyOverlayDetails(vx::BodyID body_id, vx::PhysicsWorld* physics_world);
	void DrawBodiesOverlayItems(vx::BodyManager& body_manager, vx::PhysicsWorld* physics_world);
	void DrawConstraintsOverlayItems(vx::BodyManager& body_manager, std::vector<vx::Constraint*>& constraints);


	static void HelpInformation(const char* desc);

	static bool EditQuatWithDrag(vx::Quat& quat, bool& editing,
		vx::Vec2& pad_size);

	static bool EditEulerWithDrag(vx::Vec3& euler, bool& editing, vx::Vec2& pad_size, const void* tex);


	static bool ColourEdit3(const char* label, vx::Colour& c);
	static bool ColourEdit4(const char* label, vx::Colour& c);
	

	template<typename Enum>
	static bool Combo(const char* label, Enum& value, const char* items_separated_by_zeros, int height_in_items = -1)
	{
		int _v = static_cast<int>(value);
		bool changed = InternalCombo(label, &_v, items_separated_by_zeros, height_in_items);
		if (changed)
			value = static_cast<Enum>(_v);
		return changed;
	}

private: 
	static bool InternalCombo(const char* label, int* current_item, const char* items_separated_by_zeros, int height_in_items = -1);
	

	static void DrawBodyFlags(vx::EBodySimphaseFlags flags);
	static bool DrawAllowedDofFlags(vx::EDynamicsDofs& flags);

	template<vx::EMotionType Type>
	void DrawBodyOverlayDetailsImpl(vx::Body& body, vx::BodyDebug& body_debug_info);
	void DrawDistanceConstraintOverlayUniqueProps(vx::DistanceConstraint& constraint);
	void DrawPointConstraintOverlayUniqueProps(vx::PointConstraint& constraint);
};



