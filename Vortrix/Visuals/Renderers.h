#pragma once
#include "Vortrix/Core/NonCopyable.h"

#include "ETextAlignment.h"

struct RenderableMesh;
class ApplicationWindow;
struct DrawCommand;
class IRenderTarget;

#include "Vortrix/Core/Colours.h"

namespace vx {
	
	//#include <GLM/glm/glm.hpp>
	//since multiple object could share Mesh 
	struct RenderableEntity
	{
		RenderableMesh* renderableMesh = nullptr;
		vx::Mat44 transform = vx::Mat44(1.0f);
		bool solidRender = true;
		bool canCastShadow = true;

		//glm::vec4 colour = glm::vec4(1.0f);
		vx::Colour colour = vx::Colour::sWhite;
		bool plainTexture = false;
	};

	struct AABB;
	
	enum class ERenderInstanceFlags : uint32;

	class Renderer : public NonCopyable
	{
	public:
		virtual ~Renderer() = default;

		virtual void SubmitSpherePrimitive(const RenderableEntity entity, const ERenderInstanceFlags flags) = 0;
		virtual void SubmitCubePrimitive(const RenderableEntity entity, const ERenderInstanceFlags flags) = 0;
		virtual void SubmitCapsulePrimitive(const RenderableEntity entity, const ERenderInstanceFlags flags) = 0;
		virtual void SubmitQuadPrimitive(const RenderableEntity entity, const ERenderInstanceFlags flags) = 0;
		virtual void SubmitQuadXZPrimitive(const RenderableEntity entity, const ERenderInstanceFlags flags) = 0;
		virtual void SubmitQuadXZDoubledSidedPrimitive(const RenderableEntity entity, const ERenderInstanceFlags flags) = 0;

		virtual void DrawText3D(const std::string_view& text,
			const vx::Vec3& pos, float scale,
			const vx::Colour& col,
			ETextAlignment align) = 0;
	
		virtual void DrawText3D_DynScale(const std::string_view& text,
			const vx::Vec3& pos, float scale,
			const vx::Colour& col,
			ETextAlignment align) = 0;
	};

	

	class DebugGizmosRenderer : public NonCopyable
	{
	public:
		virtual ~DebugGizmosRenderer() = default;

		virtual bool Init(ApplicationWindow* window) = 0;

		virtual void DrawLine(const vx::Vec3& v0, const vx::Vec3& v1, const vx::Colour& colour) = 0;
		virtual void DrawWireTriangle(const vx::Vec3& v1, const vx::Vec3& v2, const vx::Vec3& v3, const vx::Colour& colour) = 0;
		virtual void DrawSolidTriangle(const vx::Vec3& v1, const vx::Vec3& v2, const vx::Vec3& v3, const vx::Colour& colour) = 0;
		virtual void DrawSolidWireTriangle(const vx::Vec3& v1, const vx::Vec3& v2, const vx::Vec3& v3, const vx::Colour& colour) = 0;
		//void DrawTriangle(vx::Vec3 v0, vx::Vec3 v1, vx::Vec3 v2, glm::vec4 colour, bool cull_face = true);
		virtual void DrawWireSphereDiscs(const vx::Vec3& center, float radius, const vx::Colour col, int segments = 48) = 0;
		virtual void DrawWireDisc(const vx::Vec3& center, float radius, const vx::Colour col, const vx::Vec3& plane_axis_x = vx::Vec3::Right(),
			const vx::Vec3& plane_axis_y = vx::Vec3::Up(), int segments = 48) = 0;
		virtual void DrawHalfWireDisc(const vx::Vec3& center, float radius, const vx::Colour col, const vx::Vec3& plane_axis_x = vx::Vec3::Right(),
			const vx::Vec3& plane_axis_y = vx::Vec3::Up(), int segments = 48) = 0;
		virtual void DrawWireDisc(const vx::Vec3& center, float radius, float ratio, const vx::Colour col, const vx::Vec3& plane_axis_x = vx::Vec3::Right(),
			const vx::Vec3& plane_axis_y = vx::Vec3::Up(), int segments = 48) = 0;
		virtual void DrawArrow(const vx::Vec3& v0, const vx::Vec3& v1, const vx::Vec3& base_axis, float width, float height, vx::Colour col) = 0;

		virtual void DrawCone(const vx::Vec3& apex, const vx::Vec3& base_center, float radius, int segments = 12, vx::Colour col = vx::Colour::sMagenta) = 0;
		virtual void DrawArrowCone(const vx::Vec3& start, const vx::Vec3& end,
			float shaft_radius, float head_height, float head_radius,
			int cone_segment = 12, vx::Colour col = vx::Colour::sMagenta) = 0;
		/// draw a rectangle prism from corners 
		virtual void DrawBox(const std::array<vx::Vec3, 8>& corners, const vx::Colour& col, bool wireframe = true) = 0;
		virtual void DrawAABB(const vx::Vec3& min, const vx::Vec3& max, const vx::Colour& col, bool wireframe = true) = 0;
		virtual void DrawAABB(const vx::AABB& aabb, const vx::Colour& col, bool wireframe = true) = 0;

		/// axis aligned cross 
		virtual void DrawAACross(const vx::Vec3& pos, const Colour* colour, int colour_count, float scale = 1.0f) = 0;
		virtual void DrawCross(const vx::Vec3& pos, const vx::Vec3& rt,
			const vx::Vec3& up, const vx::Vec3& fwd, const Colour* colour, int colour_count, float scale = 1.0f) = 0;

		virtual void DrawSphere(const vx::Vec3& center, float radius, vx::Colour col) = 0;
		virtual void DrawWireSphere(const vx::Vec3& center, float radius, vx::Colour col) = 0;
		/// 4 sector, 4 stack
		virtual void DrawSphere4x4(const vx::Vec3& center, float radius, vx::Colour col) = 0;

		virtual void DrawBasis(const vx::Vec3& pos, const vx::Vec3& rt,
			const vx::Vec3& up, const vx::Vec3& fwd,
			float arrow_width, float arrow_height,
			bool b_draw_plane = true, uint32_t cone_segment = 8) = 0;

		virtual void DrawBasis(const vx::Mat44 transform, float arrow_width, float arrow_height, bool b_draw_plane = true, uint32_t cone_segment = 8) = 0;
	
		virtual float GetLineWidth() const = 0;
		virtual void SetLineWidth(float value) = 0;


		virtual DrawCommand* PushDrawCommand(DrawCommand cmd) = 0;
		virtual DrawCommand* PushDrawCommand(vx::Mat44 proj, vx::Mat44 view, IRenderTarget* render_target) = 0;
		virtual bool EndCurrentDrawCommand() = 0;
		virtual void RemoveDrawCommand(DrawCommand& cmd) = 0;
		virtual void ExecuteDraws() = 0;
		virtual bool ExecuteDraw(IRenderTarget* render_target) = 0;
		virtual bool ExecuteDraw(const DrawCommand& cmd) = 0;
		virtual bool Flush(IRenderTarget* render_target) = 0;
	};


	//template<typename Enum>
	//extern bool UICombo(const char* label, Enum& value, const char* items_separated_by_zeros, int height_in_items = -1);
} //namespace vx