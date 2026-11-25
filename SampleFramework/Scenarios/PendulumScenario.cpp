#include "PendulumScenario.h"
#include <external/imgui/imgui.h>

#include "SampleFramework/EditorUI.h"

#include "Vortrix/PhysicsWorld.h"
#include "Vortrix/Dynamics/Body/Body.h"

#include "Vortrix/Collision/Shapes/Shape.h"
#include "Vortrix/Collision/Shapes/SphereShape.h"

#include "Vortrix/Dynamics/Constraints/DistanceConstraint.h"

#include "SampleFramework/Camera.h"


vx::BodyID test_sphere_body_id;
vx::Vec3 old_pos;

void PendulumScenario::Init(vx::PhysicsWorld* i_world)
{
    Scenario::Init(i_world);

    if (mAppCamera)
    {
        Camera::State cam_state(
            vx::Vec3(-1.62f, 19.30, 29.27f),
            vx::Vec3(0.17f, -0.27f, -0.95f),
            vx::Vec3(0.05f, 0.96f, -0.27f)
        );

        mAppCamera->SetState(cam_state);
    }

    vx::BodySettings static_bodies_settings = vx::BodySettings::DefaultStaticConstruct();
    
    vx::BodySettings dynamic_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();
    dynamic_bodies_settings.linearDamping = 0.0f;
    dynamic_bodies_settings.angularDamping = 0.0f;
    dynamic_bodies_settings.restitution = 0.75f;// 0.2f;

    vx::SphereShapeSettings shape_settings(0.5f);
    vx::Ref<vx::SphereShape> unit_sphere = vx::MakeRef<vx::SphereShape>(shape_settings);

    vx::Vec3 last_pivot_position = vx::Vec3(0.0f, 16.5f, 0.0f);
    shape_settings.SetDensity(0.0f);
    static_bodies_settings = vx::BodySettings::DefaultStaticConstruct();
    static_bodies_settings.position = last_pivot_position;
    static_bodies_settings.debug_name = "sphere";
    static_bodies_settings.shape = unit_sphere;
    static_bodies_settings.inBroadphase = false;
    vx::Body* pivot_body = mPhysicsWorld->CreateBody(static_bodies_settings, false);
    test_sphere_body_id = pivot_body->ID();

    //pendulumn bob distance from pivot
    float rod_length = 5.0f;
    vx::Vec3 rod_displacement = vx::Vec3::Right() * rod_length;

    dynamic_bodies_settings.position = last_pivot_position + rod_displacement;
    dynamic_bodies_settings.debug_name = "sphere";
    dynamic_bodies_settings.shape = unit_sphere;
    vx::Body* body = mPhysicsWorld->CreateBody(dynamic_bodies_settings);

    old_pos = vx::Vec3(-1.0f, 16.5f, 0.0f);
    dynamic_bodies_settings.position = old_pos;
    test_sphere_body_id = mPhysicsWorld->CreateBody(dynamic_bodies_settings)->ID();


    vx::DistanceConstraintSettings dist_constraint_settings;
    //dist_constraint_settings.localAnchorB = vx::Vec3(0.25f);
    dist_constraint_settings.minDist = rod_length;
    dist_constraint_settings.maxDist = rod_length;
    dist_constraint_settings.frequency = 0.0f;// vx::DegToRad(360.0f);
    //dist_constraint_settings.dampingRatio = 1.0f;

    vx::DistanceConstraint dist_constraint{
        pivot_body,
        body,
        dist_constraint_settings
    };
    mPhysicsWorld->CreateConstraintT(dist_constraint);

    dynamic_bodies_settings.position = /*last_pivot_position + */last_pivot_position + rod_displacement + rod_displacement;
    dynamic_bodies_settings.debug_name = "sphere";
    dynamic_bodies_settings.shape = unit_sphere;
    vx::Body* body2 = mPhysicsWorld->CreateBody(dynamic_bodies_settings);
    vx::DistanceConstraint dist_constraint4{
    body,
    body2,
    dist_constraint_settings
    };

    mPhysicsWorld->CreateConstraintT(dist_constraint4);

    ///create multiple pendulums
    for (size_t i = 0; i < 0; ++i)
    {
        /// move the next pendulum along x 
        last_pivot_position -= vx::Vec3(1.25f, 0.0f, 0.0f);
        ///pivot
        static_bodies_settings.position = last_pivot_position;
        pivot_body = mPhysicsWorld->CreateBody(static_bodies_settings);
        //bob
        dynamic_bodies_settings.position = last_pivot_position + vx::Vec3::Up() * -rod_length;
        body = mPhysicsWorld->CreateBody(dynamic_bodies_settings);

        vx::DistanceConstraint dist_constraint2{
            pivot_body,
            body,
            dist_constraint_settings
        };
        mPhysicsWorld->CreateConstraintT(dist_constraint2);
    }


    CreateGroundPlane(100.0f);
}



void Ext_DrawAnimGraphSubDivision(ImVec2 min, ImVec2 size, ImDrawList* draw_list/*, AnimCurve& anim_graph*/)
{
    ImU32 line_col = IM_COL32(255, 255, 255, 50);

    ImVec2 p0 = ImVec2(0.0f, 0.0f);
    ImVec2 p1 = ImVec2(0.0f, 0.0f);

    std::vector<std::string> v = {
        "0.0", //just starting marker
        "0.1", "0.2", "0.3", "0.4",
        "0.5",
        "0.6", "0.7", "0.8", "0.9", "1.0", "1.1"
    };


    float graph_origin = 0.0f;
    float graph_step = 1.0f;
    int graph_step_count = 20;
    v.clear();
    v.reserve(graph_step_count);
    for (size_t i = 0; i < graph_step_count; i++)
        v.push_back(vx::ToString(graph_origin + graph_step * i, "%.1f").Data());
       // v.push_back(std::to_string(graph_origin + graph_step * i));


    //origin line
    float extra = 5.0f;
    //draw_list->AddLine(min, ImVec2(min.x, min.y + size.y + extra), IM_COL32(255, 0, 0, 150), 2.0f);
    //draw_list->AddLine(ImVec2(min.x - extra, min.y + size.y), ImVec2(min.x + size.x, min.y + size.y), IM_COL32(0, 255, 0, 150), 2.0f);

    ///graph origin
   ; /// need to move x pos backwards
    draw_list->AddText(ImVec2(min.x - 21.0f, min.y + size.y),
        IM_COL32(255, 255, 255, 100), v[0].c_str());

    //ImVec2 max = ImVec2(min.x + size.x, min.y + size.y);
    float text_offset = 10.0f;

    int subdivision_count = 10;
    subdivision_count = v.size();
    for (size_t i = 1; i < subdivision_count; ++i)
    {
        float ratio = float(i) / subdivision_count;

        p0 = ImVec2(min.x + size.x * ratio, min.y);
        p1 = p0;
        p1.y += size.y;
        draw_list->AddLine(p0, p1, line_col);
        //p1.y -= 15.0f;
        p1.x -= 10.0f;
        ImGui::SetCursorScreenPos(p1);
        draw_list->AddText(p1, IM_COL32(255, 255, 255, 100), v[i].c_str());
        //draw_list->AddLine(p0, ImVec2(p0.x, p0.y + size.y), line_col);

        //horiz
        ///horizontal in reverse, because origin is top-left
        ratio = float(subdivision_count - i) / subdivision_count;
        p0 = ImVec2(min.x, min.y + size.y * ratio);
        p1 = p0;
        p1.x += size.x;
        draw_list->AddLine(p0, p1, line_col);
        ImVec2 text_pos_xaxis(min.x - 21.0f, p1.y - 10.0f); /// need to move x pos backwards
        //p0.x += 5.0f;
        draw_list->AddText(text_pos_xaxis, IM_COL32(255, 255, 255, 100), v[i].c_str());
    }

    //for (float i = 0.1f; i < 1.0f; i += 0.1f)
    //{
    //    //v - min / max - min
    //    //vertical
    //    //p0 = ImVec2(i - min.x / max.x - min.x, min.y);
    //    p0 = ImVec2(min.x + size.x * i, min.y);
    //    p1 = p0;
    //    p1.y += size.y;
    //    draw_list->AddLine(p0, p1, line_col);
    //    //p1.y -= 15.0f;
    //    p1.x -= 10.0f;
    //    ImGui::SetCursorScreenPos(p1);
    //    draw_list->AddText(p1, IM_COL32(255, 255, 255, 100), v[(i - 0.1f) * 10].c_str());
    //    //draw_list->AddLine(p0, ImVec2(p0.x, p0.y + size.y), line_col);

    //    //horiz
    //    //p0 = ImVec2(min.x, i - min.y / max.y - min.y);
    //    p0 = ImVec2(min.x, min.y + size.y * i);
    //    p1 = p0;
    //    p1.x += size.x;
    //    draw_list->AddLine(p0, p1, line_col);
    //    ImVec2 text_pos_xaxis(min.x - 21.0f, p1.y - 10.0f); /// need to move x pos backwards
    //    //p0.x += 5.0f;
    //    draw_list->AddText(text_pos_xaxis, IM_COL32(255, 255, 255, 100), v[((1.0f - i) * 10)].c_str());
    //}
}


void AnimCurveWindowUI(/*AnimCurve& curve,*/ bool* p_open)
{
	static ImVec2 panel_size = ImVec2(350, 350);
	static ImVec2 curve_prop_size = ImVec2(500, 350);
	static ImVec2 graph_padding_offset = ImVec2(0, 0);

	ImGui::SetNextWindowSize(ImVec2(panel_size.x + curve_prop_size.x, 450), ImGuiCond_FirstUseEver);
	if (ImGui::Begin("Animation Curve", p_open))
	{
		ImDrawList* draw_list = ImGui::GetWindowDrawList();

		ImVec2 start_win_property = ImGui::GetCursorScreenPos();
		ImU32 border_colour = IM_COL32(200, 200, 200, 255);

        static ImVec2 start_win_property_offset(15, 0);
        start_win_property.x += start_win_property_offset.x;
        start_win_property.y += start_win_property_offset.y;

		//border 
		ImVec2 border_max = ImVec2(start_win_property.x + panel_size.x, start_win_property.y + panel_size.y);
		draw_list->AddRect(start_win_property, border_max, border_colour, 0, 2.0f);

		ImVec2 inner_border_min = ImVec2(start_win_property.x + graph_padding_offset.x, start_win_property.y + graph_padding_offset.y);
		ImVec2 inner_border_max = ImVec2(border_max.x - graph_padding_offset.x, border_max.y - graph_padding_offset.y);
		ImVec2 inner_panel_size = ImVec2(inner_border_max.x - inner_border_min.x, inner_border_max.y - inner_border_min.y);


		Ext_DrawAnimGraphSubDivision(inner_border_min, inner_panel_size, draw_list/*, curve*/);
		//Ext_DrawPointsInAnimGraph(inner_border_min, inner_panel_size, draw_list, curve);

		//Editor Properties
		float offset = 20.0f;
		ImGui::SetCursorScreenPos(ImVec2(start_win_property.x, start_win_property.y + panel_size.y + offset));
		if (ImGui::TreeNode("Editor Setting"))
		{
			ImGui::DragFloat2("Graph Offset", &start_win_property_offset[0]);
			if (ImGui::DragFloat2("Panel Size", &panel_size[0]))
				curve_prop_size.y = panel_size.y;
			ImGui::DragFloat("Curve Property Width", &curve_prop_size[0]);
			ImGui::DragFloat2("Padding", &graph_padding_offset[0]);
			//ImGui::DragFloat2("Anim Graph", &curve.size[0]);
			ImGui::TreePop();
		}


		//Curve Properties
		offset = 10.0f;
		ImGui::SetCursorScreenPos(ImVec2(border_max.x + offset, start_win_property.y));
		if (ImGui::BeginChild("##curve_properties", curve_prop_size))
		{
			ImGui::SeparatorText("Curve Properties");

			//ImGui::SliderFloat("Time", &curve.time, 0.0f, 1.0f, "%.3f");

			//int curr_sel = (int)curve.GetCurveType();
			//const char* curve_type[] = { "Linear", "QuadraticBeizer", "CubicBeizer" };
			//if (ImGui::Combo("Curve Types", &curr_sel, curve_type, 3))
			//	curve.SetCurveType((CurveType)curr_sel);

			//if (ImGui::Checkbox("Corner Archor", &curve.cornerAnchor))
			//	curve.ResetAnchor();

			ImGui::SeparatorText("Points");

			//for (size_t i = 0; i < curve.points.size(); i++)
			//{
			//	if (ImGui::DragFloat2(("Point " + std::to_string(i)).c_str(), &curve.points[i][0], 0.05f))
			//	{
			//		curve.points[i] = glm::clamp(curve.points[i], 0.0f, 1.0f);
			//	}

			//}

		}
		ImGui::EndChild();

	}
	ImGui::End();
}


//void Ext_DrawPointsInAnimGraph(ImVec2 min, ImVec2 size, ImDrawList* draw_list/*, AnimCurve& anim_graph*/)
//{
//	ImU32 point_col = IM_COL32(200, 200, 200, 200);
//	static float p_radius = 7.0f;
//
//	//invert y one-off
//	ImVec2 origin = ImVec2(min.x, min.y + size.y);
//
//
//
//	if (anim_graph.points.size() >= 2)
//	{
//
//		for (size_t i = 0; i < anim_graph.points.size() - 1; i++)
//		{
//			ImVec2 p0 = ImVec2(origin.x + size.x * anim_graph.points[i].x, origin.y - size.y * anim_graph.points[i].y);
//			ImVec2 p1 = ImVec2(origin.x + size.x * anim_graph.points[i + 1].x, origin.y - size.y * anim_graph.points[i + 1].y);
//
//			draw_list->AddLine(p0, p1, IM_COL32(0, 200, 0, 255), 1.5f);
//		}
//	}
//
//
//	//Draw curves 
//	if (anim_graph.points.size() >= 3)
//	{
//		ImVec2 p0 = ImVec2(origin.x + size.x * anim_graph.points[0].x, origin.y - size.y * anim_graph.points[0].y);
//		ImVec2 p1 = ImVec2(origin.x + size.x * anim_graph.points[1].x, origin.y - size.y * anim_graph.points[1].y);
//		ImVec2 p2 = ImVec2(origin.x + size.x * anim_graph.points[2].x, origin.y - size.y * anim_graph.points[2].y);
//
//		if (anim_graph.GetCurveType() == CurveType::QuadraticBeizer)
//		{
//			draw_list->AddBezierQuadratic(p0, p1, p2, IM_COL32(255, 255, 0, 200), 1.5f);
//			glm::vec2 o_p1;
//			glm::vec2 o_p2;
//			anim_graph.OutBeizerQuadratic(o_p1, o_p2);
//			draw_list->AddLine(ImVec2(origin.x + size.x * o_p1.x, origin.y - size.y * o_p1.y),
//				ImVec2(origin.x + size.x * o_p2.x, origin.y - size.y * o_p2.y),
//				IM_COL32(255, 0, 0, 155), 1.0f);
//		}
//		else if (anim_graph.GetCurveType() == CurveType::CubicBeizer && anim_graph.points.size() >= 4)
//		{
//			ImVec2 p3 = ImVec2(origin.x + size.x * anim_graph.points[3].x, origin.y - size.y * anim_graph.points[3].y);
//			draw_list->AddBezierCubic(p0, p1, p2, p3, IM_COL32(255, 255, 0, 200), 1.5f);
//
//			glm::vec2 pt[5] = { glm::vec2(), glm::vec2(), glm::vec2(), glm::vec2(), glm::vec2() };
//			anim_graph.OutBeizerCubic(pt[0], pt[1], pt[2], pt[3], pt[4]);
//			for (int i = 0; i < 4; i++)
//			{
//				if (i == 2)
//					continue;
//
//				draw_list->AddLine(ImVec2(origin.x + size.x * pt[i].x, origin.y - size.y * pt[i].y),
//					ImVec2(origin.x + size.x * pt[i + 1].x, origin.y - size.y * pt[i + 1].y),
//					IM_COL32(255, 0, 0, 200), 1.0f);
//			}
//		}
//	}
//
//
//	//quick hack 
//	int id = 0;
//	for (auto& p : anim_graph.points)
//	{
//		ImVec2 p0 = ImVec2(origin.x + size.x * p.x, origin.y - size.y * p.y);
//		draw_list->AddCircleFilled(p0, p_radius, point_col);
//
//		//Interactie dragging key frame
//		ImVec2 p_pos = ImVec2(p0.x - p_radius, p0.y - p_radius);
//		ImGui::SetCursorScreenPos(p_pos);
//		ImGui::InvisibleButton(("##p_event" + std::to_string(id++)).c_str(), ImVec2(p_radius * 2, p_radius * 2));
//		if (ImGui::IsItemActive() && ImGui::IsMouseDragging(0))
//		{
//			draw_list->AddCircle(ImVec2(p_pos.x + p_radius, p_pos.y + p_radius), p_radius * 2.5f, IM_COL32(0, 255, 255, 255), 0, 2.0f);
//			ImVec2 mouse = ImGui::GetMousePos();
//			p.x = (mouse.x - min.x) / size.x;
//			p.y = (mouse.y - min.y) / size.y;
//			//invert y 
//			p.y = 1.0f - p.y;
//			p = glm::clamp(p, 0.0f, 1.0f);
//		}
//		else if (ImGui::IsItemHovered())
//			draw_list->AddCircle(ImVec2(p_pos.x + p_radius, p_pos.y + p_radius), p_radius * 1.5f, IM_COL32(0, 255, 0, 255), 0, 2.0f);
//	}
//
//
//
//	//input x
//	if (anim_graph.time >= 0.0f && anim_graph.time <= 1.0f)
//	{
//		ImVec2 p0 = ImVec2(min.x + size.x * anim_graph.time, min.y + size.y);
//
//		//draw line
//		draw_list->AddLine(p0, ImVec2(p0.x, min.y), IM_COL32(225, 225, 225, 255), 1.5f);
//
//		//get value one first line
//		if (anim_graph.points.size() >= 2)
//		{
//
//			//actual line value
//			glm::vec2 test = anim_graph.OutValueVec2();
//			ImVec2 on_line = ImVec2(origin.x + size.x * test.x, origin.y - size.y * test.y);
//			draw_list->AddCircleFilled(on_line, p_radius, IM_COL32(200, 0, 0, 200));
//
//
//			for (size_t i = 0; i < anim_graph.points.size() - 1; i++)
//			{
//				glm::vec2 p = anim_graph.points[i] + anim_graph.time * (anim_graph.points[i + 1] - anim_graph.points[i]);
//				ImVec2 time_screen = ImVec2(origin.x + size.x * p.x, origin.y - size.y * p.y);
//				draw_list->AddCircle(time_screen, p_radius * 1.5f, IM_COL32(200, 200, 0, 200));
//			}
//		}
//	}
//}


void PendulumScenario::OnUI()
{
    
    //if (ImGui::Begin("Simulation Validation"))
    //{

    //    ImGui::End();
    //}

 //   bool open = true;
 //   AnimCurveWindowUI(&open);

	//if (ImGui::Begin("Pendulum Scenario"))
	//{
 //       if (test_sphere_body_id.IsValid())
 //       {
 //           static int last_step_validation = 0;// = mPhysicsWorld->Context()->mStepIndex;

 //           float physics_dt = mPhysicsWorld->Context()->mDeltaTime;
 //           vx::Body* body = &mPhysicsWorld->GetBodyManager().GetBody(test_sphere_body_id);

 //           vx::Vec3 body_pos = body->Position();

 //           ///chanvge in position
 //           vx::Vec3 delta_pos = body_pos - old_pos;
 //           int past_step_count = mPhysicsWorld->Context()->mStepIndex - last_step_validation;
 //           
 //           static vx::Vec3 predict_vel = vx::Vec3::Zero();
 //           if(past_step_count > 0)
 //               predict_vel = delta_pos / (physics_dt * past_step_count);
 //           float predict_speed = predict_vel.Length();

 //           ///new state 
 //           old_pos = body_pos;
 //           last_step_validation = mPhysicsWorld->Context()->mStepIndex;

 //           if (ImGui::DragFloat3("Body Position", &body_pos[0], 0.1f))
 //               body->SetPosition(body_pos);

 //           float speed = body->LinearVelocity().Length();
 //           ImGui::Text("Predicted Speed: %.2fms", predict_speed);
 //           ImGui::Text("Speed: %.2fms", speed);
 //           ImGui::Text("Acceleration: %.2fm/s/s", speed / physics_dt);
 //       }
 //       else
 //           ImGui::TextColored(ImVec4(1, 0, 0, 1), "Invalid body id");
	//	ImGui::End();
	//}
}