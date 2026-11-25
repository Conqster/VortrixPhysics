#include "RagdollScenario.h"
#include "Vortrix/PhysicsWorld.h"

#include "external/imgui/imgui.h"

#include "Vortrix/Dynamics/RagdollBuilder.h"

#include "SampleFramework/EditorUI.h"

#include "SampleFramework/Display/ApplicationWindow.h"
#include "Vortrix/Core/Profiler.h"

std::vector<vx::Ragdoll> mRagdolls;

void RagdollScenario::Init(vx::PhysicsWorld* i_world)
{
	Scenario::Init(i_world);

	//enable to set/override pose
	//RAGDOLL BUILDER

	//make hip the origin

	vx::RagdollBuilder ragdoll_builder(mPhysicsWorld);

	const vx::uint32 multiples = 30;
	mRagdolls.clear();
	mRagdolls.resize(6 * multiples);
	vx::uint32 created = 0;
	float y_offset = 0;
	float x_offset = 0;
	for(vx::uint32 i = 0; i < multiples; ++i)
	{
		vx::RagdollSettings rag_settings;
		rag_settings.position = vx::Vec3(-5.0f + x_offset, 2.0f + y_offset, 0.0f);
		rag_settings.limbsOffset = 0.1f;
		CreateRagdoll(rag_settings, &mRagdolls[created++]);

		rag_settings.position = vx::Vec3(0.0f + x_offset, 2.0f + y_offset, 0.0f);
		CreateRagdoll(rag_settings, &mRagdolls[created++]);

		rag_settings.position = vx::Vec3(5.0f + x_offset, 2.0f + y_offset, 0.0f);
		rag_settings.splitTorso = false;
		rag_settings.mShapesType = vx::EShapeType::Capsule;
		CreateRagdoll(rag_settings, &mRagdolls[created++]);

		rag_settings = vx::RagdollSettings();
		rag_settings.position = vx::Vec3(-5.0f + x_offset, 2.0f + y_offset, -4.0f);
		CreateRagdoll(rag_settings, &mRagdolls[created++]);

		rag_settings.position = vx::Vec3(5.0f + x_offset, 2.0f + y_offset, -4.0f);
		CreateRagdoll(rag_settings, &mRagdolls[created++]);

		rag_settings.position = vx::Vec3(0.0f + x_offset, 2.0f + y_offset, -4.0f);
		CreateRagdoll(rag_settings, &mRagdolls[created++]);

		if ((i % 3) == 0)
		{
			x_offset += 10.0f;
			y_offset += 3.0f;
		}
	}


	CreateGroundPlane(300.0f);
}

void RagdollScenario::PostPhysicsStep(float dt)
{
	Scenario::PostPhysicsStep(dt);
}

void RagdollScenario::OnUI()
{
	//using namespace vx::Profiler;
	vx::Profiler::TimeTaken scope_time(__FUNCTION__, vx::Profiler::EProfileMode::VariableRate);
	Scenario::OnUI();

	if (mPhysicsWorld == nullptr)
		return;

	const auto& body_manager = mPhysicsWorld->GetBodyManager();
	if (mAppUI->BeginPanel("##Ragdolls", nullptr,
		500.0f, float(mAppWindow->Height()),
		EditorUI::EPanelScreenAnchor::TopLeft, vx::Vec2(0.25f, 1.0f),
		ImGuiWindowFlags_NoTitleBar))
	{
		EditorUI::TextColoured(EditorUI::kAccent, "Ragdolls");
		for (auto& ragdoll : mRagdolls)
		{
			ImGui::PushID(&ragdoll);

			ImGui::SeparatorText("Limbs");
			for (auto& body_name : ragdoll.mBodyNames)
			{
				ImGui::Text("%d: %s, Body: [%s]", body_name.idx, body_name.name.Data(), body_manager.GetBodyDebugName(ragdoll.mBodyIDs[body_name.idx]));
			}
			ImGui::SeparatorText("joints");
			for (auto& constraint_name : ragdoll.mConstraintNames)
			{
				ImGui::Text("%d: %s, Constraint idx: %d", constraint_name.idx, constraint_name.name.Data(), ragdoll.mConstraints[constraint_name.idx]->ConstraintIdx());
			}

			ImGui::Separator();
			ImGui::PopID();
		}
	}
	ImGui::End();
}
