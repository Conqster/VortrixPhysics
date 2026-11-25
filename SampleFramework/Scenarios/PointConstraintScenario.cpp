#include "PointConstraintScenario.h"
#include "Vortrix/PhysicsWorld.h"

#include "Vortrix/Maths/ScalarMath.h"

#include "Vortrix/Collision/Shapes/Shape.h"
#include "Vortrix/Collision/Shapes/BoxShape.h"
#include "Vortrix/Collision/Shapes/CapsuleShape.h"
#include "Vortrix/Collision/Shapes/SphereShape.h"

#include "SampleFramework/Camera.h"

#include "external/imgui/imgui.h"

#include "Vortrix/Dynamics/Constraints/PointConstraint.h"

#include "SampleFramework/EditorUI.h"

void PointConstraintScenario::Init(vx::PhysicsWorld* i_world)
{
	Scenario::Init(i_world);

	//vx::BodySettings dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();

	//dyn_bodies_settings.position = vx::Vec3(0.0f, 5.5f, 0.0f);
	//dyn_bodies_settings.debug_name = "box";
	//dyn_bodies_settings.shape = new vx::BoxShape(0.5f);
	//mPhysicsWorld->CreateBody(dyn_bodies_settings);



	vx::BodySettings static_bodies_settings = vx::BodySettings::DefaultStaticConstruct();

	vx::CapsuleShapeSettings capsule_settings(0.5f, 0.5f);
	capsule_settings.SetDensity(0.0f);

	static_bodies_settings.position = vx::Vec3(-2.0f, 5.5f, 0.0f);
	static_bodies_settings.debug_name = "capsule";
	static_bodies_settings.shape = vx::MakeRef<vx::CapsuleShape>(capsule_settings); // new vx::BoxShape(0.5f);
	static_bodies_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
	mPhysicsWorld->CreateBody(static_bodies_settings);

	vx::BodySettings dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();
	dyn_bodies_settings.position = vx::Vec3(-1.5f, 5.0f, 0.0f);
	//dyn_bodies_settings.position = vx::Vec3(1.0f, 4.0f, 0.0f);
	
	//dyn_bodies_settings.position = vx::Vec3(2.0f, 5.5f, 0.0f);
	//dyn_bodies_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
	dyn_bodies_settings.debug_name = "capsule";
	dyn_bodies_settings.shape = vx::MakeRef<vx::CapsuleShape>(0.5f, 0.5f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);


	//vx::DistanceConstraint joint = vx::DistanceConstraint(&mPhysicsWorld->GetBodies()[0], &mPhysicsWorld->GetBodies()[1], rope_constraint_settings);
	//joint.SetLocalAnchorA(vx::Vec3(0.0f, 0.5f, 0.0f));
	//joint.SetLocalAnchorB(vx::Vec3(0.0f, 0.5f, 0.0f));

	vx::PointConstraintSettings point_constraint_setting_ws;
	point_constraint_setting_ws.anchorPointFrame = vx::EConstraintFrame::World;
	point_constraint_setting_ws.anchorA = dyn_bodies_settings.position + vx::Vec3(0.0f, 0.5f, 0.0f);
	point_constraint_setting_ws.anchorB = dyn_bodies_settings.position + vx::Vec3(0.0f, 0.5f, 0.0f);

	vx::PointConstraintSettings point_constraint_setting;
	point_constraint_setting.anchorPointFrame = vx::EConstraintFrame::Local;
	point_constraint_setting.anchorA = vx::Vec3(0.0f, 1.1875f, 0.0f);
	point_constraint_setting.anchorB = vx::Vec3(0.0f, -1.1875f, 0.0f);


	//maybe every frame update worldanhor
	//mPhysicsWorld->mBallJoint = new vx::PointConstraint(&mPhysicsWorld->GetBodies()[0], &mPhysicsWorld->GetBodies()[1], point_constraint_setting);
	mConstraints.emplace_back(new vx::PointConstraint(&mPhysicsWorld->Bodies()[0], &mPhysicsWorld->Bodies()[1], point_constraint_setting));
	//mPhysicsWorld->AddConstraint(mTestConstraint);

	point_constraint_setting.anchorA = vx::Vec3(0.0f, 1.0f, 0.0f);
	point_constraint_setting.anchorB = vx::Vec3(0.0f, -1.0f, 0.0f);


	//offset next test to the right 
	vx::Vec3 offset = vx::Vec3(4.0f, 0.0f, 0.0f);
	static_bodies_settings.position += offset;
	mPhysicsWorld->CreateBody(static_bodies_settings);



	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	mConstraints.emplace_back(new vx::PointConstraint(&mPhysicsWorld->Bodies()[2], &mPhysicsWorld->Bodies()[3], point_constraint_setting));

	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	mConstraints.emplace_back(new vx::PointConstraint(&mPhysicsWorld->Bodies()[1], &mPhysicsWorld->Bodies()[4], point_constraint_setting));

	//constraint between a cube and sphere 
	dyn_bodies_settings.position += offset;
	dyn_bodies_settings.debug_name = "box";
	dyn_bodies_settings.shape = vx::MakeRef<vx::BoxShape>(0.5f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.debug_name = "sphere";
	dyn_bodies_settings.shape = vx::MakeRef<vx::SphereShape>(0.5f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	point_constraint_setting.anchorA = vx::Vec3(0.0f, 1.0f, 0.0f);
	point_constraint_setting.anchorB = vx::Vec3(0.0f, 0.0f, 0.0f);

	

	mConstraints.emplace_back(new vx::PointConstraint((&mPhysicsWorld->Bodies().back()) - 1, &mPhysicsWorld->Bodies().back(), point_constraint_setting));
	
	//mPhysicsWorld->CreateConstraintsT(pt_joints, 3);
	mPhysicsWorld->AddConstraints(mConstraints.data(), mConstraints.size());

	/// Ground plane
	CreateGroundPlane(100.0f);
}

void PointConstraintScenario::PostPhysicsStep(float dt)
{
	Scenario::PostPhysicsStep(dt);
	//mPhysicsWorld->mBallJoint->ResolveOffset();
}

void PointConstraintScenario::OnClose()
{
	Scenario::OnClose();
}

PointConstraintScenario::~PointConstraintScenario()
{
	Scenario::OnClose();
	
	//dontr use desconstructor here
	// as Os would reclaim mem anyway for now 
	//OnClose();
}

void PointConstraintScenario::OnUI()
{
	Scenario::OnUI();

	if (mPhysicsWorld == nullptr)
		return;


	//contact contraints
	auto& contact_constraint_stat = mPhysicsWorld->ContactConstraintSolverStats();

	if (mAppUI->BeginPanel("##PersistentContactScenario Window", nullptr,
		500, 250, EditorUI::EPanelScreenAnchor::HackTopRightConsiderPrimaryPanel,
		1.0f / vx::Vec2(3.0f, 3.0f), ImGuiWindowFlags_NoTitleBar))
	{
		EditorUI::TextColoured(EditorUI::kAccent, "PersistentContactScenario Window");
		vx::StackString<32> text;
		ImGui::Text("Last Step Bias: %s", text.Data());

		if (!mConstraints.empty())
		{
			static int curr_idx = 0;
			if (EditorUI::SliderInt("Constraint Idx", &curr_idx, 0, mConstraints.size()-1))
				curr_idx = vx::VxClamp(curr_idx, 0, int(mConstraints.size()-1));
			ImGui::SameLine();
			auto* constraint = mConstraints[curr_idx];
			if(constraint->ConstraintIdx() != vx::Constraint::kInvalidIdx)
			{
				if (ImGui::Button("Remove"))
					mPhysicsWorld->RemoveConstraint(constraint);
			}
			else
			{
				if (ImGui::Button("Add"))
					mPhysicsWorld->AddConstraint(constraint);
			}
		}


		ImGui::Text("Num of Contacts: %d", contact_constraint_stat.numContactConstraints);
		ImGui::Text("Num of Persistnet Contacts: %d", contact_constraint_stat.numPersistentContact);

		ImGui::Text("Num of Persistent Point: %d", contact_constraint_stat.actualPointCounts);
		ImGui::Text("Num of Valid Persistent Point: %d", contact_constraint_stat.actualPersistentPointCounts);
	}
	ImGui::End();
}

