#include "JointScenario.h"

#include "Vortrix/PhysicsWorld.h"

#include "Vortrix/Collision/Shapes/Shape.h"
#include "Vortrix/Collision/Shapes/SphereShape.h"
#include "Vortrix/Collision/Shapes/BoxShape.h"
#include "Vortrix/Collision/Shapes/CapsuleShape.h"

#include "external/imgui/imgui.h"

#include "Vortrix/Dynamics/Constraints/DistanceConstraint.h"
#include "Vortrix/Dynamics/ConstraintCoordinator.h"

#include "SampleFramework/EditorUI.h"
#include "SampleFramework/Display/ApplicationWindow.h"

#include <external/imgui/imgui.h>

void RopeSetting(vx::DistanceConstraintSettings& constraint_settings)
{
	constraint_settings.minDist = 1.25f;
	constraint_settings.maxDist = 2.5f;
	constraint_settings.frequency = vx::DegToRad(180.0f);
	constraint_settings.dampingRatio = 0.0f;
}

void SuspensionShockSettingCriticalDamping(vx::DistanceConstraintSettings& constraint_settings)
{
	constraint_settings.frequency = vx::DegToRad(360.0f);
	constraint_settings.dampingRatio = 0.1f;
}

void SuspensionShockSetting(vx::DistanceConstraintSettings& constraint_settings)
{
	constraint_settings.frequency = vx::DegToRad(270.0f);
	constraint_settings.dampingRatio = 2.0f;
}


void HardBarSetting(vx::DistanceConstraintSettings& constraint_settings)
{
	constraint_settings.minDist = 2.5f;
	constraint_settings.maxDist = 2.5f;
	constraint_settings.frequency = vx::DegToRad(0.0f);
	constraint_settings.dampingRatio = 0.0f;
}




void JointScenario::Init(vx::PhysicsWorld* i_world)
{
	Scenario::Init(i_world);
	
	vx::BodySettings dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();



	vx::Ref<vx::SphereShape> unit_sphere = vx::MakeRef<vx::SphereShape>(0.5f);
	dyn_bodies_settings.position = vx::Vec3(3.0f, 5.5f, 0.0f);
	dyn_bodies_settings.debug_name = "sphere";
	dyn_bodies_settings.shape = unit_sphere;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	vx::Ref<vx::BoxShape> unit_box = vx::MakeRef<vx::BoxShape>(0.5f);
	dyn_bodies_settings.position = vx::Vec3(-2.0f, 5.5f, 0.0f);
	dyn_bodies_settings.debug_name = "box";
	dyn_bodies_settings.shape = unit_box;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	vx::DistanceConstraintSettings rope_constraint_settings;
	//hard bar 
	RopeSetting(rope_constraint_settings);

	vx::DistanceConstraint joint = vx::DistanceConstraint(&mPhysicsWorld->Bodies()[0], &mPhysicsWorld->Bodies()[1], rope_constraint_settings);
	joint.SetLocalAnchorA(vx::Vec3(0.0f, 0.5f, 0.0f));
	joint.SetLocalAnchorB(vx::Vec3(0.0f, 0.5f, 0.0f));




	//mNotInPipelineJoints.reserve(25);
	mPhysicsWorld->CreateConstraintT(joint);

	vx::Ref<vx::CapsuleShape> unit_capsule = vx::MakeRef<vx::CapsuleShape>(0.5f, 0.5f);
	dyn_bodies_settings.position = vx::Vec3(-2.0f, 5.5f, 0.0f);
	dyn_bodies_settings.debug_name = "capsule";
	dyn_bodies_settings.shape = unit_capsule;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);


	vx::DistanceConstraint* new_j = mPhysicsWorld->CreateConstraintT(
		vx::DistanceConstraint(&mPhysicsWorld->Bodies()[1], 
							   &mPhysicsWorld->Bodies()[2], 
								rope_constraint_settings));
	new_j->SetLocalAnchorA(vx::Vec3(0.0f, -0.5f, 0.0f));
	new_j->SetLocalAnchorB(vx::Vec3(0.0f, 1.0f, 0.0f));
	

	dyn_bodies_settings.position = vx::Vec3(-2.0f, 5.5f, 0.0f);
	dyn_bodies_settings.debug_name = "box";
	dyn_bodies_settings.shape = unit_box;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);


	auto& new_joint2 = *mPhysicsWorld->CreateConstraintT(
		vx::DistanceConstraint(&mPhysicsWorld->Bodies()[2], 
							   &mPhysicsWorld->Bodies()[3],
								rope_constraint_settings));
	new_joint2.SetLocalAnchorA(vx::Vec3(0.0f, -1.0f, 0.0f)); //quick offset
	new_joint2.SetLocalAnchorB(vx::Vec3(0.5f)); //quick offset


	vx::DistanceConstraintSettings constraint_hardbar_settings;
	HardBarSetting(constraint_hardbar_settings);

	dyn_bodies_settings.position = vx::Vec3(0.0f, 3.5f, 0.0f);
	dyn_bodies_settings.debug_name = "box";
	dyn_bodies_settings.shape = unit_box;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(0.0f, 0.5f, 0.0f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(3.5f, 0.5f, 0.0f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(3.5f, 3.5f, 0.0f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	vx::Body* body_a = &mPhysicsWorld->Bodies()[4];
	vx::Body* body_b = &mPhysicsWorld->Bodies()[5];
	vx::Body* body_c = &mPhysicsWorld->Bodies()[6];
	vx::Body* body_d = &mPhysicsWorld->Bodies()[7];


	vx::DistanceConstraint joints[] =
	{
		vx::DistanceConstraint(body_a, body_b,constraint_hardbar_settings),
		vx::DistanceConstraint(body_b, body_c,constraint_hardbar_settings),
		vx::DistanceConstraint(body_c, body_d,constraint_hardbar_settings),
		vx::DistanceConstraint(body_d, body_a,constraint_hardbar_settings),
		vx::DistanceConstraint(body_a, body_c,constraint_hardbar_settings),
		vx::DistanceConstraint(body_b, body_d,constraint_hardbar_settings)
	};
	mPhysicsWorld->CreateConstraintsT(joints, 6);


	//achor to the static 
	vx::BodySettings static_bodies_settings = vx::BodySettings::DefaultStaticConstruct();
	static_bodies_settings.position = vx::Vec3(0.0f, 12.5f, 0.0f);
	static_bodies_settings.debug_name = "box";
	vx::BoxShapeSettings shape_settings(unit_box->HalfExtents());
	shape_settings.SetDensity(0.0f);
	static_bodies_settings.shape = vx::MakeRef<vx::BoxShape>(shape_settings);
	static_bodies_settings.intialVelocity = vx::Vec3(1.0f);
	vx::uint32 static_body_idx = mPhysicsWorld->Bodies().size();
	mPhysicsWorld->CreateBody(static_bodies_settings);

	RopeSetting(rope_constraint_settings);

	dyn_bodies_settings.position = vx::Vec3(0.0f, 12.5f, 0.0f);
	dyn_bodies_settings.shape = vx::MakeRef<vx::CapsuleShape>(0.5f, 0.5f);;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	auto& new_joint3 = *mPhysicsWorld->CreateConstraintT(
		vx::DistanceConstraint(&mPhysicsWorld->Bodies()[static_body_idx],
			&mPhysicsWorld->Bodies().back(),
			rope_constraint_settings));

	new_joint3.SetDistance(0.75f, 1.5f);
	new_joint3.SetLocalAnchorA(vx::Vec3(0.0f, -0.5f, 0.0f));
	new_joint3.SetLocalAnchorB(vx::Vec3(0.0f, 1.0f, 0.0f));

	//quickk reverse, B already set
	auto& new_joint4 = *mPhysicsWorld->CreateConstraintT(
		vx::DistanceConstraint(body_d,
			&mPhysicsWorld->Bodies().back(),
			rope_constraint_settings));
	new_joint4.SetLocalAnchorB(vx::Vec3(0.0f, -1.0f, 0.0f));
	new_joint4.SetLocalAnchorA(vx::Vec3(0.0f, 0.5f, 0.0f));


	CreateLattice();

	/// Ground plane
	CreateGroundPlane(100.0f);
}

void JointScenario::PostPhysicsStep(float dt)
{
	Scenario::PostPhysicsStep(dt);
}


//Constraint window
void JointScenario::ConstaintPanel(vx::DistanceConstraint& constraint)
{
	const vx::Body& bA = *constraint.BodyA();
	const vx::Body& bB = *constraint.BodyB();

	const auto& body_manager = mPhysicsWorld->GetBodyManager();
	ImGui::Text("Body A: [%s], id: %d \nBody B: [%s], id: %d", 
		body_manager.GetBodyDebugName(bA), bA.ID(),
		body_manager.GetBodyDebugName(bB), bB.ID() );

	vx::Vec3 _p = constraint.LocalAnchorA();
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
	
	ImGui::SeparatorText("Spring Setting");

	float v = constraint.SpringFrequency();
	if (ImGui::SliderAngle("mFrequency [Hz:Rad/sec]", &v, 0.0f))
		constraint.SetSpringFrequency(v);

	v = constraint.SpringDampingRatio();
	if (ImGui::DragFloat("Damping Ratio", &v, 0.01f))
		constraint.SetSpringDampingRatio(v);

	//ImGui::SliderFloat("Softness", &spring.softness, 0.0f, 1.0f);
}


void JointScenario::OnUI()
{
	Scenario::OnUI();

	if (mPhysicsWorld == nullptr)
		return;

	if (mPhysicsWorld->NonContactConstraints().empty())
		return;

	if (mAppUI->BeginPanel("##JointWin", nullptr, 
		500.0f, float(mAppWindow->Height()),
		EditorUI::EPanelScreenAnchor::TopLeft, vx::Vec2(0.25f, 1.0f), 
		ImGuiWindowFlags_NoTitleBar))
	{
		EditorUI::TextColoured(EditorUI::kAccent, "Joint Window");
		static bool use_new = true;

		//if (use_new)
		//	ConstaintPanel(mJoint);
		//else
		//{
		//	ImGui::SliderFloat("Joint Rest length", &mJoint.mRestLength, 0.0f, 10.0f);
		//	ImGui::SliderFloat("Joint Damping Ratio", &mJoint.mmDampingRatio, 0.0f, 1.0f);
		//	ImGui::DragFloat("Joint Stiffness", &mJoint.mStiffness);
		//}

		int _idx = 1;
		for (auto& joint : mPhysicsWorld->NonContactConstraints())
		{
			ImGui::Separator();
			ImGui::Spacing();
			ImGui::PushID(&joint);
			ImGui::Text("joint %d", _idx++);
			if (use_new)
				ConstaintPanel(*static_cast<vx::DistanceConstraint*>(joint));
				//ConstaintPanel(joint);
			else
			{
				//ImGui::SliderFloat("Joint Rest length", &joint.mRestLength, 0.0f, 10.0f);
				//ImGui::SliderFloat("Joint Damping Ratio", &joint.mmDampingRatio, 0.0f, 1.0f);
				//ImGui::DragFloat("Joint Stiffness", &joint.mStiffness);
			}
			ImGui::PopID();
		}
	}
	ImGui::End();
}

void JointScenario::CreateLattice()
{
	float depth = -4.0f;
	vx::uint32 first_body = mPhysicsWorld->Bodies().size();

	vx::BodySettings dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();

	dyn_bodies_settings.position = vx::Vec3(0.0f);
	dyn_bodies_settings.debug_name = "capsule"; //"box";
	dyn_bodies_settings.shape = vx::MakeRef<vx::CapsuleShape>(0.5f, 0.5f);//vx::BoxShape(0.5f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.debug_name = "box";
	dyn_bodies_settings.shape = vx::MakeRef<vx::BoxShape>(0.5f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	dyn_bodies_settings.position = vx::Vec3(0.0f, 3.5f, depth);
	dyn_bodies_settings.debug_name = "sphere"; //"box";
	dyn_bodies_settings.shape = vx::MakeRef<vx::SphereShape>(0.5f);//vx::BoxShape(0.5f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(0.0f, 0.5f, depth);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(3.5f, 0.5f, depth);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(3.5f, 3.5f, depth);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	depth -= 2.0f;

	dyn_bodies_settings.position = vx::Vec3(0.0f, 3.5f, depth);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(0.0f, 0.5f, depth);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(3.5f, 0.5f, depth);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(3.5f, 3.5f, depth);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);


	vx::Body* capsule0 = &mPhysicsWorld->Bodies()[first_body++];
	vx::Body* capsule1 = &mPhysicsWorld->Bodies()[first_body++];
	vx::Body* box = &mPhysicsWorld->Bodies()[first_body++];
	vx::Body* body_a = &mPhysicsWorld->Bodies()[first_body++];
	vx::Body* body_b = &mPhysicsWorld->Bodies()[first_body++];
	vx::Body* body_c = &mPhysicsWorld->Bodies()[first_body++];
	vx::Body* body_d = &mPhysicsWorld->Bodies()[first_body++];
	vx::Body* body_a1 = &mPhysicsWorld->Bodies()[first_body++];
	vx::Body* body_b1 = &mPhysicsWorld->Bodies()[first_body++];
	vx::Body* body_c1 = &mPhysicsWorld->Bodies()[first_body++];
	vx::Body* body_d1 = &mPhysicsWorld->Bodies()[first_body++];

	vx::DistanceConstraintSettings rope_constraint_settings;
	RopeSetting(rope_constraint_settings);

	///rope from capsule to box 
	rope_constraint_settings.localAnchorA = vx::Vec3(0.0f, -1.0f, 0.0f);
	rope_constraint_settings.localAnchorB = vx::Vec3(0.0f, 0.5f, 0.0f);
	mPhysicsWorld->CreateConstraintT(vx::DistanceConstraint(capsule0, box, rope_constraint_settings));

	//rope from box to mid capsule 
	rope_constraint_settings.localAnchorA = vx::Vec3(0.0f, -0.5f, 0.0f);
	mPhysicsWorld->CreateConstraintT(vx::DistanceConstraint(box, capsule1, rope_constraint_settings));

	rope_constraint_settings.minDist = 3.0f;
	rope_constraint_settings.maxDist = 6.0f;
	rope_constraint_settings.localAnchorA = vx::Vec3(0.0f, 1.0f, 0.0f);
	rope_constraint_settings.localAnchorB = vx::Vec3(0.25f);
	mTrackCapsuleSphereRope = mPhysicsWorld->CreateConstraintT(vx::DistanceConstraint(capsule1, body_a, rope_constraint_settings));
	
	rope_constraint_settings.localAnchorA = vx::Vec3(0.0f, -1.0f, 0.0f);
	mPhysicsWorld->CreateConstraintT(vx::DistanceConstraint(capsule1, body_d, rope_constraint_settings));
	
	rope_constraint_settings = vx::DistanceConstraintSettings();

	vx::DistanceConstraintSettings hardbar_constraint_settings;
	HardBarSetting(hardbar_constraint_settings);

	hardbar_constraint_settings.minDist = 3.5f;
	hardbar_constraint_settings.maxDist = 3.5f;
	hardbar_constraint_settings.dampingRatio = 0.0f;


	vx::DistanceConstraint joints[] =
	{
		vx::DistanceConstraint(body_a, body_b, hardbar_constraint_settings),
		vx::DistanceConstraint(body_b, body_c, hardbar_constraint_settings),
		vx::DistanceConstraint(body_c, body_d, hardbar_constraint_settings),
		vx::DistanceConstraint(body_d, body_a, hardbar_constraint_settings),

		vx::DistanceConstraint(body_a, body_c, hardbar_constraint_settings),
		vx::DistanceConstraint(body_b, body_d, hardbar_constraint_settings),

		vx::DistanceConstraint(body_a1, body_b1, hardbar_constraint_settings),
		vx::DistanceConstraint(body_b1, body_c1, hardbar_constraint_settings),
		vx::DistanceConstraint(body_c1, body_d1, hardbar_constraint_settings),
		vx::DistanceConstraint(body_d1, body_a1, hardbar_constraint_settings),

		vx::DistanceConstraint(body_a1, body_c1, hardbar_constraint_settings),
		vx::DistanceConstraint(body_b1, body_d1, hardbar_constraint_settings),

		vx::DistanceConstraint(body_a, body_a1, hardbar_constraint_settings),
		vx::DistanceConstraint(body_b, body_b1, hardbar_constraint_settings),
		vx::DistanceConstraint(body_c, body_c1, hardbar_constraint_settings),
		vx::DistanceConstraint(body_d, body_d1, hardbar_constraint_settings),

		vx::DistanceConstraint(body_c, body_d1, hardbar_constraint_settings),
		vx::DistanceConstraint(body_d, body_c1, hardbar_constraint_settings),

		vx::DistanceConstraint(body_a, body_b1, hardbar_constraint_settings),
		vx::DistanceConstraint(body_b, body_a1, hardbar_constraint_settings),

		vx::DistanceConstraint(body_a, body_d1, hardbar_constraint_settings),
		vx::DistanceConstraint(body_d, body_a1, hardbar_constraint_settings),

		vx::DistanceConstraint(body_b, body_c1, hardbar_constraint_settings),
		vx::DistanceConstraint(body_c, body_b1, hardbar_constraint_settings),
	};
	mPhysicsWorld->CreateConstraintsT(joints, 24);
}
