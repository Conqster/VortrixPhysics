#include "WorldQueriesScenario.h"
#include <external/imgui/imgui.h>

#include "Vortrix/PhysicsWorld.h"

#include "SampleFramework/EditorUI.h"

#include <Vortrix/Visuals/Renderers.h>
#include "Vortrix/Collision/Shapes/SphereShape.h"
#include "Vortrix/Collision/Shapes/CapsuleShape.h"
#include "Vortrix/Dynamics/Body/Body.h"



void WorldQueriesScenario::Init(vx::PhysicsWorld* i_world)
{
	Scenario::Init(i_world);

	//if (mAppCamera)
	//{
	//	vx::Vec3 pos = vx::Vec3(-14.65, 12.30, -17.44);
	//	vx::Vec3 fwd = vx::Vec3(0.72, -0.32, 0.62);
	//	vx::Vec3 up = vx::Vec3(0.25, 0.95, 0.21);
	//	Camera::State cam_state = Camera::State(pos, fwd, up);
	//	mAppCamera->SetState(cam_state);
	//}

	mExperimentRay = vx::RayCast(vx::Vec3(-15.0f, 0.5f, 0.0f), vx::Vec3(30.0f, 0.0f, 0.0f));

	vx::Ref<vx::SphereShape> unit_sphere = vx::MakeRef<vx::SphereShape>(0.5f);
	vx::BodySettings settings = vx::BodySettings::DefaultDynamicConstruct();
	settings.position = vx::Vec3(-12.0f, 1.0f, 0.0f);
	settings.debug_name = "sphere";
	settings.shape = unit_sphere;
	mPhysicsWorld->CreateBody(settings);


	settings.position = vx::Vec3(-10.0f, 1.0f, 0.0f);
	mPhysicsWorld->CreateBody(settings);

	settings.position = vx::Vec3(-6.0f, 1.0f, 0.0f);
	mPhysicsWorld->CreateBody(settings);

	settings.position = vx::Vec3(-3.0f, 1.0f, 0.0f);
	mPhysicsWorld->CreateBody(settings);


	settings.debug_name = "Capsule";
	settings.shape = vx::MakeRef<vx::CapsuleShape>(0.5f, 0.5f);
	settings.orientation.SetAxisAngle(vx::Vec3::Right(), vx::DegToRad(90.0f));


	settings.position = vx::Vec3(1.0f, 1.0f, -0.5f);
	mPhysicsWorld->CreateBody(settings);

	settings.position = vx::Vec3(4.0f, 1.0f, 0.0f);
	mPhysicsWorld->CreateBody(settings);
	settings.position = vx::Vec3(7.0f, 1.0f, 1.0f);
	mPhysicsWorld->CreateBody(settings);

	settings.orientation = vx::Quat::Identity();


	CreateGroundPlane(100.0f);
}



void WorldQueriesScenario::CastRayClosest()
{
	vx::ClosestRaycastHitProcessor processor;
	mPhysicsWorld->GetWorldQuery().CastRay(mExperimentRay, processor);


	if (mDebugGizmos)
	{
		vx::Colour ray_colour = (processor.HasHit()) ? vx::Colour::sRed : vx::Colour::sGreen;

		vx::Vec3 ray_end_point = mExperimentRay.End();
		if (processor.HasHit())
		{
			vx::RaycastHit hit = processor.Hit();

			//Vec3 point = mExperimentRay.origin + (mExperimentRay.displacement * hit.fraction);
			vx::Vec3 point = mExperimentRay.PointAlongRay(hit.fraction);
			ray_end_point = point;

			vx::Body body = mPhysicsWorld->GetBodyManager().GetBody(hit.body);
			mDebugGizmos->DrawAABB(body.GetAABBWorld(), vx::Colour::sDeepTeal);

			mDebugGizmos->DrawAACross(point, GetBasisAxisColourArray().data(), 3, 0.2f);
			mDebugGizmos->DrawArrowCone(point, point + hit.normal, 0.05, 0.1, 0.05, 3, vx::Colour::sCyan);
		}


		mDebugGizmos->DrawLine(mExperimentRay.origin, ray_end_point, ray_colour);
		if (!mExperimentRay.End().IsApprox(ray_end_point))
			mDebugGizmos->DrawLine(ray_end_point, mExperimentRay.End(), vx::Colour::sGreen);
	}
}

void WorldQueriesScenario::CastRayAny()
{
	vx::AnyRaycastHitProcessor processor;
	mPhysicsWorld->GetWorldQuery().CastRay(mExperimentRay, processor);


	if (mDebugGizmos)
	{
		vx::Colour ray_colour = (processor.HasHit()) ? vx::Colour::sRed : vx::Colour::sGreen;

		//vx::Vec3 ray_end_point = mExperimentRay.End();
		//if (processor.HasHit())
		//{
		//	vx::RaycastHit hit = processor.Hit();
		//	Vec3 point = mExperimentRay.origin + (mExperimentRay.displacement * hit.fraction);
		//	//ray_end_point = point;
		//	//mDebugGizmos->DrawAACross(point, GetBasisAxisColourArray().data(), 3, 0.2f);
		//	//mDebugGizmos->DrawArrowCone(point, point + hit.normal, 0.05, 0.1, 0.05, 3, vx::Colour::sCyan);
		//}


		mDebugGizmos->DrawLine(mExperimentRay.origin, mExperimentRay.End(), ray_colour);
	}
}

void WorldQueriesScenario::CastRayAll()
{
	vx::AllRaycastHitProcessor<16> processor;
	mPhysicsWorld->GetWorldQuery().CastRay(mExperimentRay, processor);


	if (mDebugGizmos)
	{
		vx::Colour ray_colour = (processor.HasHit()) ? vx::Colour::sRed : vx::Colour::sGreen;
		vx::Vec3 ray_end_point = mExperimentRay.End();

		if (processor.HasHit())
		{
			processor.Sort();
			vx::RaycastResult result = processor.Result();
			vx::RaycastHit last_hit = result.hits[result.hitCount - 1];

			//ray_end_point = mExperimentRay.origin + (mExperimentRay.displacement * last_hit.fraction);
			ray_end_point = mExperimentRay.PointAlongRay(last_hit.fraction);

			//mDebugGizmo
			//last and first
			vx::Body body = mPhysicsWorld->GetBodyManager().GetBody(last_hit.body);
			mDebugGizmos->DrawAABB(body.GetAABBWorld(), vx::Colour::sGreen);
			mDebugGizmos->DrawAACross(ray_end_point, GetBasisAxisColourArray().data(), 3, 0.2f);
			mDebugGizmos->DrawArrowCone(ray_end_point, ray_end_point + last_hit.normal, 0.05, 0.1, 0.05, 3, vx::Colour::sCyan);

			//first
			//vx::RaycastHit hit = processor.Hits()[0];
			vx::RaycastHit hit = result.hits[0];
			body = mPhysicsWorld->GetBodyManager().GetBody(hit.body);
			mDebugGizmos->DrawAABB(body.GetAABBWorld(), vx::Colour::sPurple);

			vx::Vec3 point = mExperimentRay.PointAlongRay(hit.fraction);
			mDebugGizmos->DrawAACross(point, GetBasisAxisColourArray().data(), 3, 0.2f);
			mDebugGizmos->DrawArrowCone(point, point + hit.normal, 0.05, 0.1, 0.05, 3, vx::Colour::sCyan);

	
			for (int i = 1; i < result.hitCount-1; ++i)
			{
				vx::RaycastHit hit = processor.Hits()[i];
				//vx::RaycastHit hit = result.hits[i];

				vx::Body body = mPhysicsWorld->GetBodyManager().GetBody(hit.body);
				mDebugGizmos->DrawAABB(body.GetAABBWorld(), vx::Colour::sDeepTeal);

				vx::Vec3 point = mExperimentRay.PointAlongRay(hit.fraction);
				mDebugGizmos->DrawAACross(point, GetBasisAxisColourArray().data(), 3, 0.2f);
				mDebugGizmos->DrawArrowCone(point, point + hit.normal, 0.05, 0.1, 0.05, 3, vx::Colour::sCyan);
			}
		}
		mDebugGizmos->DrawLine(mExperimentRay.origin, ray_end_point, ray_colour);

		if (!mExperimentRay.End().IsApprox(ray_end_point))
			mDebugGizmos->DrawLine(ray_end_point, mExperimentRay.End(), vx::Colour::sGreen);
	}
}



void WorldQueriesScenario::PostPhysicsStep(float dt)
{
	Scenario::PostPhysicsStep(dt);


	switch (mQueryType)
	{
	case EQueryType::CastRayClosest: CastRayClosest();
		break;
	case EQueryType::CastRayAny: CastRayAny();
		break;
	case EQueryType::CastRayAll: CastRayAll();
		break;
	}


	//ray start and end
	if (mDebugGizmos)
	{
		vx::Colour c = vx::Colour::sGreen;
		mDebugGizmos->DrawAACross(mExperimentRay.origin, &c, 1, 0.5);
		c = vx::Colour::sCyan;
		mDebugGizmos->DrawAACross(mExperimentRay.End(), &c, 1, 0.5);
	}
}

void WorldQueriesScenario::OnUI()
{
	Scenario::OnUI();

	if (mPhysicsWorld == nullptr || mAppUI == nullptr)
		return;

	if (mAppUI->BeginPanel("##World Queries Scenario", nullptr,
		350, 150, EditorUI::EPanelScreenAnchor::HackTopRightConsiderPrimaryPanel,
		1.0f / vx::Vec2(4.0f, 5.0f), ImGuiWindowFlags_NoTitleBar))
	{
		EditorUI::TextColoured(EditorUI::kAccent, "World Queries Scenario");
		EditorUI::Combo("Query Type", mQueryType, kQueryTypeNames);
		bool changed = ImGui::DragFloat3("Ray Origin", &mExperimentRay.origin[0], 0.1f);
		vx::Vec3 dir = mExperimentRay.displacement;// .Normalised();
		changed |= ImGui::DragFloat3("Ray displacement", &dir[0], 0.1f);
		//float len = mExperimentRay.displacement.Length();
		//changed |= ImGui::DragFloat("Max Distance", &len, 0.1f);
		if (changed)	
			mExperimentRay = vx::RayCast(mExperimentRay.origin, dir/* * len*/);

		ImGui::End();
	}
}
