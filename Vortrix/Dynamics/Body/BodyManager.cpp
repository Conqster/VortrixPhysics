#include "BodyManager.h"

#include "Body.h"
#include "BodySimStats.h"

#include "Vortrix/Collision/Shapes/Shape.h"

#include <set>

#include "Vortrix/Core/Profiler.h"
namespace vx
{
	void BodyManager::Init(uint32 max_bodies)
	{
		mMaxBodies = max_bodies;
		mBodies.reserve(mMaxBodies);
		mBodiesDebugInfo.reserve(mMaxBodies);

		//may be pass active body ratio
		mMaxActiveBodies = max_bodies;// *0.5f;
		mActiveBodies = new BodyID[mMaxActiveBodies];

		uint32 min_free_list = 64;
		mFreedIndices.reserve(min_free_list);
		mBodyIdxGenerations.resize(mMaxBodies, 0);
	}
	BodyManager::~BodyManager()
	{
		/// JAY: move to smart pointer; ref counted
		/// solve with set for now 
		/// because multiple bodies might be sharing 
		/// a shape
		//std::set<GetShape*> shapes;
		//for (auto& b : mBodies)
		//	shapes.insert(b.mShape);
		//
		//for(auto& s : shapes)
		//		delete s;

		delete[] mActiveBodies;
	}
	const BodyID BodyManager::AddBody(const BodySettings& body_setting)
	{
		if(mBodies.size() >= mMaxBodies - 1 && mFreedIndices.empty())
		{
			VX_LOG_WARN("Body manager body limit attained");
			return BodyID();
		}

		vx::Body body = Body(body_setting.position);

		body.mOrientation = body_setting.orientation;
		body.mFriction = body_setting.friction;
		body.mRestitution = body_setting.restitution;


		if (body_setting.motionType == EMotionType::Dynamic)
		{
			SetBodyMotionType<EMotionType::Dynamic>(body, false);

			body.mLinearDamping = body_setting.linearDamping;
			body.mAngularDamping = body_setting.angularDamping;

			body.mMaxLinearVelocity = body_setting.maxLinearVelocity;		
			body.mMaxAngularVelocity = body_setting.maxAngularVelocity;

			body.mAllowedDynamicsDof = body_setting.degreeFreedom;
		}
		else
		{
			SetBodyMotionType<EMotionType::Static>(body, false);
			body.mAwake = false;
		}

		
		if(body_setting.overrideMasses)
		{
			/// Set shape handles the bodies shape 
			/// as well as bounds 
			body.SetShape(body_setting.shape, false);
			body.SetMass(body_setting.mass);
			body.SetInertiaTensor(Vec3::LoadFloat3Raw(body_setting.inertia));
		}
		else
			body.SetShape(body_setting.shape, true);


		/// Post body setup 
		if (!body_setting.intialVelocity.IsZero())
			body.SetLinearVelocity(body_setting.intialVelocity);
		if (!body_setting.impluse.IsZero())
			body.ApplyImpulse(body_setting.impluse);

		//since orientation might have change percompute bounds
		body.ComputeWorldSpaceBoundsInternal();

		BodyID id = AddBody(body);
		bool success = id.IsValid();

		/// TO-DO: might want to check, if old slot 
		/// is available
		if (id.Generation() <= 1U)
			mBodiesDebugInfo.emplace_back();

		success &= body.ID().Idx() < mBodiesDebugInfo.size();
		VX_ASSERT_WARN(success, "Invalid Body creation or Miss-matching id for body & body debug");
		StackString<40> _s(body_setting.debug_name);
		_s << "_body_" << body.ID().ID() << "_idx_" << body.ID().Idx();
		mBodiesDebugInfo[body.ID().Idx()].name = _s;

		//if (success && body_setting.motionType == EMotionType::Dynamic)
		//	ActivateBodies(&id, 1);

		return id;
	}
	const BodyID BodyManager::AddBody(Body& _body)
	{
		if (_body.ID().IsValid())
			return _body.ID();


		BodyID id = BodyID();
		if (!mFreedIndices.empty())
		{
			uint32 idx = mFreedIndices[0];
			std::swap(mFreedIndices[0], mFreedIndices.back());
			mFreedIndices.pop_back();
			
			uint8 gen = GetBodyIdxNextGeneration(idx);
			id = BodyID(idx, gen);
			_body.mID = id;
			mBodies[idx] = _body;
		}
		else
		{
			uint32 idx = static_cast<uint32>(mBodies.size());

			uint8 gen = GetBodyIdxNextGeneration(idx);
			id = BodyID(idx, gen);
			_body.mID = id;
			mBodies.emplace_back(_body);
		}
		return id;
	}

	template<EMotionType Type>
	void BodyManager::SetBodyMotionType(Body& body, bool update_mass_inertia)
	{
		body.mSleepTimer = 0.0f;

		body.SetMotionType(Type);

		///if not it must means that 
		// caller would set the mass properties after Motion type
		if constexpr (Type == EMotionType::Dynamic)
		{
			body.mAwake = true;
			if (update_mass_inertia)
			{
				MassProperties mp = body.GetShape()->GetMassProperties();
				body.SetMass(mp.mass);
				body.SetInertiaTensor(Vec3::LoadFloat3Raw(mp.inertialTensorDiagonal));
			}
		}
		else if constexpr (Type == EMotionType::Static)
		{
			body.mAwake = false;
			if (update_mass_inertia)
			{
				body.SetMass(0.0f);
				body.SetInertiaTensor(Vec3::Zero());
			}
		}
	}

	void BodyManager::SetBodyShape(Body& body, RefConst<Shape> shape, bool update_mass_inertia)
	{
		body.SetShape(shape, update_mass_inertia);
	}

	void BodyManager::RemoveBody(const BodyID& id)
	{
		VX_ASSERT(id.IsValid(), "Attempting to remove invalid body");
		VX_ASSERT_WARN_VOID(mBodies[id.Idx()].mID.IsValid(), "Attempting to remove invalid body");

		//remove from broadphase 
		mFreedIndices.push_back(id.Idx());
		mBodies[id.Idx()].mID = BodyID();
		//this would affect shape 

		/// also make sure, to remove debug info 
		/// 
		mBodiesDebugInfo[id.Idx()] = BodyDebug();
	}

	const BodyDebug& BodyManager::GetBodyDebugInfo(const Body& body) const
	{
		return GetBodyDebugInfo(body.ID());
	}

	BodyDebug& BodyManager::GetBodyDebugInfo(const Body& body)
	{
		return GetBodyDebugInfo(body.ID());
	}

	const char* BodyManager::GetBodyDebugName(const Body& body) const
	{
		return GetBodyDebugName(body.ID());
	}

	const BodySimStats& BodyManager::GetBodySimStats(const Body& body) const
	{
		return GetBodySimStats(body.ID());
	}

	BodySimStats& BodyManager::GetBodySimStats(const Body& body)
	{
		return GetBodySimStats(body.ID());
		//return mBodiesDebugInfo[body.ID().Value()].simulationStats;
	}

	void BodyManager::UpdateBodyVelocitySimStat(const Body& body)
	{
		//BodySimStats& sim_stat = mBodiesDebugInfo[body.ID().Value()].simulationStats;
		
		BodySimStats& sim_stat = GetBodySimStats(body.ID());

		sim_stat.maxAttainedLinearVelocitySq = VxMax(sim_stat.maxAttainedLinearVelocitySq, body.LinearVelocity().LengthSq());
		sim_stat.maxAttainedAngularVelocitySq = VxMax(sim_stat.maxAttainedAngularVelocitySq, body.AngularVelocity().LengthSq());
	}

	void BodyManager::UpdateBodiesActiveState(float dt, const SleepingSettings& sleeping_setting)
	{
		VX_PROFILE_FUNCTION();
		mNumActiveBodies = 0;
		for (auto& body : GetBodies())
		{
			body.SetIndexInActiveBodies(Body::kInvalidActiveIdx);

			//quick hack 
			if (!body.ID().IsValid())
				continue;

			if (sleeping_setting.enable)
				body.UpdateSleepState(dt, sleeping_setting);

			if (body.IsAwake())
			{
				VX_ASSERT_WARN(mNumActiveBodies < mMaxActiveBodies, "Reach max bodies limits");
				if (mNumActiveBodies >= mMaxActiveBodies) continue;
				body.SetIndexInActiveBodies(mNumActiveBodies);
				mActiveBodies[mNumActiveBodies++] = body.ID();
			}
			else
				body.SetIndexInActiveBodies(Body::kInvalidActiveIdx);
		}



		//VX_PROFILE_FUNCTION();

		//uint32* deactive_body = (uint32*)VX_STACK_ALLOC(mNumActiveBodies * sizeof(uint32));
		//int body_remove = 0;

		//for (size_t i = 0; i < mNumActiveBodies; ++i)
		//{
		//	const auto& active_body_id = mActiveBodies[i];
		//	VX_ASSERT(active_body_id.IsValid());

		//	auto& body = GetBody(active_body_id);

		//	body.UpdateSleepState(dt, sleeping_setting);
		//	if (!body.IsAwake()) //later body should not have awake flag
		//		deactive_body[body_remove++] = i;
		//}

		//for (size_t i = 0; i < body_remove; ++i)
		//{
		//	uint32_t idx = deactive_body[i];
		//	if(idx < mNumActiveBodies-1)
		//		std::swap(mActiveBodies[idx], mActiveBodies[mNumActiveBodies-1]);
		//	mActiveBodies[mNumActiveBodies - 1] = BodyID();
		//	mNumActiveBodies--;
		//}



	}

	void BodyManager::ActivateBodies(const BodyID* body_ids, uint32 count)
	{
#if TEST_CONTACT_CONSTRAINT_MT
		std::lock_guard lock(mBodiesActivationMutex);
#endif // TEST_CONTACT_CONSTRAINT_MT

		VX_ASSERT(body_ids && count > 0);

		for (uint32 i = 0; i < count; ++i)
		{
			VX_ASSERT(mNumActiveBodies < mMaxActiveBodies, "Reach max active bodies limits");
			if (mNumActiveBodies >= mMaxActiveBodies) return;

			BodyID id = body_ids[i];
			if (!id.IsValid())
			{
				VX_LOG_WARN("Invalid id for boddy activation");
				continue;
			}

			auto& body = GetBody(id);

			if (body.IsStatic()) continue;

			body.SetIndexInActiveBodies(mNumActiveBodies);
			body.WakeUp();
			mActiveBodies[mNumActiveBodies++] = id;
		}

	}

	void BodyManager::AddBodiesToActivate(bool activate_sleeping)
	{
		for (auto& body : GetBodies())
		{
			//quick hack 
			if (!body.ID().IsValid())
				continue;

			VX_ASSERT_WARN(mNumActiveBodies < mMaxActiveBodies, "Reach max bodies limits");
			if (mNumActiveBodies >= mMaxActiveBodies) return;

			if (body.IsAwake())
			{
				body.SetIndexInActiveBodies(mNumActiveBodies);
				mActiveBodies[mNumActiveBodies++] = body.ID();
			}
			else if (activate_sleeping)
			{
				body.SetIndexInActiveBodies(mNumActiveBodies);
				mActiveBodies[mNumActiveBodies++] = body.ID();

				body.WakeUp();
			}
			else
			{
				/// not awake invalidate idxs
				body.SetIndexInActiveBodies(Body::kInvalidActiveIdx);
				body.SetIslandIndex(Body::kInvalidIslandIdx);
			}

		}
	}



	//void BodyManager::DeactivateBodies(BodyID* bodies_id, uint32 count)
	//{
	//	for (BodyID* body_id = bodies_id, *end_body_id = bodies_id + count;
	//		body_id < end_body_id; ++body_id)
	//	{

	//	}
	//}

}