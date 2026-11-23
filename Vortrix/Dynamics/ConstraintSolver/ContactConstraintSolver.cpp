#include "ContactConstraintSolver.h"

#include "Vortrix/Dynamics/Body/Body.h"
#include "Vortrix/Collision/Shapes/Shape.h"

#include "Vortrix/Visuals/Renderers.h"

#include "Vortrix/SimulationContexts.h"

#include "Vortrix/Dynamics/ConstraintSolver.h"


#include "Vortrix/Dynamics/IslandCoordinator.h"

#include "Vortrix/PhysicsWorld.h"
#include "Vortrix/Core/Colours.h"

namespace vx {


	void ContactConstraintSolver::ContactPointConstraint::SetupAxesVelocityConstraints(const Body& body0, const Body& body1, 
		const Vec3& world_pos0, const Vec3& world_pos1, const Vec3& normal, float e, float mu, const ContactConstraintAxesSetting& settings)
	{
		/// only accounting dynamic bodies 
		bool body0_nonstatic = !body0.IsStatic();
		bool body1_nonstatic = !body1.IsStatic();

		const float inv0 = (body0_nonstatic) ? body0.InverseMass() : 0.0f;
		const float inv1 = (body1_nonstatic) ? body1.InverseMass() : 0.0f;

		float total_inv_mass = inv0 + inv1;

		//contact average pt 
		Vec3 p = (world_pos0 + world_pos1) * 0.5f;
		/// point relative to bodies
		Vec3 r0 = p - body0.Position();
		Vec3 r1 = p - body1.Position();


		Vec3 rel_vel = Vec3::Zero();
		Vec3 potential_rel_acc = Vec3::Zero();


		Vec3 gravity_dt = settings.gravity * settings.timeStep;
		float gravity_dt_along_nor = gravity_dt.Dot(normal);
		//float gravity_dt_along_nor;

		/// normal axis properties
		{
			/// inverse effective mass: K = J M^-1 J^T
			float inv_effective_mass = total_inv_mass;

			if (body0_nonstatic)
			{
				rel_vel = -body0.GetPointVelocityRelCOM(r0);

				Vec3 r0_X_n = r0.Cross(normal);
				r0_X_n.Store(this->normal.r0XAxis);

				Vec3 inv_Ir0_n = body0.ComputeInvInteriaWorld().Multiply3x3(r0_X_n);
				inv_Ir0_n.Store(this->normal.invIr0XAxis);

				inv_effective_mass += inv_Ir0_n.Dot(r0_X_n);

				potential_rel_acc -= settings.gravity - (Vec3::LoadFloat3Raw(body0.AccumulatedForce()) * body0.InverseMass());
			}


			if (body1_nonstatic)
			{
				rel_vel += body1.GetPointVelocityRelCOM(r1);

				Vec3 r1_X_n = r1.Cross(normal);
				r1_X_n.Store(this->normal.r1XAxis);

				Vec3 inv_Ir1_n = body1.ComputeInvInteriaWorld().Multiply3x3(r1_X_n);
				inv_Ir1_n.Store(this->normal.invIr1XAxis);

				inv_effective_mass += inv_Ir1_n.Dot(r1_X_n);

				potential_rel_acc += settings.gravity + (Vec3::LoadFloat3Raw(body0.AccumulatedForce()) * body0.InverseMass());
			}
			if (inv_effective_mass > 0)
				this->normal.effMass = 1.0f / inv_effective_mass;

			normal.Store(this->normal.axis);


			float penetration = (world_pos0 - world_pos1).Dot(normal);

			float restitution_bias = 0.0f;
			//relative velocity along normal
			float rel_velN = rel_vel.Dot(normal);
			//restitution_bias = VxMax(0.0f, -penetration / settings.timeStep);
			if (e > 0 && rel_velN < -settings.restitutionThreshold)
				restitution_bias = e * rel_velN;// *penetration;


			this->normal.bias = restitution_bias;
		}


		if (mu > 0)
		{
			Vec3 tangent_axis[2];
			GetTangentBasis(normal, &rel_vel, tangent_axis[0], tangent_axis[1]);

#if VX_DEBUG_DRAW
			if(settings.debug_renderer && settings.debugDrawAxes)
			{
				settings.debug_renderer->DrawArrowCone(p,
					p + normal,
					0.05f, 0.1, 0.05f, 3, Colour::sGreen);

				settings.debug_renderer->DrawArrowCone(p,
					p + tangent_axis[1],
					0.05f, 0.1, 0.05f, 3, Colour::sBlue);

				settings.debug_renderer->DrawArrowCone(p,
					p + tangent_axis[0],
					0.05f, 0.1, 0.05f, 3, Colour::sRed);
			}
#endif // VX_DEBUG_DRAW



			/// tangential axes constriat properties
			for (int i = 0; i < 2; ++i)
			{
				const Vec3& axis = tangent_axis[i];
				ConstraintAxis& constraint_axis = lateralTangent[i];
				axis.Store(constraint_axis.axis);

				/// inverse effective mass: K = J M^-1 J^T
				float inv_effective_mass = total_inv_mass;

				if (body0_nonstatic)
				{
					Vec3 r0_X_axis = r0.Cross(axis);
					r0_X_axis.Store(constraint_axis.r0XAxis);

					Vec3 inv_Ir0_axis = body0.ComputeInvInteriaWorld().Multiply3x3(r0_X_axis);
					inv_Ir0_axis.Store(constraint_axis.invIr0XAxis);

					inv_effective_mass += inv_Ir0_axis.Dot(r0_X_axis);
				}


				if (body1_nonstatic)
				{
					Vec3 r1_X_axis = r1.Cross(axis);
					r1_X_axis.Store(constraint_axis.r1XAxis);

					Vec3 inv_Ir1_axis = body1.ComputeInvInteriaWorld().Multiply3x3(r1_X_axis);
					inv_Ir1_axis.Store(constraint_axis.invIr1XAxis);

					inv_effective_mass += inv_Ir1_axis.Dot(r1_X_axis);
				}

				if(inv_effective_mass > 0)
					constraint_axis.effMass = 1.0f / inv_effective_mass;
				//constraint_axis.totalLambda = 0.0f;

				/// surface does not have velocity
				constraint_axis.bias = 0.0f;
				//constraint_axis.bias = -1.0f;

				/////surface velicuty
				//if (!body1_nonstatic)
				//{
				//	Vec3 local_space_vel = Vec3::Forward() * 2.0f;
				//	Vec3 linear_surface_vel = body1.Orientation() * local_space_vel;

				//	constraint_axis.bias = axis.Dot(linear_surface_vel);
				//}

			}
		}

	}


	void ContactConstraintSolver::Init(uint32 max_constraints)
	{
		mMaxConstraints = VxMin(max_constraints, kConstraintLimit);
		mConstraints = new ContactConstraint[max_constraints];
		//mCachePoints = new CacheContactConstraint[max_constraints];

		//mManifoldCache->Init(max_constraints);
		mManifoldCache[0].Init(max_constraints);
		mManifoldCache[1].Init(max_constraints);

		mStepWriteManifoldCache = new CachedManifold[max_constraints];

		mMaxCacheContactPoints = max_constraints * 4; //if all boxes this create up to 4 contact points

		if (!IsPowerof2(mMaxCacheContactPoints))
		{
			uint32 v = RoundUpPowerof2(mMaxCacheContactPoints);
			VX_LOG_INFO(mMaxCacheContactPoints, " Was not a power to two, rounding up ", v);
			mMaxCacheContactPoints = v;
		}

		VX_ASSERT(IsPowerof2(mMaxCacheContactPoints), "mMaxCacheContactPoints is needs to be power of 2");
		mCacheContactPoint = new CacheContactPoint[mMaxCacheContactPoints];

		/// at start tail is equal to the total count available 
		/// or maybe half of total to make as its double linear buffer
		/// 
		/// after first step head = last + 1 from last step (i.e current location) 
		/// tail end or start of last step (i.e last step/read manifold cache)
		mCacheContactPointTail = mMaxCacheContactPoints;
	}

	void ContactConstraintSolver::SetupContactConstraint(const ContactManifold& _manifold)
	{
#if TEST_CONTACT_CONSTRAINT_MT
		SetupContactConstraint2Mt(_manifold);
#else
		SetupContactConstraint2(_manifold);
#endif // TEST_CONTACT_CONSTRAINT_MT
	}


	ContactConstraintSolver::CachedManifold* ContactConstraintSolver::CreateNewManifold(const BodyPair key, BodyID a_id, BodyID b_id, uint32 num_contact_pts)
	{
		ManifoldMap& write_manifold_cache = mManifoldCache[mManifoldWriteCache];
		ManifoldMapEntry new_manifold_entry = write_manifold_cache.Create(key, CachedManifold(a_id, b_id, num_contact_pts));

		VX_ASSERT_WARN_RETURN(new_manifold_entry.Valid(), nullptr, "unable to create new cache manifold entry");

		return &new_manifold_entry.Value();
	}



	void ContactConstraintSolver::SetupContactConstraint2(const ContactManifold& _manifold)
	{
		/// now for debugginf cache
		VX_PROFILE_FUNCTION();

		ContactManifold manifold = _manifold;

		/// for determintic simulation, enforce that 
		/// 1. if particpating bodies are a dynamic against a static
		/// 2. if both dynamic, for consitency id a < b
		/// 
		/// ensure that the A is the dynamic while B is the static 
		int priority_a = static_cast<int>(manifold.a->MotionType());
		int priority_b = static_cast<int>(manifold.b->MotionType());

		/// 1. dynamic > static
		if (priority_a < priority_b)
			manifold.Swap();
		else if (priority_a == priority_b)
		{
			/// 2. dynamic - dynamic 
			//if (manifold.a->ID() < manifold.b->ID())
			//	manifold.Swap();

			if (manifold.a->ID() > manifold.b->ID())
				manifold.Swap();
		}
		

		VX_ASSERT_WARN_VOID(manifold.a && manifold.b, "Either Bodies to not exists");

		/// vaild tests 
		/// dyn - dyn 
		/// dyn - static 
		VX_ASSERT_WARN(manifold.a->IsDynamic(), "Contact Manifold body A is Static, while B is Dynamic");


		//both bodies need to be sorted and valid up to this point
		BodyPair key = BodyPair::Create(manifold.a->ID(), manifold.b->ID());

		uint32 num_contact_pts = VxMin(manifold.mPointCount, kMaxPoints);

		CachedManifold* new_manifold = CreateNewManifold(key, manifold.a->ID(), manifold.b->ID(), num_contact_pts);
		VX_ASSERT_WARN_VOID(new_manifold != nullptr, "unable to create new cache manifold entry");

		/// since body 2 is less dominates to 1 either static if static is part of 
		/// participating body
		Vec3 norBl = manifold.b->Orientation().InverseRotate(manifold.normal);
		norBl.Store(new_manifold->mNormal);
	


		//read manifold map 
		ManifoldMap& read_manifold_cache = mManifoldCache[mManifoldWriteCache ^ 1];
		ManifoldMapEntry old_manifold_entry = read_manifold_cache.Find(key);



		const Body& body0 = *manifold.a;
		const Body& body1 = *manifold.b;

		const CacheContactPoint* cache_pt_start;
		uint32 cache_pt_count;
		//persistent
		if (old_manifold_entry.Valid())
		{
			CachedManifold* old_manifold = &old_manifold_entry.Value();
			cache_pt_start = old_manifold->ContactPointPtr();
			cache_pt_count = old_manifold->NumPoints();
			old_manifold->mPersistent = true;
		}
		else
		{
			//not persistent, new 
			cache_pt_start = nullptr;
			cache_pt_count = 0;
		}



		////create constraint
		////allocate mem
		//uint32 idx = mNumConstraints;
		//VX_ASSERT_WARN_VOID(idx < mMaxConstraints, "Max frame contact constraint attianed returning");
		////mConstraints[idx] = {};
		//ContactConstraint& constraint = mConstraints[idx];
		//mNumConstraints++;
		//mStats.numContactConstraints++;

		uint32 constraint_idx = mNumConstraints.fetch_add(1, std::memory_order_relaxed);
		VX_ASSERT_WARN_VOID(constraint_idx < mMaxConstraints, "Max frame contact constraint attianed returning");
		ContactConstraint& constraint = mConstraints[constraint_idx];
		mStats.numContactConstraints++; //need to fix


#if CONTACT_USE_SOLVERBODY
		VX_ASSERT_WARN_VOID(mPhysicsContext->constraintSolver, "trying to setup constact constraint from manifold, but solver/builder not available");


		{
			SolverBodyIndex solver_body_idx0 = mPhysicsContext->constraintSolver->GetOrCreateSolverBody(*manifold.a);
			SolverBodyIndex solver_body_idx1 = mPhysicsContext->constraintSolver->GetOrCreateSolverBody(*manifold.b);

			constraint.SetBodies(solver_body_idx0, solver_body_idx1);

			/// in some cases; both are dynamic but one might be active while the other is not 
			/// wake up bodies if sleeping
			/// 
			uint32 bodies_activate_count = 0;
			BodyID body_ids[2];

			if (manifold.a->IsDynamic() && !manifold.a->IsAwake())
				body_ids[bodies_activate_count++] = manifold.a->ID();
			if (manifold.b->IsDynamic() && !manifold.b->IsAwake())
				body_ids[bodies_activate_count++] = manifold.b->ID();

			if (bodies_activate_count > 0)
				mPhysicsContext->bodyManager->ActivateBodies(body_ids, bodies_activate_count);


			/// a is alway dynam,ic
			if (manifold.a->IsDynamic() && manifold.b->IsDynamic())
			{
				VX_ASSERT(manifold.a->GetIndexInActiveBodies() != Body::kInvalidActiveIdx && manifold.b->GetIndexInActiveBodies() != Body::kInvalidActiveIdx, "Invalid Body index");
				mPhysicsContext->mIslandCoordinator->LinkBodies(manifold.a->GetIndexInActiveBodies(), manifold.b->GetIndexInActiveBodies());
			}

			if (manifold.a->IsDynamic())
				mPhysicsContext->mIslandCoordinator->LinkContactConstraint(constraint_idx, manifold.a->GetIndexInActiveBodies());
			else if (manifold.b->IsDynamic())
				mPhysicsContext->mIslandCoordinator->LinkContactConstraint(constraint_idx, manifold.b->GetIndexInActiveBodies());
			else
				VX_ASSERT(false);
		}

#else
		constraint.SetBodies(manifold.a, manifold.b);
#endif // CONTACT_USE_SOLVERBODY


		float fricition_coeff = CombinedCoefficient::GetFriction(mCombinedFrictionMode, body0, body1);
		float restitution_coff = CombinedCoefficient::GetRestitution(mCombinedRestitutionMode, body0, body1);
		constraint.FrictionCoeff(fricition_coeff);
		constraint.RestitutionCoeff(restitution_coff);

		manifold.normal.Normalise();
		constraint.Normal(manifold.normal);

		Quat qA = manifold.a->Orientation();
		Vec3 tA = manifold.a->Position();
		Quat qB = manifold.b->Orientation();
		Vec3 tB = manifold.b->Position();


		ContactConstraintAxesSetting constraint_axes_setting;
#if VX_DEBUG_DRAW
		constraint_axes_setting.debug_renderer = mPhysicsContext->mDebugRenderer;
		constraint_axes_setting.debugDrawAxes = mPhysicsContext->mDrawSettings->drawContactConstraintSolverTBNs;
#endif // VX_DEBUG_DRAW
		constraint_axes_setting.timeStep = mPhysicsContext->mDeltaTime;

		const auto& physics_setting = mPhysicsContext->mPhysicsWorld->Settings();
		constraint_axes_setting.restitutionThreshold = physics_setting->solver.restitutionThreshold;
		constraint_axes_setting.positionCorrectionSlop = physics_setting->solver.positionCorrectionSlop;
		constraint_axes_setting.baumgarte = physics_setting->solver.baumgarte;
		//constraint_axes_setting.maxSpeed = physics_setting->solver

		//create cache constrain
		//allocate mem
		//uint32 _idx = mNumCachePoints;
		//mNumCachePoints++;
		//VX_ASSERT_WARN_VOID(_idx < mMaxConstraints, "Max frame contact constraint attianed returning");
		//mCachePoints[_idx] = {};
		//CacheContactConstraint& cache_constraint = mCachePoints[_idx];
		///the above should become a linear array of actual points, that each system could point to
		///but at the moment, i am using the new cached manifolds std::array

		//transfer points 
		mStats.actualPointCounts = cache_pt_count;
		for (int i = 0; i < num_contact_pts; ++i)
		{
			const ManifoldPoint& mp = manifold.Points()[i];

			///constraint point constraitn part
			//ContactPointConstraint& point_constraint = constraint.contactPoints[constraint.numConstraintPoints++];
			ContactPointConstraint& point_constraint = *constraint.CreatePointConstraint();


			Vec3 p0_ls = qA.InverseRotate(mp.pointA - tA);
			Vec3 p1_ls = qB.InverseRotate(mp.pointB - tB);

			//check if close to any contact pt if any
			bool was_close = false;
			for (const CacheContactPoint* cache_pt = cache_pt_start;
				cache_pt < (cache_pt_start + cache_pt_count); cache_pt++)
			{
				if (p0_ls.IsApprox(Vec3::LoadFloat3Raw(cache_pt->localPoint0), VxSqr(0.01)) &&
					p1_ls.IsApprox(Vec3::LoadFloat3Raw(cache_pt->localPoint1), VxSqr(0.01)))
				{
					point_constraint.normal.totalLambda = cache_pt->totalNormalLambda;
					point_constraint.lateralTangent[0].totalLambda = cache_pt->totalTangentLambda[0];
					point_constraint.lateralTangent[1].totalLambda = cache_pt->totalTangentLambda[1];

					was_close = true;
					mStats.actualPersistentPointCounts++;
					break;
				}
			}

			if (!was_close)
			{
				point_constraint.normal.totalLambda = 0.0f;
				point_constraint.lateralTangent[0].totalLambda = 0.0f;
				point_constraint.lateralTangent[1].totalLambda = 0.0f;
			}

			/// now only copy the local points 
			CacheContactPoint& cp = new_manifold->ContactPointPtr()[i];
			p0_ls.Store(cp.localPoint0);
			p1_ls.Store(cp.localPoint1);


			//solving constraint point also points to the cache 
			point_constraint.cacheLocalPoint = &cp;


			//Vec3 pt = mp.pointB;
			////hack to ensure right penetration for now
			//float depth_sq = (mp.pointA - mp.pointB).LengthSq();
			//if (depth_sq < mp.peneration)
			//	pt = mp.pointA + manifold.normal * mp.peneration;

			point_constraint.SetupAxesVelocityConstraints(*manifold.a, *manifold.b, mp.pointA, mp.pointB, manifold.normal, restitution_coff, fricition_coeff, constraint_axes_setting);
		}

		VX_ASSERT_WARN(num_contact_pts == constraint.NumConstraintPoints(), "Number of point stored needs to be equal compute attainable");
	}


	void ContactConstraintSolver::SetupContactConstraint2Mt(const ContactManifold& _manifold)
	{
		VX_PROFILE_FUNCTION();
		///Copy manifold for easy modification
		ContactManifold manifold = _manifold;

		/// for determintic simulation, enforce that 
		/// 1. if particpating bodies are a dynamic against a static
		/// 2. if both dynamic, for consitency id a < b
		/// 
		/// ensure that the A is the dynamic while B is the static 
		int priority_a = static_cast<int>(manifold.a->MotionType());
		int priority_b = static_cast<int>(manifold.b->MotionType());

		/// 1. dynamic > static
		if (priority_a < priority_b)
			manifold.Swap();
		else if (priority_a == priority_b)
		{
			/// 2. dynamic - dynamic 
			//if (manifold.a->ID() < manifold.b->ID())
			//	manifold.Swap();

			if (manifold.a->ID() > manifold.b->ID())
				manifold.Swap();
		}

		VX_ASSERT_WARN_VOID(manifold.a && manifold.b, "Either Bodies to not exists");

		/// vaild tests 
		/// dyn - dyn 
		/// dyn - static 
		VX_ASSERT_WARN(manifold.a->IsDynamic(), "Contact Manifold body A is Static, while B is Dynamic");





		/// Create new manifold during setup
		//uint32 new_manifold_cache_idx = mWriteManifoldCacheIdx.fetch_add(1, std::memory_order_relaxed);
		/// should not try to copy the address (ref to element) as this might thrash cache line on every modification 
		/// of ref
		/// so it better to full complete manifold cache generation
		
		uint32 num_contact_pts = VxMin(manifold.mPointCount, kMaxPoints);

		CachedManifold new_manifold_cache(manifold.a->ID(), manifold.b->ID(), num_contact_pts);

		/// befoer this 
		/// 	CacheContactPoint& cp = new_manifold->ContactPointPtr()[i];
		///		p0_ls.Store(cp.localPoint0);
		///		p1_ls.Store(cp.localPoint1);
		/// 
		/// create required local pts with 
		///acquire solts 
		//uint32 required_count = num_contact_pts;
		//uint32 contact_pt_head_idx = mCacheContactPointHead.fetch_add(required_count, std::memory_order_relaxed);

		//VX_LOG_INFO("Last head indx, ", contact_pt_head_idx);
		//VX_LOG_INFO("wrap returned, ", WrapPowerof2(contact_pt_head_idx, mMaxCacheContactPoints - 1)); //this might break, if we are at the end it would break
		////as it need to jump to the start not contigous. quick solution; detect break then jump to sytart intead
		/////quick sample 
		//uint32 new_end = mCacheContactPointHead.load(std::memory_order_relaxed);
		//if (WrapPowerof2(new_end, mMaxCacheContactPoints - 1) < WrapPowerof2(contact_pt_head_idx, mMaxCacheContactPoints - 1))
		//{
		//	///wrap inbetween required range 
		//	if (mCacheContactPointHead.compare_exchange_strong(new_end, 0 + required_count)) ///<-- already wrap failed so much easiler making 0 over max plus required slots 
		//	{
		//		/// but if failed acq new value and atomic increment,
		//		/// another thread as progress
		//		contact_pt_head_idx = mCacheContactPointHead.fetch_add(required_count, std::memory_order_relaxed);
		//	}
		//}
		//CacheContactPoint* contact_pt_head_pt = &mCacheContactPoint[WrapPowerof2(contact_pt_head_idx, mMaxCacheContactPoints - 1)];

		uint32 required_count = num_contact_pts;
		uint32 contact_pt_head_idx;
		for (;;)
		{
			uint32 head = mCacheContactPointHead.load(std::memory_order_relaxed);
			uint32 wrapped = WrapPowerof2(head, mMaxCacheContactPoints - 1);

			uint32 new_buff_head;
			uint32 alloc_head;

			///check if fits before end
			if (wrapped + required_count <= mMaxCacheContactPoints)
			{
				alloc_head = wrapped;
				new_buff_head = head + required_count;
			}
			else //would fit, skip to very beginning of buffer
			{
				alloc_head = 0;
				new_buff_head = head + (mMaxCacheContactPoints - wrapped) + required_count;
			}

			if (mCacheContactPointHead.compare_exchange_weak(head, new_buff_head, std::memory_order_relaxed))
			{
				contact_pt_head_idx = alloc_head;
				break;
			}
		}

		CacheContactPoint* contact_pt_head_pt = &mCacheContactPoint[contact_pt_head_idx];
		new_manifold_cache.SetContactPointPtr(contact_pt_head_pt);
		//VX_ASSERT(contact_pt_head_pt->totalNormalLambda == 0.0f, (StackString<8>()<< contact_pt_head_pt->totalNormalLambda).Data());


		/// since body 2 is less dominates to 1 either static if static is part of 
		/// participating body
		Vec3 norBl = manifold.b->Orientation().InverseRotate(manifold.normal);
		norBl.Store(new_manifold_cache.mNormal);

		BodyPair key = new_manifold_cache.CreatePairKey();

		/// read cache manifold map for persistency 
		ManifoldMap& read_manifold_cache = mManifoldCache[mManifoldWriteCache ^ 1];
		ManifoldMapEntry old_manifold_entry = read_manifold_cache.Find(key);

		const Body& body0 = *manifold.a;
		const Body& body1 = *manifold.b;

		const CacheContactPoint* cache_pt_start;
		uint32 cache_pt_count;
		//persistent
		if (old_manifold_entry.Valid())
		{
			CachedManifold* old_manifold = &old_manifold_entry.Value();
			cache_pt_start = old_manifold->ContactPointPtr();
			cache_pt_count = old_manifold->NumPoints();
			//old_manifold->mPersistent = true;
		}
		else
		{
			//not persistent, new 
			cache_pt_start = nullptr;
			cache_pt_count = 0;
		}




		//create constraint
//allocate mem
		uint32 constraint_idx = mNumConstraints.fetch_add(1, std::memory_order_relaxed);
		VX_ASSERT_WARN_VOID(constraint_idx < mMaxConstraints, "Max frame contact constraint attianed returning");
		ContactConstraint& constraint = mConstraints[constraint_idx];
		mStats.numContactConstraints++; //need to fix


#if CONTACT_USE_SOLVERBODY
		VX_ASSERT_WARN_VOID(mPhysicsContext->constraintSolver, "trying to setup constact constraint from manifold, but solver/builder not available");


		{
			SolverBodyIndex solver_body_idx0 = mPhysicsContext->constraintSolver->GetOrCreateSolverBody(*manifold.a);
			SolverBodyIndex solver_body_idx1 = mPhysicsContext->constraintSolver->GetOrCreateSolverBody(*manifold.b);

			constraint.SetBodies(solver_body_idx0, solver_body_idx1);

			/// in some cases; both are dynamic but one might be active while the other is not 
			/// wake up bodies if sleeping
			/// 
			
			uint32 bodies_activate_count = 0;
			BodyID body_ids[2];

			if (manifold.a->IsDynamic() && !manifold.a->IsAwake())
				body_ids[bodies_activate_count++] = manifold.a->ID();
			if (manifold.b->IsDynamic() && !manifold.b->IsAwake())
				body_ids[bodies_activate_count++] = manifold.b->ID();

			if (bodies_activate_count > 0)
				mPhysicsContext->bodyManager->ActivateBodies(body_ids, bodies_activate_count);


			/// a is alway dynam,ic
			if (manifold.a->IsDynamic() && manifold.b->IsDynamic())
			{
				VX_ASSERT(manifold.a->GetIndexInActiveBodies() != Body::kInvalidActiveIdx && manifold.b->GetIndexInActiveBodies() != Body::kInvalidActiveIdx, "Invalid Body index");
				mPhysicsContext->mIslandCoordinator->LinkBodies(manifold.a->GetIndexInActiveBodies(), manifold.b->GetIndexInActiveBodies());
			}


			if (manifold.a->IsDynamic())
				mPhysicsContext->mIslandCoordinator->LinkContactConstraint(constraint_idx, manifold.a->GetIndexInActiveBodies());
			else if (manifold.b->IsDynamic())
				mPhysicsContext->mIslandCoordinator->LinkContactConstraint(constraint_idx, manifold.b->GetIndexInActiveBodies());
			else
				VX_ASSERT(false);
		}


#else
		constraint.SetBodies(manifold.a, manifold.b);
#endif // CONTACT_USE_SOLVERBODY


		float fricition_coeff = CombinedCoefficient::GetFriction(mCombinedFrictionMode, body0, body1);
		float restitution_coff = CombinedCoefficient::GetRestitution(mCombinedRestitutionMode, body0, body1);
		constraint.FrictionCoeff(fricition_coeff);
		constraint.RestitutionCoeff(restitution_coff);

		manifold.normal.Normalise();
		constraint.Normal(manifold.normal);

		Quat qA = manifold.a->Orientation();
		Vec3 tA = manifold.a->Position();
		Quat qB = manifold.b->Orientation();
		Vec3 tB = manifold.b->Position();


		ContactConstraintAxesSetting constraint_axes_setting;
#if VX_DEBUG_DRAW
		constraint_axes_setting.debug_renderer = mPhysicsContext->mDebugRenderer;
		constraint_axes_setting.debugDrawAxes = mPhysicsContext->mDrawSettings->drawContactConstraintSolverTBNs;
#endif // VX_DEBUG_DRAW
		constraint_axes_setting.timeStep = mPhysicsContext->mDeltaTime;

		const auto& physics_setting = mPhysicsContext->mPhysicsWorld->Settings();
		constraint_axes_setting.restitutionThreshold = physics_setting->solver.restitutionThreshold;
		constraint_axes_setting.positionCorrectionSlop = physics_setting->solver.positionCorrectionSlop;
		constraint_axes_setting.baumgarte = physics_setting->solver.baumgarte;
		//constraint_axes_setting.maxSpeed = physics_setting->solver


		//transfer points 
		mStats.actualPointCounts = cache_pt_count;
		for (int i = 0; i < num_contact_pts; ++i)
		{
			const ManifoldPoint& mp = manifold.Points()[i];

			///constraint point constraitn part
			//ContactPointConstraint& point_constraint = constraint.contactPoints[constraint.numConstraintPoints++];
			ContactPointConstraint& point_constraint = *constraint.CreatePointConstraint();


			Vec3 p0_ls = qA.InverseRotate(mp.pointA - tA);
			Vec3 p1_ls = qB.InverseRotate(mp.pointB - tB);

			//check if close to any contact pt if any
			bool was_close = false;
			for (const CacheContactPoint* cache_pt = cache_pt_start;
				cache_pt < (cache_pt_start + cache_pt_count); cache_pt++)
			{
				if (p0_ls.IsApprox(Vec3::LoadFloat3Raw(cache_pt->localPoint0), VxSqr(0.01)) &&
					p1_ls.IsApprox(Vec3::LoadFloat3Raw(cache_pt->localPoint1), VxSqr(0.01)))
				{
					point_constraint.normal.totalLambda = cache_pt->totalNormalLambda;
					point_constraint.lateralTangent[0].totalLambda = cache_pt->totalTangentLambda[0];
					point_constraint.lateralTangent[1].totalLambda = cache_pt->totalTangentLambda[1];

					was_close = true;
					mStats.actualPersistentPointCounts++;
					break;
				}
			}

			if (!was_close)
			{
				point_constraint.normal.totalLambda = 0.0f;
				point_constraint.lateralTangent[0].totalLambda = 0.0f;
				point_constraint.lateralTangent[1].totalLambda = 0.0f;
			}



			/// now only copy the local points 
			CacheContactPoint& cp = new_manifold_cache.ContactPointPtr()[i];
			p0_ls.Store(cp.localPoint0);
			p1_ls.Store(cp.localPoint1);

			cp.totalNormalLambda = point_constraint.normal.totalLambda;
			cp.totalTangentLambda[0] = point_constraint.lateralTangent[0].totalLambda;
			cp.totalTangentLambda[1] = point_constraint.lateralTangent[1].totalLambda;


			//solving constraint point also points to the cache 
			point_constraint.cacheLocalPoint = &cp; ///later make this const to prevent modification outside


			//Vec3 pt = mp.pointB;
			////hack to ensure right penetration for now
			//float depth_sq = (mp.pointA - mp.pointB).LengthSq();
			//if (depth_sq < mp.peneration)
			//	pt = mp.pointA + manifold.normal * mp.peneration;

			point_constraint.SetupAxesVelocityConstraints(*manifold.a, *manifold.b, mp.pointA, mp.pointB, manifold.normal, restitution_coff, fricition_coeff, constraint_axes_setting);
		}


		/// now the Manifold Cache; is ready
		uint32 new_manifold_cache_idx = mWriteManifoldCacheIdx.fetch_add(1, std::memory_order_relaxed);
		mStepWriteManifoldCache[new_manifold_cache_idx] = new_manifold_cache;

		VX_ASSERT_WARN(num_contact_pts == constraint.NumConstraintPoints(), "Number of point stored needs to be equal compute attainable");
		
	}

	void ContactConstraintSolver::WarmStart()
	{
		VX_ASSERT(false, "Out of service!!!!"); //need house keep last frame cache frame etc

#if !CONTACT_USE_SOLVERBODY
		//for a warm start up use info from last frame 
		for (uint32 contact_idx = 0; contact_idx < mNumConstraints; ++contact_idx)
		{
			auto& constraint_info = mConstraints[contact_idx];//constraint info
			if (!constraint_info.body0 || !constraint_info.body1 || constraint_info.numConstraintPoints < 0)
			{
				VX_ASSERT_WARN(constraint_info.body0, "Body 0 is invalid");
				VX_ASSERT_WARN(constraint_info.body1, "Body 1 is invalid");
				VX_ASSERT_WARN(constraint_info.numConstraintPoints > 0, "No contact points");
				continue;
			}

			Body* bodyA = constraint_info.body0;
			Body* bodyB = constraint_info.body1;

			bool dyn_a = (bodyA->IsDynamic());
			bool dyn_b = (bodyB->IsDynamic());

			Vec3 n = Vec3::LoadFloat3Raw(constraint_info.normal);
			n.Normalise();
			//contact basis


			Vec3 tangents[2];
			for (int i = 0; i < constraint_info.numConstraintPoints; ++i)
			{
				auto& pt = constraint_info.contactPoints[i];

				tangents[0] = Vec3::LoadFloat3Raw(pt.lateralTangent[0].axis);
				tangents[1] = Vec3::LoadFloat3Raw(pt.lateralTangent[1].axis);
				Vec3 impluse = (n * pt.normal.totalLambda) +
					(tangents[0] * pt.lateralTangent[0].totalLambda) +
					(tangents[1] * pt.lateralTangent[1].totalLambda);


				//handle this differenely make use of the arm for right displacement
				bodyA->ApplyImpulse(-impluse * constraint_info.invMass0, Vec3::LoadFloat3Raw(pt.cacheLocalPoint->localPoint0));
				if(dyn_b)
					bodyB->ApplyImpulse(impluse * constraint_info.invMass1, Vec3::LoadFloat3Raw(pt.cacheLocalPoint->localPoint1));
			}
		}
#endif // !CONTACT_USE_SOLVERBODY
	}


	void ContactConstraintSolver::DebugDraw(DebugGizmosRenderer* debug_renderer, const ConstraintSolver* constraint_solver, const BodyManager* body_manager, const DrawSettings& settings) const
	{
		VX_PROFILE_FUNCTION();
#if !CONTACT_USE_SOLVERBODY


		/// Keep in mind for manifold debug
		/// the manifold points are store in world space 
		/// 
		/// making most point invalid after simulation step 
		/// as when velocity & positional contrainst are solved 
		/// bodies might have moved away for manifold points 
		/// 
		/// need to improve this later......
		auto draw_contact_manifold = [&](const ContactConstraint& constraint) {

			//Vec3 half_extent(mSettings.drawContactPointSize * 0.5f);

			//Draw bodies bound particapting 
			if (settings.drawCollidingPairRefAndInc)
			{
				AABB aabb = constraint.body0->ComputeAABBWorld();
				debug_renderer->DrawAABB(aabb.mMin, aabb.mMax, settings.collidingPairRefColour, false);
				aabb = constraint.body1->ComputeAABBWorld();
				debug_renderer->DrawAABB(aabb.mMin, aabb.mMax, settings.collidingPairIncColour, false);
			}

			if (!settings.drawContacts)return;

			Vec3 n = Vec3::LoadFloat3Raw(constraint.normal);

			Mat44 transform0 = constraint.body0->ComputeWorldTransform();
			Mat44 transform1 = constraint.body1->ComputeWorldTransform();
			//for (const auto& [pointA, peneration, pointB] : manifold.points)
			for (int i = 0; i < constraint.numConstraintPoints; ++i)
			{
				auto& contact_point = constraint.contactPoints[i];
				Vec3 pointA = transform0.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint0));
				Vec3 pointB = transform1.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint1));

				//const Vec3 p0 = transform0.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint0));
				//const Vec3 p1 = transform1.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint1));

				//hack penetration 
				float penetration = (pointA - pointB).Dot(n);

				//if draw as aabb
				//Vec3 min = point - half_extent;
				//Vec3 max = point + half_extent;

				//debug_renderer->DrawAABB(min.AsGLM(), max.AsGLM(), ColourToGLM(mSettings.drawContactPointColour));
				Colour col = settings.drawContactPointColour;
				//glm::vec3 col = ColourToGLM(mSettings.drawContactPointColour);

				/// point A
				debug_renderer->DrawLine(
					(pointA - Vec3(settings.drawContactPointSize, 0.0f, 0.0f)),
					(pointA + Vec3(settings.drawContactPointSize, 0.0f, 0.0f)),
					col);
				debug_renderer->DrawLine(
					(pointA - Vec3(0.0f, settings.drawContactPointSize, 0.0f)),
					(pointA + Vec3(0.0f, settings.drawContactPointSize, 0.0f)),
					col);

				col = Colour::sMagenta;
				/// point B
				debug_renderer->DrawLine(
					(pointB - Vec3(settings.drawContactPointSize, 0.0f, 0.0f)),
					(pointB + Vec3(settings.drawContactPointSize, 0.0f, 0.0f)),
					col);
				debug_renderer->DrawLine(
					(pointB - Vec3(0.0f, settings.drawContactPointSize, 0.0f)),
					(pointB + Vec3(0.0f, settings.drawContactPointSize, 0.0f)),
					col);

				if (settings.drawContactsNormals)
				{
					float depth = penetration;
					if (ManifoldPoint::kUseNewManifoldPt)
						//depth = (pointB - point).Length();
						depth = (pointA - pointB).Length();
					const float size = (settings.drawContactsNormalsWithPeneration) ? depth * settings.drawContactNormalSize :
						settings.drawContactNormalSize;

					//debug_renderer->DrawLine(point.AsGLM(), end.AsGLM(), ColourToGLM(mSettings.drawContactNormalsColour));
					//debug_renderer->DrawLine(point, end, mSettings.drawContactNormalsColour);

					//just for old manifold point 
					const Vec3 pt0 = pointA;
					const Vec3 pt1 = (ManifoldPoint::kUseNewManifoldPt) ? pointB : pointA - n * penetration;

					const Vec3 end = pt0 + n * size;

					debug_renderer->DrawArrowCone(pt0, end, 0.02f, 0.05f, 0.02f, 3, settings.drawContactNormalsColour);
					//This line need to match or almost match he above
					debug_renderer->DrawLine(pt1, pt0, vx::Colour::sTurquoise);
				}
			}
			};


		for (uint32 contact_idx = 0; contact_idx < mNumConstraints; ++contact_idx)
		{
			draw_contact_manifold(mConstraints[contact_idx]);
		}
#else
/// Keep in mind for manifold debug
		/// the manifold points are store in world space 
		/// 
		/// making most point invalid after simulation step 
		/// as when velocity & positional contrainst are solved 
		/// bodies might have moved away for manifold points 
		/// 
		/// need to improve this later......
auto draw_contact_manifold = [&](const ContactConstraint& constraint) {

	//Vec3 half_extent(mSettings.drawContactPointSize * 0.5f);
	const Body* body0 = &constraint_solver->AttemptGetBody(body_manager, constraint.BodyA());
	const Body* body1 = &constraint_solver->AttemptGetBody(body_manager, constraint.BodyB());

	//Draw bodies bound particapting 
	if (settings.drawCollidingPairRefAndInc)
	{
		AABB aabb = body0->GetAABBWorld();
		debug_renderer->DrawAABB(aabb.mMin, aabb.mMax, settings.collidingPairRefColour, false);
		aabb = body1->GetAABBWorld();
		debug_renderer->DrawAABB(aabb.mMin, aabb.mMax, settings.collidingPairIncColour, false);
	}

	if (!settings.drawContacts && !settings.drawContactConstraintSolverTBNs)return;

	Vec3 n = constraint.Normal();

	Mat44 transform0 = body0->ComputeWorldTransform();
	Mat44 transform1 = body1->ComputeWorldTransform();
	//for (const auto& [pointA, peneration, pointB] : manifold.points)
	for (int i = 0; i < constraint.NumConstraintPoints(); ++i)
	{
		auto& contact_point = constraint.PointConstraint(i);
		Vec3 pointA = transform0.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint0));
		Vec3 pointB = transform1.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint1));




		//const Vec3 p0 = transform0.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint0));
		//const Vec3 p1 = transform1.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint1));

		//hack penetration 
		float penetration = (pointA - pointB).Dot(n);


		if (settings.drawContactConstraintSolverTBNs)
		{
			Vec3 normal = Vec3::LoadFloat3Raw(contact_point.normal.axis);
			Vec3 t0 = Vec3::LoadFloat3Raw(contact_point.lateralTangent[0].axis);
			Vec3 t1 = Vec3::LoadFloat3Raw(contact_point.lateralTangent[1].axis);

			Vec3 p = (pointA + pointA) * 0.5f;

			float scale = 1.0f;
			{

				if (settings.scaleAxesWithDepth)
					//scale = VxAbs((pointB - pointA).Length());
					scale = VxAbs(penetration);
				else
					scale = settings.drawContactConstraintSolverTBNsScale;
			}

			debug_renderer->DrawArrowCone(p,
				p + (normal * scale),
				0.05f, 0.1, 0.05f, 3, Colour::sGreen);

			debug_renderer->DrawArrowCone(p,
				p + (t0 * scale),
				0.05f, 0.1, 0.05f, 3, Colour::sBlue);

			debug_renderer->DrawArrowCone(p,
				p + (t1 * scale),
				0.05f, 0.1, 0.05f, 3, Colour::sRed);
		}


		if (settings.drawContacts)
		{
			//if draw as aabb
			//Vec3 min = point - half_extent;
			//Vec3 max = point + half_extent;

			//debug_renderer->DrawAABB(min.AsGLM(), max.AsGLM(), ColourToGLM(mSettings.drawContactPointColour));
			Colour col = settings.drawContactPointColour;
			//glm::vec3 col = ColourToGLM(mSettings.drawContactPointColour);

			/// point A
			debug_renderer->DrawLine(
				(pointA - Vec3(settings.drawContactPointSize, 0.0f, 0.0f)),
				(pointA + Vec3(settings.drawContactPointSize, 0.0f, 0.0f)),
				col);
			debug_renderer->DrawLine(
				(pointA - Vec3(0.0f, settings.drawContactPointSize, 0.0f)),
				(pointA + Vec3(0.0f, settings.drawContactPointSize, 0.0f)),
				col);

			col = Colour::sMagenta;
			/// point B
			debug_renderer->DrawLine(
				(pointB - Vec3(settings.drawContactPointSize, 0.0f, 0.0f)),
				(pointB + Vec3(settings.drawContactPointSize, 0.0f, 0.0f)),
				col);
			debug_renderer->DrawLine(
				(pointB - Vec3(0.0f, settings.drawContactPointSize, 0.0f)),
				(pointB + Vec3(0.0f, settings.drawContactPointSize, 0.0f)),
				col);

			if (settings.drawContactsNormals)
			{
				float depth = penetration;
				//if (ManifoldPoint::kUseNewManifoldPt)
				//	//depth = (pointB - point).Length();
				//	depth = (pointA - pointB).Length();
				const float size = (settings.drawContactsNormalsWithPeneration) ? depth * settings.drawContactNormalSize :
					settings.drawContactNormalSize;

				//debug_renderer->DrawLine(point.AsGLM(), end.AsGLM(), ColourToGLM(mSettings.drawContactNormalsColour));
				//debug_renderer->DrawLine(point, end, mSettings.drawContactNormalsColour);

				//just for old manifold point 
				const Vec3 pt0 = pointA;
				//const Vec3 pt1 = (ManifoldPoint::kUseNewManifoldPt) ? pointB : pointA - n * penetration;
				const Vec3 pt1 = pointB;

				const Vec3 end = pt0 + n * size;

				debug_renderer->DrawArrowCone(pt0, end, 0.02f, 0.05f, 0.02f, 3, settings.drawContactNormalsColour);
				//This line need to match or almost match he above
				debug_renderer->DrawLine(pt1, pt0, vx::Colour::sTurquoise);
			}
		}
	}
	};


for (uint32 contact_idx = 0; contact_idx < mNumConstraints; ++contact_idx)
{
	draw_contact_manifold(mConstraints[contact_idx]);
}
#endif // !CONTACT_USE_SOLVERBODY
	}

	void ContactConstraintSolver::OnDraw(Renderer* renderer, const DrawSettings& settings) const
	{
		if (!settings.drawPenetrationText)
			return;
		for (const ContactConstraint* constraint = mConstraints, *constraint_end = mConstraints + mNumConstraints;
			constraint < constraint_end; ++constraint)
		{
			const Body* body_a = &mPhysicsContext->constraintSolver->AttemptGetBody(mPhysicsContext->bodyManager, constraint->BodyA());
			const Body* body_b = &mPhysicsContext->constraintSolver->AttemptGetBody(mPhysicsContext->bodyManager, constraint->BodyB());

			Mat44 transform_a = body_a->ComputeWorldTransform();
			Mat44 transform_b = body_b->ComputeWorldTransform();

			for (const ContactPointConstraint* contact_point = constraint->PointConstraintPtr(),
				*contact_point_end = constraint->PointConstraintPtr() + constraint->NumConstraintPoints();
				contact_point < contact_point_end; ++contact_point)
			{
				Vec3 normal = Vec3::LoadFloat3Raw(contact_point->normal.axis);


				Vec3 p;

				float depth = 1.0f;
				{
					Vec3 p0 = transform_a.Transform(Vec3::LoadFloat3Raw(contact_point->cacheLocalPoint->localPoint0));
					Vec3 p1 = transform_b.Transform(Vec3::LoadFloat3Raw(contact_point->cacheLocalPoint->localPoint1));

					p = (p0 + p1) * 0.5f;

					depth = (p0 - p1).Dot(normal);
				}

				renderer->DrawText3D((StackString<5>() << depth).Data(), p, 0.0025f, Colour::sGreen, ETextAlignment::Right);
				//renderer->DrawText3D((StackString<64>("depth along nor: ") << depth).Data(), p, 0.00125f, Colour::sGreen, ETextAlignment::Right);
			}
		}
	}


#if CONTACT_USE_SOLVERBODY
#else

	void ContactConstraintSolver::PositionalCorrection(ContactConstraint& constraint, float baumgarte, float slop, float min_limit, float max_limit, float limit_scale)
	{
		Body* a = constraint.body0;
		Body* b = constraint.body1;

		VX_ASSERT_WARN_VOID(a && b, "either bodies needs to be valid");

		bool dyn_a = a->IsDynamic();
		bool dyn_b = b->IsDynamic();

		VX_ASSERT_WARN_VOID(dyn_a || dyn_b, "not possible one of the bodies need to be non static");

		Vec3 n = Vec3::LoadFloat3Raw(constraint.normal);

		float correction_limit = 0.01;
		{
			float limitA = dyn_a ?
				a->GetShape()->HalfExtents().MinComponent() * limit_scale: kMaxf;
			float limitB = dyn_b ?								
				b->GetShape()->HalfExtents().MinComponent() * limit_scale: kMaxf;

			correction_limit = VxMin(limitA, limitB);

			correction_limit = VxClamp(correction_limit, min_limit, max_limit);
		}

		Mat44 transform0 = a->ComputeWorldTransform();
		Mat44 transform1 = b->ComputeWorldTransform();

		//effective mass 
		float total_inv_mass = constraint.invMass0 + constraint.invMass1;

		for (int i = 0; i < constraint.numConstraintPoints; ++i)
		{
			auto& contact_point = constraint.contactPoints[i];

			///New contact point in world as bodies position might have been corrected
			const Vec3 p0 = transform0.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint0));
			const Vec3 p1 = transform1.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint1));

			float seperation = (p1 - p0).Dot(n);
			///seperation constant
			/// dist along normal + slop 
			/// to avoid jittering between bodies
			//float C = VxMax(seperation + slop, -kEpsilon);
			float C = seperation + slop;

			if (C < 0.0f)
			{
				float inv_effective_mass = total_inv_mass;
				Vec3 p = (p0 + p1) * 0.5f;
				/// point relative to bodies
				Vec3 r0 = p - a->Position();
				Vec3 r1 = p - b->Position();

				Vec3 inv_Ir0_Xn, inv_Ir1_Xn;
				if (dyn_a)
				{
					Vec3 r0_X_n = r0.Cross(n);
					inv_Ir0_Xn = a->ComputeInvInteriaWorld().Multiply3x3(r0_X_n);
					inv_effective_mass += r0_X_n.Dot(inv_Ir0_Xn);
				}
				if (dyn_b)
				{
					Vec3 r1_X_n = r1.Cross(n);
					inv_Ir1_Xn = b->ComputeInvInteriaWorld().Multiply3x3(r1_X_n);
					inv_effective_mass += r1_X_n.Dot(inv_Ir1_Xn);
				}

				if (inv_effective_mass < 1e-9f)
					continue;

				C = VxMax(C, -correction_limit);
				float lambda = -(baumgarte * C) / inv_effective_mass;
				Vec3 lambda_vector = lambda * n;

				if (dyn_a)
				{
					a->ApplyLinearDisplacement(-lambda_vector * a->InverseMass());
					a->ApplyAngularDisplacement(-lambda * inv_Ir0_Xn);
				}
				if (dyn_b)
				{
					b->ApplyLinearDisplacement(lambda_vector * b->InverseMass());
					b->ApplyAngularDisplacement(lambda * inv_Ir1_Xn);
				}
			}
		}
	}

#endif // CONTACT_USE_SOLVERBODY


#if CONTACT_USE_SOLVERBODY


	void ContactConstraintSolver::WarmStart(const ContactConstraint& contact_constraint, SolverBody& body0, SolverBody& body1)
	{

		/// later queue, or sort, constaints with warm start lambdas 
		/// not just going through all. 
		/// 
		/// 
		for (const ContactPointConstraint* cpt_c = contact_constraint.PointConstraintPtr(),
			*cpt_c_end = contact_constraint.PointConstraintPtr() + contact_constraint.NumConstraintPoints();
			cpt_c < cpt_c_end; ++cpt_c)
		{
			float impluse;

			//tangents 
			for (int i = 0; i < 2; ++i)
			{
				impluse = cpt_c->lateralTangent[i].totalLambda;

				body0.v -= impluse * body0.invMass * Vec3::LoadFloat3Raw(cpt_c->lateralTangent[i].axis);
				body0.w -= impluse * Vec3::LoadFloat3Raw(cpt_c->lateralTangent[i].invIr0XAxis);

				body1.v += impluse * body1.invMass * Vec3::LoadFloat3Raw(cpt_c->lateralTangent[i].axis);
				body1.w += impluse * Vec3::LoadFloat3Raw(cpt_c->lateralTangent[i].invIr1XAxis);
			}

			impluse = cpt_c->normal.totalLambda;

			body0.v -= impluse * body0.invMass * Vec3::LoadFloat3Raw(cpt_c->normal.axis);
			body0.w -= impluse * Vec3::LoadFloat3Raw(cpt_c->normal.invIr0XAxis);

			body1.v += impluse * body1.invMass * Vec3::LoadFloat3Raw(cpt_c->normal.axis);
			body1.w += impluse * Vec3::LoadFloat3Raw(cpt_c->normal.invIr1XAxis);
		}




	}
	void ContactConstraintSolver::WarmStart(ContactConstraint* contact_constraints, size_t count, SolverBody* bodies)
	{
		VX_PROFILE_FUNCTION();
		for (ContactConstraint* c = contact_constraints,
			*c_end = contact_constraints + count; c < c_end; 
			++c)
		{
			SolverBody& sbA = bodies[c->BodyA().Value()];
			SolverBody& sbB = bodies[c->BodyB().Value()];

			WarmStart((*c), sbA, sbB);
		}
	}

	void ContactConstraintSolver::WarmStart(const uint32* constraint_indices, uint32 indices_count, const ContactConstraint* constraint_buff, uint32 total_constraints, SolverBody* solver_bodies, uint32 total_solver_bodies)
	{
		for (const uint32* idx = constraint_indices, *idx_end = constraint_indices + indices_count; idx < idx_end; ++idx)
		{
			VX_ASSERT((*idx) < total_constraints);

			const ContactConstraint* constraint = &constraint_buff[(*idx)];

			VX_ASSERT(constraint->BodyA().Value() < total_solver_bodies);
			VX_ASSERT(constraint->BodyB().Value() < total_solver_bodies);

			SolverBody& sbA = solver_bodies[constraint->BodyA().Value()];
			SolverBody& sbB = solver_bodies[constraint->BodyB().Value()];

			WarmStart(*constraint, sbA, sbB);
		}
	}

	void ContactConstraintSolver::SolveVelocityConstraint(ContactConstraint& constraint_info, SolverBody& sbA, SolverBody& sbB)
	{
		bool dyn_a = (sbA.invMass > 0);
		bool dyn_b = (sbB.invMass > 0);

		Vec3 n = constraint_info.Normal();
		VX_ASSERT(n.IsNormalised());

		//////Get velocities 
		Vec3 lin_vel0;
		Vec3 ang_vel0;

		if (dyn_a)
		{
			lin_vel0 = sbA.v;
			ang_vel0 = sbA.w;
		}

		Vec3 lin_vel1;
		Vec3 ang_vel1;

		if (dyn_b)
		{
			lin_vel1 = sbB.v;
			ang_vel1 = sbB.w;
		}


		for (int i = 0; i < constraint_info.NumConstraintPoints(); ++i)
		{
			auto& pt = constraint_info.PointConstraint(i);

			////////////////////////////
			////// Solve tangential axis
			////////////////////////////
			const float prev_nor_axis_total_lambda = pt.normal.totalLambda;
			VX_ASSERT(!VxIsNaN(prev_nor_axis_total_lambda) && !VxIsInf(prev_nor_axis_total_lambda));
			if (constraint_info.FrictionCoeff() > 0.0f)
			{
				float max_friction = constraint_info.FrictionCoeff() * prev_nor_axis_total_lambda;

				for (int i = 0; i < 2; ++i)
				{
					ContactPointConstraint::ConstraintAxis& axis_contraint = pt.lateralTangent[i];
					const Vec3& axis = Vec3::LoadFloat3Raw(axis_contraint.axis);// = tangents[i];

					float jv;
					if (dyn_a && dyn_b)
						jv = (lin_vel0 - lin_vel1).Dot(axis);
					else if (dyn_a)
						jv = lin_vel0.Dot(axis);
					else if (dyn_b)
						jv = (-lin_vel1).Dot(axis);
					else
					{
						VX_ASSERT_WARN(false, "Static vs static this should not be possible");
						jv = 0.0f;
					}


					//simplify 
					if (dyn_a)
						jv += Vec3::LoadFloat3Raw(axis_contraint.r0XAxis).Dot(ang_vel0);
					if (dyn_b)
						jv -= Vec3::LoadFloat3Raw(axis_contraint.r1XAxis).Dot(ang_vel1);

					//float lambda = contact_info.friction * axis_contraint.effectiveMass * jv;
					//float lambda = contact_info.friction * 0.5f * axis_contraint.effectiveMass * jv;

					//ignore surface relative velocity
					//float lambda = jv * axis_contraint.effMass;
					float lambda = (jv - axis_contraint.bias) * axis_contraint.effMass;


					float old_lambda = axis_contraint.totalLambda;
					//ensure non negative
					axis_contraint.totalLambda = VxClamp(old_lambda + lambda, -max_friction, max_friction);
					//updated jn
					float impluse = axis_contraint.totalLambda - old_lambda;

					//store changes
					if (dyn_a)
					{
						lin_vel0 -= impluse * sbA.invMass * axis;
						ang_vel0 -= impluse * Vec3::LoadFloat3Raw(axis_contraint.invIr0XAxis);
					}
					if (dyn_b)
					{
						lin_vel1 += impluse * sbB.invMass * axis;
						ang_vel1 += impluse * Vec3::LoadFloat3Raw(axis_contraint.invIr1XAxis);
					}
				}
			}


			//////////////////////////
			//// Solve normal/penetration axis
			//////////////////////////
			ContactPointConstraint::ConstraintAxis& nor_axis_contraint = pt.normal;
			// 
			{
				VX_ASSERT(Vec3::LoadFloat3Raw(nor_axis_contraint.axis).IsApprox(n));
				/// curreny relative velocity (J * v) 
				float jn;
				if (dyn_a && dyn_b)
					jn = (lin_vel0 - lin_vel1).Dot(n);
				else if (dyn_a)
					jn = lin_vel0.Dot(n);
				else if (dyn_b)
					jn = (-lin_vel1).Dot(n);
				else
				{
					VX_ASSERT_WARN(false, "Static vs static this should not be possible");
					jn = 0.0f;
				}
				//simplify 
				if (dyn_a)
					jn += Vec3::LoadFloat3Raw(nor_axis_contraint.r0XAxis).Dot(ang_vel0);
				if (dyn_b)
					jn -= Vec3::LoadFloat3Raw(nor_axis_contraint.r1XAxis).Dot(ang_vel1);

				/// -K^-1(Jv + b)
				/// -K^-1((1-e)Jv)
				/// nor_axis_contraint.effectiveMass = 1/inv effective mass
				//float lambda = (nor_axis_contraint.bias - jn) * nor_axis_contraint.effectiveMass;
				float lambda = (jn - nor_axis_contraint.bias) * nor_axis_contraint.effMass;

				float old_lambda = nor_axis_contraint.totalLambda;
				//ensure non negative
				nor_axis_contraint.totalLambda = VxMax(old_lambda + lambda, 0.0f);
				//updated jn
				float impluse = nor_axis_contraint.totalLambda - old_lambda;

				//store changes
				if (dyn_a)
				{
					lin_vel0 -= impluse * sbA.invMass * n;
					ang_vel0 -= impluse * Vec3::LoadFloat3Raw(nor_axis_contraint.invIr0XAxis);
				}
				if (dyn_b)
				{
					lin_vel1 += impluse * sbB.invMass * n;
					ang_vel1 += impluse * Vec3::LoadFloat3Raw(nor_axis_contraint.invIr1XAxis);
				}

			}


			////set velocities; prevent multiple bodies value value changes
			/// and heavy torque level & world moment inetria internal to bodies ApplyImpluse
			if (dyn_a)
			{
				VX_ASSERT(!lin_vel0.IsNaN(), "lin_vel0 is nan");
				VX_ASSERT(!ang_vel0.IsNaN(), "ang_vel0 is nan");
				sbA.v = lin_vel0;
				sbA.w = ang_vel0;
			}

			if (dyn_b)
			{
				VX_ASSERT(!lin_vel1.IsNaN(), "lin_vel1 is nan");
				VX_ASSERT(!ang_vel1.IsNaN(), "ang_vel1 is nan");
				sbB.v = lin_vel1;
				sbB.w = ang_vel1;
			}
		}
	}

	


	void ContactConstraintSolver::SolveVelocityConstraint(SolverBody* bodies)
	{
		VX_PROFILE_FUNCTION();
		//curr_manifold_idx = -1;
		//for (auto& contact_info : manifolds)
		for (uint32 contact_idx = 0; contact_idx < mNumConstraints; ++contact_idx)
		{
			auto& constraint_info = mConstraints[contact_idx];//constraint info

			SolverBodyIndex& body0 = constraint_info.BodyA();
			SolverBodyIndex& body1 = constraint_info.BodyB();

			if (!body0.IsValid() || !body1.IsValid() || constraint_info.NumConstraintPoints() < 0)
			{
				VX_ASSERT_WARN(body0.IsValid(), "Body 0 is invalid");
				VX_ASSERT_WARN(body1.IsValid(), "Body 1 is invalid");
				VX_ASSERT_WARN(constraint_info.NumConstraintPoints() > 0, "No contact points");
				continue;
			}


			SolverBody& sbA = bodies[body0.Value()];
			SolverBody& sbB = bodies[body1.Value()];

			SolveVelocityConstraint(constraint_info, sbA, sbB);
		}
	}
	void ContactConstraintSolver::SolveVelocityConstraint(const uint32* constraint_start_idx, uint32 count, SolverBody* bodies)
	{
		VX_PROFILE_FUNCTION();
		for (const uint32* idx = constraint_start_idx, *idx_end = constraint_start_idx + count; idx < idx_end; ++idx)
		{

		

			auto& constraint_info = mConstraints[(*idx)];//constraint info

			SolverBodyIndex& body0 = constraint_info.BodyA();
			SolverBodyIndex& body1 = constraint_info.BodyB();

			if (!body0.IsValid() || !body1.IsValid() || constraint_info.NumConstraintPoints() < 0)
			{
				VX_ASSERT_WARN(body0.IsValid(), "Body 0 is invalid");
				VX_ASSERT_WARN(body1.IsValid(), "Body 1 is invalid");
				VX_ASSERT_WARN(constraint_info.NumConstraintPoints() > 0, "No contact points");
				continue;
			}


			SolverBody& sbA = bodies[body0.Value()];
			SolverBody& sbB = bodies[body1.Value()];


			SolveVelocityConstraint(constraint_info, sbA, sbB);
		}
	}

	void ContactConstraintSolver::SolvePositionCorrection(const ContactConstraint& constraint, Body* bodyA, Body* bodyB, float total_inv_mass, float baumgarte, float slop, float min_limit, float max_limit, float limit_scale)
	{
		bool dyn_a = bodyA->IsDynamic();
		bool dyn_b = bodyB->IsDynamic();

		Vec3 n = constraint.Normal();

		float correction_limit = 0.01;
		{
			float limitA = dyn_a ?
				bodyA->GetShape()->HalfExtents().MinComponent() * limit_scale : kMaxf;
			float limitB = dyn_b ?
				bodyB->GetShape()->HalfExtents().MinComponent() * limit_scale : kMaxf;

			correction_limit = VxMin(limitA, limitB);

			correction_limit = VxClamp(correction_limit, min_limit, max_limit);
		}

		Mat44 transform0 = bodyA->ComputeWorldTransform();
		Mat44 transform1 = bodyB->ComputeWorldTransform();


		for (int i = 0; i < constraint.NumConstraintPoints(); ++i)
		{
			auto& contact_point = constraint.PointConstraint(i);

			///New contact point in world as bodies position might have been corrected
			const Vec3 p0 = transform0.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint0));
			const Vec3 p1 = transform1.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint1));

			float seperation = (p1 - p0).Dot(n);
			///seperation constant
			/// dist along normal + slop 
			/// to avoid jittering between bodies
			//float C = VxMax(seperation + slop, -kEpsilon);
			//float C = seperation + slop;
			float C = seperation + 0.02f;

			if (C < 0.0f)
			{
				float inv_effective_mass = total_inv_mass;
				Vec3 p = (p0 + p1) * 0.5f;
				/// point relative to bodies
				Vec3 r0 = p - bodyA->Position();
				Vec3 r1 = p - bodyB->Position();

				Vec3 inv_Ir0_Xn, inv_Ir1_Xn;
				if (dyn_a)
				{
					Vec3 r0_X_n = r0.Cross(n);
					inv_Ir0_Xn = bodyA->ComputeInvInteriaWorld().Multiply3x3(r0_X_n);
					inv_effective_mass += r0_X_n.Dot(inv_Ir0_Xn);
				}
				if (dyn_b)
				{
					Vec3 r1_X_n = r1.Cross(n);
					inv_Ir1_Xn = bodyB->ComputeInvInteriaWorld().Multiply3x3(r1_X_n);
					inv_effective_mass += r1_X_n.Dot(inv_Ir1_Xn);
				}

				//if (inv_effective_mass < 1e-9f)
				if (inv_effective_mass > 0)
				{
					//continue;

					//C = VxMax(C, -correction_limit);
					float lambda = -(baumgarte * C) / inv_effective_mass;
					Vec3 lambda_vector = lambda * n;

					if (dyn_a)
					{
						bodyA->ApplyLinearDisplacement(-lambda_vector * bodyA->InverseMass());
						bodyA->ApplyAngularDisplacement(-lambda * inv_Ir0_Xn);
					}
					if (dyn_b)
					{
						bodyB->ApplyLinearDisplacement(lambda_vector * bodyB->InverseMass());
						bodyB->ApplyAngularDisplacement(lambda * inv_Ir1_Xn);
					}
				}
			}
		}

		if (dyn_a)
			bodyA->UpdateTransformState();
		if (dyn_b)
			bodyB->UpdateTransformState();
	}

	void ContactConstraintSolver::SolvePositionCorrections(SolverBody* bodies, BodyManager& body_manager, float baumgarte, float slop, float min_limit, float max_limit, float limit_scale)
	{
		VX_PROFILE_FUNCTION();
		for (uint32 contact_idx = 0; contact_idx < mNumConstraints; ++contact_idx)
		{

			ContactConstraint& constraint = mConstraints[contact_idx];



			SolverBodyIndex& body0 = constraint.BodyA();
			SolverBodyIndex& body1 = constraint.BodyB();

			if (!body0.Value() && !body1.Value())
			{
				VX_LOG_WARN("either bodies needs to be valid");
				continue;
			}

			SolverBody& sbA = bodies[body0.Value()];
			SolverBody& sbB = bodies[body1.Value()];


			Body* a = &body_manager.GetBody(sbA.bodyID);
			Body* b = &body_manager.GetBody(sbB.bodyID);

			//effective mass 
			float total_inv_mass = sbA.invMass + sbB.invMass;
			SolvePositionCorrection(constraint, a, b, total_inv_mass, baumgarte, slop, min_limit, max_limit, limit_scale);
		}
	}
#else


	void ContactConstraintSolver::SolverContactManifold(const SolverSettings& phy_settings)
	{
		VX_PROFILE_FUNCTION();

		for (int i = 0; i < phy_settings.velocityIterations; ++i)
		{

			//curr_manifold_idx = -1;
			//for (auto& contact_info : manifolds)
			for (uint32 contact_idx = 0; contact_idx < mNumConstraints; ++contact_idx)
			{
				auto& constraint_info = mConstraints[contact_idx];//constraint info


				if (!constraint_info.body0 || !constraint_info.body1 || constraint_info.numConstraintPoints < 0)
				{
					VX_ASSERT_WARN(constraint_info.body0, "Body 0 is invalid");
					VX_ASSERT_WARN(constraint_info.body1, "Body 1 is invalid");
					VX_ASSERT_WARN(constraint_info.numConstraintPoints > 0, "No contact points");
					continue;
				}

				Body* bodyA = constraint_info.body0;
				Body* bodyB = constraint_info.body1;

				bool dyn_a = (bodyA->IsDynamic());
				bool dyn_b = (bodyB->IsDynamic());

				Vec3 n = Vec3::LoadFloat3Raw(constraint_info.normal);
				n.Normalise();
				//contact basis
				Vec3 tangents[2];
				//GetTangentBasis(n, nullptr, tangents[0], tangents[1]);
				//tangents[0] = Vec3::LoadFloat3Raw(constraint_info.contactPoints[0].tangent[0].axis);
				//tangents[1] = Vec3::LoadFloat3Raw(constraint_info.contactPoints[0].tangent[1].axis);


				float initial_total_energy = bodyA->GetKineticEnergy() + bodyB->GetKineticEnergy();

				//////Get velocities 
				Vec3 lin_vel0 = Vec3(0.0f);
				Vec3 ang_vel0 = Vec3(0.0f);

				if (dyn_a)
				{
					lin_vel0 = bodyA->LinearVelocity();
					ang_vel0 = bodyA->AngularVelocity();
				}

				Vec3 lin_vel1 = Vec3(0.0f);
				Vec3 ang_vel1 = Vec3(0.0f);

				if (dyn_b)
				{
					lin_vel1 = bodyB->LinearVelocity();
					ang_vel1 = bodyB->AngularVelocity();
				}




				//for (auto& pt : contact_info.points)
				for (int i = 0; i < constraint_info.numConstraintPoints; ++i)
				{
					auto& pt = constraint_info.contactPoints[i];

					tangents[0] = Vec3::LoadFloat3Raw(pt.lateralTangent[0].axis);
					tangents[1] = Vec3::LoadFloat3Raw(pt.lateralTangent[1].axis);


					///check closeness 
					//VX_ASSERT_WARN(n.IsApprox(Vec3::LoadFloat3Raw(pt.normal.axis)), "normal is bad");
					//VX_ASSERT_WARN(tangents[0].IsApprox(Vec3::LoadFloat3Raw(pt.tangent[0].axis)), "tangent0 is bad");
					//VX_ASSERT_WARN(tangents[1].IsApprox(Vec3::LoadFloat3Raw(pt.tangent[1].axis)), "tangent1 is bad");

					///this could be solve in simd parallel n, t0, t1
					///
					//////////////////////////
					//// Solve normal/penetration axis
					//////////////////////////
					ContactPointConstraint::ConstraintAxis& nor_axis_contraint = pt.normal;
					// 
					{
						/// curreny relative velocity (J * v) 
						float jn;
						if (dyn_a && dyn_b)
							jn = (lin_vel0 - lin_vel1).Dot(n);
						else if (dyn_a)
							jn = lin_vel0.Dot(n);
						else if (dyn_b)
							jn = (-lin_vel1).Dot(n);
						else
						{
							VX_LOG_ERROR("Static vs static this should not be possible");
							jn = 0.0f;
						}
						//simplify 
						if (dyn_a)
							jn += Vec3::LoadFloat3Raw(nor_axis_contraint.r0XAxis).Dot(ang_vel0);
						if(dyn_b)
							jn -= Vec3::LoadFloat3Raw(nor_axis_contraint.r1XAxis).Dot(ang_vel1);

						/// -K^-1(Jv + b)
						/// -K^-1((1-e)Jv)
						/// nor_axis_contraint.effectiveMass = 1/inv effective mass
						//float lambda = (nor_axis_contraint.bias - jn) * nor_axis_contraint.effectiveMass;
						float lambda = (jn - nor_axis_contraint.bias) * nor_axis_contraint.effMass;

						float old_lambda = nor_axis_contraint.totalLambda;
						//ensure non negative
						nor_axis_contraint.totalLambda = VxMax(old_lambda + lambda, 0.0f);
						//updated jn
						float impluse = nor_axis_contraint.totalLambda - old_lambda;
			
						//store changes
						if(dyn_a)
						{
							lin_vel0 -= impluse * constraint_info.invMass0 * n;
							ang_vel0 -= impluse * Vec3::LoadFloat3Raw(nor_axis_contraint.invIr0XAxis);
						}
						if (dyn_b)
						{
							lin_vel1 += impluse * constraint_info.invMass1 * n;
							ang_vel1 += impluse * Vec3::LoadFloat3Raw(nor_axis_contraint.invIr1XAxis);
						}

					}

				
					////////////////////////////
					////// Solve tangential axis
					////////////////////////////
					if(constraint_info.friction > 0.0f)
					{
						float max_friction = constraint_info.friction * nor_axis_contraint.totalLambda;

						for (int i = 0; i < 2; ++i)
						{
							ContactPointConstraint::ConstraintAxis& axis_contraint = pt.lateralTangent[i];
							const Vec3& axis = Vec3::LoadFloat3Raw(axis_contraint.axis);// = tangents[i];

							float jv;
							if (dyn_a && dyn_b)
								jv = (lin_vel0 - lin_vel1).Dot(axis);
							else if (dyn_a)
								jv = lin_vel0.Dot(axis);
							else if (dyn_b)
								jv = (-lin_vel1).Dot(axis);
							else
							{
								VX_LOG_ERROR("Static vs static this should not be possible");
								jv = 0.0f;
							}


							//simplify 
							if (dyn_a)
								jv += Vec3::LoadFloat3Raw(axis_contraint.r0XAxis).Dot(ang_vel0);
							if (dyn_b)
								jv -= Vec3::LoadFloat3Raw(axis_contraint.r1XAxis).Dot(ang_vel1);

							//float lambda = contact_info.friction * axis_contraint.effectiveMass * jv;
							//float lambda = contact_info.friction * 0.5f * axis_contraint.effectiveMass * jv;

							//ignore surface relative velocity
							float lambda = jv * axis_contraint.effMass;

							float old_lambda = axis_contraint.totalLambda;
							//ensure non negative
							axis_contraint.totalLambda = VxClamp(old_lambda + lambda, -max_friction, max_friction);
							//updated jn
							float impluse = axis_contraint.totalLambda - old_lambda;

							//store changes
							if (dyn_a)
							{
								lin_vel0 -= impluse * constraint_info.invMass0 * axis;
								ang_vel0 -= impluse * Vec3::LoadFloat3Raw(axis_contraint.invIr0XAxis);
							}
							if (dyn_b)
							{
								lin_vel1 += impluse * constraint_info.invMass1 * axis;
								ang_vel1 += impluse * Vec3::LoadFloat3Raw(axis_contraint.invIr1XAxis);
							}
						}
					}



					VX_ASSERT(!lin_vel0.IsNaN(), "lin_vel0 is nan");
					VX_ASSERT(!ang_vel0.IsNaN(), "ang_vel0 is nan");
					VX_ASSERT(!lin_vel1.IsNaN(), "lin_vel1 is nan");
					VX_ASSERT(!ang_vel1.IsNaN(), "ang_vel1 is nan");

					////set velocities; prevent multiple bodies value value changes
					/// and heavy torque level & world moment inetria internal to bodies ApplyImpluse
					if(dyn_a)
					{
						bodyA->SetLinearVelocity(lin_vel0);
						bodyA->SetAngularVelocity(ang_vel0);
					}

					if(dyn_b)
					{
						bodyB->SetLinearVelocity(lin_vel1);
						bodyB->SetAngularVelocity(ang_vel1);
					}

					float total_enery_after = bodyA->GetKineticEnergy() + bodyB->GetKineticEnergy();

					float solver_work = total_enery_after - initial_total_energy;
					mStats.totalKineticWork += solver_work;

					mStats.totalStepKineticWork += solver_work;

					//good / expected behaviour
					/// failed work 
					/// if after is greater than start -> solver system add energy to body
					/// if solver work if negative means solver add more energy 
					/// if solver work is positive means the solver is good job of removing energy
					if(solver_work < 0.0f)
						mStats.totalStepWorkLoss += VxAbs(solver_work);
					else 
						mStats.totalStepWorkGain += VxAbs(solver_work);

					//track peak instability 	
					mStats.maxAttainedKineticWork = VxMax(mStats.maxAttainedKineticWork, mStats.totalStepWorkGain);
				}
			}
		}
	}
#endif // CONTACT_USE_SOLVERBODY





	void ContactConstraintSolver::OnDebugDraw(DebugGizmosRenderer* debug_renderer, const DrawSettings& draw_settings)
	{
		if (!draw_settings.drawContactConstraintSolverTBNs)
			return;

		for (const ContactConstraint* constraint = mConstraints, *constraint_end = mConstraints + mNumConstraints;
			constraint < constraint_end; ++constraint)
		{


			for (const ContactPointConstraint* contact_point = constraint->PointConstraintPtr(),
				*contact_point_end = constraint->PointConstraintPtr() + constraint->NumConstraintPoints();
				contact_point < contact_point_end; ++contact_point)
			{
				Vec3 normal = Vec3::LoadFloat3Raw(contact_point->normal.axis);

				Vec3 t0 = Vec3::LoadFloat3Raw(contact_point->lateralTangent[0].axis);
				Vec3 t1 = Vec3::LoadFloat3Raw(contact_point->lateralTangent[1].axis);

				Vec3 p;

				float scale = 1.0f;
				{
					Vec3 p0 = Vec3::LoadFloat3Raw(contact_point->cacheLocalPoint->localPoint0);
					Vec3 p1 = Vec3::LoadFloat3Raw(contact_point->cacheLocalPoint->localPoint1);

					p = (p0 + p1) * 0.5f;

					if (draw_settings.scaleAxesWithDepth)
						scale = VxAbs((p0 - p1).Length());
					else
						scale = draw_settings.drawContactConstraintSolverTBNsScale;
				}

				settings.debug_renderer->DrawArrowCone(p,
					p + (normal * scale),
					0.05f, 0.1, 0.05f, 3, Colour::sGreen);

				settings.debug_renderer->DrawArrowCone(p,
					p + (t0 * scale),
						0.05f, 0.1, 0.05f, 3, Colour::sBlue);

				settings.debug_renderer->DrawArrowCone(p,
					p + (t1 * scale),
					0.05f, 0.1, 0.05f, 3, Colour::sRed);
			}
		}


	}
} //namespace vx