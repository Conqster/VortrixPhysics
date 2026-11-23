#include "DistanceConstraint.h"
#include "Vortrix/PhysicsWorldSettings.h"
#include "Vortrix/Core/Profiler.h"
		  
#include "Vortrix/Dynamics/ConstraintSolver.h"

#include "Vortrix/Dynamics/IslandCoordinator.h"

#include "Vortrix/Core/Colours.h"

namespace vx{

	DistanceConstraint::DistanceConstraint(Body* bodyA, Body* bodyB, const DistanceConstraintSettings& settings) :
		Constraint(bodyA, bodyB),
		mLocalAnchorA(settings.localAnchorA),
		mLocalAnchorB(settings.localAnchorB), mMinDistance(settings.minDist),
		mMaxDistance(settings.maxDist),
		mSpring({ settings.frequency, settings.dampingRatio })
	{
		
		mFlags = EConstraintFlags::SolveVelocity;// | ~EConstraintFlags::SolvePosition;
		
		if(mSpring.mFrequency <= 0.0f)
			mFlags |= EConstraintFlags::SolvePosition;

	}
	
	uint32 DistanceConstraint::PrepSolver(ConstraintSolver* solver, const PhysicsStepContext& ctx)
	{
		mFlags |= EConstraintFlags::Active;

		bool active = (mBodyA->IsAwake() || mBodyB->IsAwake()) && (mBodyA->IsDynamic() || mBodyB->IsDynamic());
		active &= (mBodyA->IsIDValid() && mBodyB->IsIDValid());
		if (!active)
		{
			mFlags &= ~EConstraintFlags::Active;
			return 0;
		}


		/// Later move to like two bodies constraint 
		
		///Activate bodies other body if sleepign
		uint32 bodies_activate_count = 0;
		BodyID body_ids[2];

		if (mBodyA->IsDynamic() && !mBodyA->IsAwake())
			body_ids[bodies_activate_count++] = mBodyA->ID();
		if (mBodyB->IsDynamic() && !mBodyB->IsAwake())
			body_ids[bodies_activate_count++] = mBodyB->ID();

		if (bodies_activate_count > 0)
			ctx.bodyManager->ActivateBodies(body_ids, bodies_activate_count);

		/// link bodies if not already
		/// a is alway dynam,ic
		if (mBodyA->IsDynamic() && mBodyB->IsDynamic())
		{
			VX_ASSERT(mBodyA->GetIndexInActiveBodies() != Body::kInvalidActiveIdx && mBodyB->GetIndexInActiveBodies() != Body::kInvalidActiveIdx, "Invalid Body index");
			ctx.mIslandCoordinator->LinkBodies(mBodyA->GetIndexInActiveBodies(), mBodyB->GetIndexInActiveBodies());
		}

		uint32 constraint_row_idx;

		Linear1DRow* row = solver->AllocateLinear1DRow(constraint_row_idx, 1);
		BuildDistanceJacobian(row, ctx.mDeltaTime);

		if (mBodyA->IsDynamic())
			ctx.mIslandCoordinator->LinkNonConstactConstraint(constraint_row_idx, 1, mBodyA->GetIndexInActiveBodies());
		else if (mBodyB->IsDynamic())
			ctx.mIslandCoordinator->LinkNonConstactConstraint(constraint_row_idx, 1, mBodyB->GetIndexInActiveBodies());
		else
			VX_ASSERT(false);


		if (row->effMass == 0.0f)
		{
			mFlags &= ~EConstraintFlags::Active;
			VX_ASSERT(false, "This a bug");
			return 0;
		}

		row->bodyAidx = solver->GetOrCreateSolverBody(mBodyA->ID(), ctx);
		row->bodyBidx = solver->GetOrCreateSolverBody(mBodyB->ID(), ctx);

		bool require_position_correction = RequiresPositionCorrection();
		if(require_position_correction)
			solver->AppendPositionCorrectionQueue(this);// Queue

		//quick hack 
		//if warm start is disable, then no required accumulate lambda write back 
		mAccumulatedLambda = {};

		row->info = ConstraintRowInfo(mConstraintIdx, 0, 1, require_position_correction);
		VX_ASSERT(row->info.ConstraintIndex() == mConstraintIdx && row->info.RowLocalIndex() == 0);
		VX_ASSERT(row->info.RowCount() == 1 && row->info.NeedPositionCorrection() == require_position_correction);
		return 1;
	}


	Vec3 DistanceConstraint::ComputeConstraintPropertiesDisplacement(Vec3& o_rA, Vec3& o_rB)
	{
		//lets take into consideration that 
		// that the achor point is not COM
		o_rA = mBodyA->Orientation().Rotate(mLocalAnchorA);
		o_rB = mBodyB->Orientation().Rotate(mLocalAnchorB);

		mWorldAnchorA = o_rA + mBodyA->Position();
		mWorldAnchorB = o_rB + mBodyB->Position();


		Vec3 dispW = mWorldAnchorB - mWorldAnchorA;
		Vec3 nor = dispW.Normalised();
		mWorldAxis = nor;

		return dispW;
	}

	void DistanceConstraint::BuildDistanceJacobian(Linear1DRow* o_row, float dt)
	{
		//lets take into consideration that 
		// that the achor point is not COM
		Vec3 rA, rB;
		Vec3 dispW = ComputeConstraintPropertiesDisplacement(rA, rB);

		Vec3 nor = dispW.Normalised();
		nor.Store(o_row->axis);

		bool bodyA_nonstatic = !mBodyA->IsStatic();
		bool bodyB_nonstatic = !mBodyB->IsStatic();


		float inv_eff_mass = 0.0f;
		if (bodyA_nonstatic)
		{
			Vec3 rAXn = rA.Cross(nor);
			Vec3 invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);

			rAXn.Store(o_row->rAXn);
			invIrAXn.Store(o_row->invIrAXn);

			inv_eff_mass += mBodyA->InverseMass() + invIrAXn.Dot(rAXn);
		}
		else
		{
			o_row->rAXn = Float3(0.0f);
			o_row->invIrAXn = Float3(0.0f);
		}

		if (bodyB_nonstatic)
		{
			Vec3 rBXn = rB.Cross(nor);
			Vec3 invIrBXn = mBodyB->ComputeInvInteriaWorld().Multiply3x3(rBXn);

			rBXn.Store(o_row->rBXn);
			invIrBXn.Store(o_row->invIrBXn);

			inv_eff_mass += mBodyB->InverseMass() + invIrBXn.Dot(rBXn);
		}
		else
		{
			o_row->rBXn = Float3(0.0f);
			o_row->invIrBXn = Float3(0.0f);
		}

		
		VX_ASSERT(bodyA_nonstatic || bodyB_nonstatic);
		if (!bodyA_nonstatic && !bodyB_nonstatic)
		{
			o_row->effMass = 0.0f;
			o_row->lambda = 0.0f;
			return;
		}

		float error = 0.0f;
		float curr_dist = dispW.Length();
		//compute error and limits
		//bilateral propagation
		if (mMinDistance == mMaxDistance)
		{
			error = curr_dist - mMinDistance;
			o_row->minLambda = -kMaxf;
			o_row->maxLambda = kMaxf;
		}
		else if (curr_dist >= mMaxDistance)
		{
			///max limit breached 
			error = curr_dist - mMaxDistance;
			o_row->minLambda = -kMaxf;
			o_row->maxLambda = 0.0f;
		}
		else if (curr_dist <= mMinDistance)
		{
			error = curr_dist - mMinDistance;
			o_row->minLambda = 0.0f;
			o_row->maxLambda = kMaxf;
		}
		else
		{
			o_row->effMass = 0.0f;
			o_row->lambda = 0.0f;
		}


		mSpring.ComputeProperties(dt, inv_eff_mass, error, 0.0f, o_row->effMass, o_row->bias, o_row->gamma);

		///later when figure out, caching implmentation for warm start etc
		o_row->lambda = mAccumulatedLambda;
	}


	Rigid1DConstraint DistanceConstraint::BuildRigidConstraint()
	{
		Rigid1DConstraint rigid_constraint;

		//lets take into consideration that 
		// that the achor point is not COM
		Vec3 rA, rB;
		Vec3 dispW = ComputeConstraintPropertiesDisplacement(rA, rB);
		Vec3 nor = dispW.Normalised();
		rigid_constraint.axis = nor;

		bool bodyA_nonstatic = !mBodyA->IsStatic();
		bool bodyB_nonstatic = !mBodyB->IsStatic();


		float inv_eff_mass = 0.0f;
		if (bodyA_nonstatic)
		{
			Vec3 rAXn = rA.Cross(nor);
			Vec3 invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);
			invIrAXn.Store(rigid_constraint.invIrAXn);

			inv_eff_mass += mBodyA->InverseMass() + invIrAXn.Dot(rAXn);
		}

		if (bodyB_nonstatic)
		{
			Vec3 rBXn = rB.Cross(nor);
			Vec3 invIrBXn = mBodyB->ComputeInvInteriaWorld().Multiply3x3(rBXn);

			invIrBXn.Store(rigid_constraint.invIrBXn);
			inv_eff_mass += mBodyB->InverseMass() + invIrBXn.Dot(rBXn);
		}

		if (!bodyA_nonstatic && !bodyB_nonstatic)
		{
			rigid_constraint.effMass = 0.0f;
			return rigid_constraint;
		}


		rigid_constraint.effMass = 1.0f / inv_eff_mass;
		return rigid_constraint;
	}


	void DistanceConstraint::SolvePositionConstraint(float dt, float baumgarte)
	{
		///check policy
		if (!mSpring.Active())
		{
			float distance = (mWorldAnchorB - mWorldAnchorA).Dot(mWorldAxis);


			float error = 0.0f;
			if (distance < mMinDistance)
				error = distance - mMinDistance;
			else if (distance > mMaxDistance)
				error = distance - mMaxDistance;

			if (error != 0)
			{
				Rigid1DConstraint constraint = BuildRigidConstraint();
				constraint.SolvePosition(*mBodyA, *mBodyB, error, baumgarte);
			}
		}
	}

	void DistanceConstraint::DrawConstraintBounds(DebugGizmosRenderer* debug_renderer, const Vec3& rAw, const Vec3& rBw, Colour col) const
	{
		//achorbound a
		AABB boundA = AABB(0.25);
		boundA.Translate(rAw);

		AABB boundB = AABB(0.25);
		boundB.Translate(rBw);

		boundA.Merge(boundB);
		debug_renderer->DrawAABB(boundA, col);
	}

	void DistanceConstraint::DebugGizmos(DebugGizmosRenderer* debug_renderer, const NonContactConstraintDrawSettings& draw_settings) const
	{
		if (debug_renderer && mBodyA && mBodyB)
		{

			const float mA = 1.0f / mBodyA->InverseMass();
			const float mB = 1.0f / mBodyB->InverseMass();
			//split rest length based on mass contribution 
			float total_mass = mA + mB;
			//distribute force if total mass is not too small else split
			float ratio = (total_mass > kEpsilon) ? (mB / total_mass) : 0.5;


			Vec3 rA = mBodyA->Orientation().Rotate(mLocalAnchorA);
			Vec3 rB = mBodyB->Orientation().Rotate(mLocalAnchorB);

			Vec3 rAw = rA + mBodyA->Position();
			Vec3 rBw = rB + mBodyB->Position();

			Vec3 disp = rBw - rAw;
			float curr_dist = disp.Length();

			Vec3 nor = (curr_dist > kEpsilon) ? disp / curr_dist : Vec3::Up();

			Colour line_col = Colour(0.3f);
			if (mMinDistance != mMaxDistance)
			{
				if (curr_dist >= mMaxDistance)
					line_col = Colour(1.0f, 0.3f, 0.3f);
				else if (curr_dist <= mMinDistance)
					line_col = Colour(0.3f, 0.6f, 1.0f);
			}
			//else
				//line_col = Colour(0.0f, 1.0f, 0.0f);

			debug_renderer->DrawLine(rAw, rBw, line_col);

			if (draw_settings.drawConstraintBounds)
				DrawConstraintBounds(debug_renderer, rAw, rBw, Colour::sDeepTeal);

			if (draw_settings.drawActiveBounds && Contains(EConstraintFlags::Active, mFlags))
				DrawConstraintBounds(debug_renderer, rAw, rBw, Colour::sOrange);
	
			if (draw_settings.drawVelocitySolveBounds && Contains(EConstraintFlags::SolveVelocity, mFlags))
				DrawConstraintBounds(debug_renderer, rAw, rBw, Colour::sTurquoise);
			if (draw_settings.drawPositionSolveBounds && Contains(EConstraintFlags::SolvePosition, mFlags))
				DrawConstraintBounds(debug_renderer, rAw, rBw, Colour::sCyan);


			//if(Contains(EConstraintFlags::Active, mFlags))
			////if(Contains(EConstraintFlags::SolvePosition, mFlags))
			//	DrawConstraintBounds(debug_renderer, rAw, rBw);

			debug_renderer->DrawSphere4x4(rAw, draw_settings.anchorSize, Colour(1.0f, 0.2f, 0.2f));
			debug_renderer->DrawSphere4x4(rBw, draw_settings.anchorSize, Colour(0.2f, 1.0f, 0.6f));

			//boundaries
			if (mMinDistance == mMaxDistance)
			{
				debug_renderer->DrawLine(rAw, (rAw + (nor * static_cast<float>(mMaxDistance * ratio))), Colour(1.0f, 1.0f, 0.0f));
				debug_renderer->DrawLine(rBw, (rBw + (-nor * static_cast<float>(mMaxDistance * (1 - ratio)))), Colour(0.0f, 1.0f, 0.0f));
			}
			else
			{
				Vec3 min_limit_ptA = rAw + (nor * (mMinDistance * ratio));
				Vec3 min_limit_ptB = rBw - (nor * (mMinDistance * (1.0f - ratio)));

				Vec3 max_limit_ptA = rAw + (nor * (mMaxDistance * ratio));
				Vec3 max_limit_ptB = rBw - (nor * (mMaxDistance * (1.0f - ratio)));


				constexpr bool k_draw_box = true;

				if constexpr (k_draw_box)
				{
					bool wire_frame = true;
					////later support pt 
					AABB aabb(0.125);
					aabb.Translate(min_limit_ptA);
					debug_renderer->DrawAABB(aabb, Colour(0.3f, 0.6f, 1.0f), wire_frame);
					aabb.Reset();
					aabb = AABB(0.125);
					aabb.Translate(min_limit_ptB);
					debug_renderer->DrawAABB(aabb, Colour(0.3f, 0.6f, 1.0f), wire_frame);

					aabb.Reset();
					aabb = AABB(0.125);
					aabb.Translate(max_limit_ptA);
					debug_renderer->DrawAABB(aabb, Colour(1.0f, 1.0f, 0.0f), wire_frame);
					aabb.Reset();
					aabb = AABB(0.125);
					aabb.Translate(max_limit_ptB);
					debug_renderer->DrawAABB(aabb, Colour(1.0f, 1.0f, 0.0f), wire_frame);
				}
				else
				{
					debug_renderer->DrawSphere(min_limit_ptA, 0.125, Colour(0.3f, 0.6f, 1.0f));
					debug_renderer->DrawSphere(min_limit_ptB, 0.125, Colour(0.3f, 0.6f, 1.0f));

					debug_renderer->DrawSphere(max_limit_ptA, 0.125, Colour(1.0f, 1.0f, 0.0f));
					debug_renderer->DrawSphere(max_limit_ptB, 0.125, Colour(1.0f, 1.0f, 0.0f));
				}
			}
		}
	}

	void DistanceConstraint::QuickSolve(float dt, int velocity_iteration)
	{
		if (!mBodyA || !mBodyB)
			return;


		Linear1DRow solver_row;
		BuildDistanceJacobian(&solver_row, dt);

		if (solver_row.effMass <= 0.0f)
			return;

		Body& bodyA = *mBodyA; //context.BodyManager().GetBody(solver_row.bodyA)
		Body& bodyB = *mBodyB;

		bool dyn_a = bodyA.IsDynamic();
		bool dyn_b = bodyB.IsDynamic();



		///this is the work of solver body and not direct for Body
		Vec3 lin_velA = bodyA.LinearVelocity();
		Vec3 lin_velB = bodyB.LinearVelocity();
		Vec3 ang_velA = bodyA.AngularVelocity();
		Vec3 ang_velB = bodyB.AngularVelocity();
		float inv_massA = bodyA.InverseMass();
		float inv_massB = bodyB.InverseMass();


		//load data
		Vec3 rAXn = Vec3::LoadFloat3Raw(solver_row.rAXn);
		Vec3 invIrAXn = Vec3::LoadFloat3Raw(solver_row.invIrAXn);

		Vec3 rBXn = Vec3::LoadFloat3Raw(solver_row.rBXn);
		Vec3 invIrBXn = Vec3::LoadFloat3Raw(solver_row.invIrBXn);


		Vec3 axis = Vec3::LoadFloat3Raw(solver_row.axis);


		for (int i = 0; i < velocity_iteration; ++i)
		{
			float jv = axis.Dot(lin_velA - lin_velB) +
				rAXn.Dot(ang_velA) -
				rBXn.Dot(ang_velB);

			float compliance =  solver_row.gamma * solver_row.lambda + solver_row.bias;
			float lambda = (jv - compliance) * solver_row.effMass;

			float old_lambda = solver_row.lambda;
			//ensure non negative
			//bilateral constraint
			solver_row.lambda += lambda;
			solver_row.lambda = VxClamp(old_lambda + lambda, solver_row.minLambda, solver_row.maxLambda);
			//updated jn
			float impluse = solver_row.lambda - old_lambda;

			//store changes
			if (dyn_a)
			{
				lin_velA -= impluse * inv_massA * axis;
				ang_velA -= impluse * invIrAXn;
			}
			if (dyn_b)
			{
				lin_velB += impluse * inv_massB * axis;
				ang_velB += impluse * invIrBXn;
			}
		}

		mAccumulatedLambda = solver_row.lambda;

		//write back to body 
		bodyA.SetLinearVelocity(lin_velA);
		bodyA.SetAngularVelocity(ang_velA);

		bodyB.SetLinearVelocity(lin_velB);
		bodyB.SetAngularVelocity(ang_velB);
	}

} //namespace vx
