#include "PointConstraint.h"

#include "Vortrix/Dynamics/ConstraintSolver.h"


#include "Vortrix/Dynamics/IslandCoordinator.h"

#include "Vortrix/Core/Colours.h"

namespace vx {


	PointConstraint::PointConstraint(Body* bodyA, Body* bodyB, const PointConstraintSettings& settings) : 
		Constraint(bodyA, bodyB)
	{
		VX_ASSERT(bodyA && bodyB, "invalid constraint body pairs");

		if (settings.anchorPointFrame == EConstraintFrame::World)
		{
			///anchor A should be equal B 
			//VX_ASSERT_WARN(settings.anchorA == settings.anchorB);

			mLocalAnchorA = bodyA->Orientation().InverseRotate(settings.anchorA - bodyA->Position());
			mLocalAnchorB = bodyB->Orientation().InverseRotate(settings.anchorB - bodyB->Position());
		}
		else
		{
			mLocalAnchorA = settings.anchorA;
			mLocalAnchorB = settings.anchorB;
		}

		mHasVelocityBias = settings.enableVelocityBias;
		mErrorTreshold = settings.errorTreshold;
	}


	uint32 PointConstraint::PrepSolver(ConstraintSolver* solver, const PhysicsStepContext& ctx)
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

		Linear1DRow* rows = solver->AllocateLinear1DRow(constraint_row_idx, 3);
		BuildSplit1DJacobians(rows, ctx.mDeltaTime);

		if (mBodyA->IsDynamic())
			ctx.mIslandCoordinator->LinkNonConstactConstraint(constraint_row_idx, 3, mBodyA->GetIndexInActiveBodies());
		else if (mBodyB->IsDynamic())
			ctx.mIslandCoordinator->LinkNonConstactConstraint(constraint_row_idx, 3, mBodyB->GetIndexInActiveBodies());
		else
			VX_ASSERT(false);

		SolverBodyIndex idxA = solver->GetOrCreateSolverBody(mBodyA->ID(), ctx);
		SolverBodyIndex idxB = solver->GetOrCreateSolverBody(mBodyB->ID(), ctx);

		for(int i = 0; i < 3; ++i)
		{
			auto& row = rows[i];
			row.bodyAidx = idxA;
			row.bodyBidx = idxB;

			row.info = ConstraintRowInfo(mConstraintIdx, i, 3, true);
			VX_ASSERT(row.info.ConstraintIndex() == mConstraintIdx && row.info.RowLocalIndex() == i);
			VX_ASSERT(row.info.RowCount() == 3 && row.info.NeedPositionCorrection() == true);
		}

		//alway solve constraint position correction 
		solver->AppendPositionCorrectionQueue(this);// Queue

		//quick hack 
		//if warm start is disable, then no required accumulate lambda write back 
		mAccumulatedLambda = {};

		return 3;
	}

	void PointConstraint::CommitSolverState(const Linear1DRow& row)
	{
		mAccumulatedLambda[row.info.RowLocalIndex()] = row.lambda;
	}

	void PointConstraint::SolvePositionConstraint(float dt, float baumgarte)
	{
		
		Vec3 rA = mBodyA->Orientation().Rotate(mLocalAnchorA);
		Vec3 rB = mBodyB->Orientation().Rotate(mLocalAnchorB);

		Vec3 pA = mBodyA->Position() + rA;
		Vec3 pB = mBodyB->Position() + rB;
		Vec3 separation = pB - pA;

		Mat44 invIA_x_rAx = Mat44(0.0f);
		Mat44 invIB_x_rBx = Mat44(0.0f);

		float inv_massA {0};
		float inv_massB {0};
		Mat44 inv_eff_M = Mat44(0.0f);


		bool bodyA_nonstatic = !mBodyA->IsStatic();
		bool bodyB_nonstatic = !mBodyB->IsStatic();

		//recompute properties
		if (bodyA_nonstatic)
		{
			Mat44 invIA = mBodyA->ComputeInvInteriaWorld();
			Mat44 rAx = Mat44::SkewSymmetric3x3(rA);
			invIA_x_rAx = invIA.Multiply3x3(rAx);

			inv_massA = mBodyA->InverseMass();
			inv_eff_M = rAx.Multiply3x3(invIA).Multiply3x3RightTransposed(rAx);
			//VX_LOG_DEBUG("inv_eff_M: ", inv_eff_M);
		}
		if (bodyB_nonstatic)
		{
			Mat44 invIB = mBodyB->ComputeInvInteriaWorld();
			Mat44 rBx = Mat44::SkewSymmetric3x3(rB);
			invIB_x_rBx = invIB.Multiply3x3(rBx);

			inv_massB = mBodyB->InverseMass();
			inv_eff_M = inv_eff_M.Add(rBx.Multiply3x3(invIB).Multiply3x3RightTransposed(rBx));
			//VX_LOG_DEBUG("inv_eff_M: ", inv_eff_M);
		}

		const float det = inv_eff_M.Determinant3x3();

		//if (det == 0.0f)
		if (VxAbs(det) < kEpsilon)
			return;

		Mat44 eff_M = inv_eff_M.Inverse3x3();
		float eps = (mHasVelocityBias) ? mErrorTreshold : kEpsilon;
		if (Vec3::Greater(separation.Abs(), Vec3(eps)))
		{
			Vec3 lambda = eff_M.Multiply3x3(-baumgarte * separation);

			if (bodyA_nonstatic)
			{
				Vec3 x = lambda * inv_massA;
				mBodyA->ApplyLinearDisplacement(-x);
				mBodyA->ApplyAngularDisplacement(-(invIA_x_rAx.Multiply3x3(lambda).Normalise()));
			}
			if (bodyB_nonstatic)
			{
				Vec3 x = lambda * inv_massB;
				mBodyB->ApplyLinearDisplacement(x);
				mBodyB->ApplyAngularDisplacement(invIB_x_rBx.Multiply3x3(lambda).Normalise());
			}
		}
	}

	void PointConstraint::DebugGizmos(DebugGizmosRenderer* debug_renderer, const NonContactConstraintDrawSettings& draw_settings) const
	{
		Vec3 rA, rB;
		rA = mBodyA->Orientation().Rotate(mLocalAnchorA);
		rB = mBodyB->Orientation().Rotate(mLocalAnchorB);

		Vec3 rAw = rA + mBodyA->Position();
		Vec3 rBw = rB + mBodyB->Position();

		debug_renderer->DrawAACross(rAw, &Colour(1.0f, 0.2f, 0.2f), 1, draw_settings.anchorSize);
		debug_renderer->DrawAACross(rBw, &Colour(0.2f, 1.0f, 0.4f), 1, draw_settings.anchorSize);

		//error
		if (!rAw.IsApprox(rBw))
			debug_renderer->DrawLine(rAw, rBw, Colour::sRed);
	}


	void PointConstraint::QuickSolveUnified3DJacobian(float dt, int velocity_iteration, int position_iteration, float baumgarte)
	{

		//lets take into consideration that 
		// that the achor point is not COM
		Vec3 rA = mBodyA->Orientation().Rotate(mLocalAnchorA);
		Vec3 rB = mBodyB->Orientation().Rotate(mLocalAnchorB);

		bool bodyA_nonstatic = !mBodyA->IsStatic();
		bool bodyB_nonstatic = !mBodyB->IsStatic();

		Mat44 invIA_x_rAx = Mat44(0.0f);
		Mat44 invIB_x_rBx = Mat44(0.0f);

		Mat44 inv_eff_M = Mat44(0.0f);


		Vec3 bias;
		if (mHasVelocityBias)
		{
			Vec3 pA = mBodyA->Position() + rA;
			Vec3 pB = mBodyB->Position() + rB;

			Vec3 error = pB - pA;
			bias = (Vec3::Greater(error.Abs(), Vec3(mErrorTreshold))) ? error / dt : Vec3(0.0f);

			bias = baumgarte * bias;
		}
		else
			bias.ToZero();

		float inv_massA = 0.0f;
		float inv_massB = 0.0f;

		if (bodyA_nonstatic)
		{
			Mat44 invIA = mBodyA->ComputeInvInteriaWorld();

			Mat44 rAx = Mat44::SkewSymmetric3x3(rA);
			invIA_x_rAx = invIA.Multiply3x3(rAx);

			inv_massA = mBodyA->InverseMass();
			inv_eff_M = rAx.Multiply3x3(invIA).Multiply3x3RightTransposed(rAx);
		}

		if (bodyB_nonstatic)
		{
			Mat44 invIB = mBodyB->ComputeInvInteriaWorld();

			Mat44 rBx = Mat44::SkewSymmetric3x3(rB);
			invIB_x_rBx = invIB.Multiply3x3(rBx);

			inv_massB = mBodyB->InverseMass();
			inv_eff_M = inv_eff_M.Add(rBx.Multiply3x3(invIB).Multiply3x3RightTransposed(rBx));
		}

		if (!bodyA_nonstatic && !bodyB_nonstatic)
			return;

		float point_inv_eff_mass = inv_massA + inv_massB;

		inv_eff_M = inv_eff_M.Add(Mat44::Scale(point_inv_eff_mass));

		const float det = inv_eff_M.Determinant3x3();

		Vec3 Mx = inv_eff_M.GetAxisX();
		Vec3 My = inv_eff_M.GetAxisY();
		Vec3 Mz = inv_eff_M.GetAxisZ();
		const float det2 = Mx.ScalarTriple(My, Mz);

		VX_ASSERT_WARN(VxApprox(det, det2), "det");
		//if (VxAbs(det) < kEpsilon)
		if (det == 0.0f)
			return;

		Mat44 eff_M = inv_eff_M.Inverse3x3();

		bool dyn_a = mBodyA->IsDynamic();
		bool dyn_b = mBodyB->IsDynamic();

		Vec3 lin_velA = mBodyA->LinearVelocity();
		Vec3 lin_velB = mBodyB->LinearVelocity();

		Vec3 ang_velA = mBodyA->AngularVelocity();
		Vec3 ang_velB = mBodyB->AngularVelocity();

		Vec3 acc_lambda = Vec3(0.0f);
		//velocity iteration
		for (int i = 0; i < velocity_iteration; ++i)
		{

			Vec3 impluse = eff_M.Multiply3x3(lin_velA
				- rA.Cross(ang_velA)
				- lin_velB
				+ rB.Cross(ang_velB)
				- bias);
			//Vec3 Jv = (lin_velA - lin_velB)
			//	+ rA.Cross(ang_velA)
			//	- rB.Cross(ang_velB);

			//Vec3 impluse = -eff_M.Multiply3x3(Jv);

			acc_lambda += impluse;

			if (dyn_a)
			{
				lin_velA -= impluse * inv_massA;
				ang_velA -= invIA_x_rAx.Multiply3x3(impluse);
			}
			if (dyn_b)
			{
				lin_velB += impluse * inv_massB;
				ang_velB += invIB_x_rBx.Multiply3x3(impluse);
			}
		}

		//write back to body 
		mBodyA->SetLinearVelocity(lin_velA);
		mBodyA->SetAngularVelocity(ang_velA);

		mBodyB->SetLinearVelocity(lin_velB);
		mBodyB->SetAngularVelocity(ang_velB);


		//return;
		///cache for position constrint props
		Mat44 invIA = mBodyA->ComputeInvInteriaWorld();
		Mat44 invIB = mBodyB->ComputeInvInteriaWorld();
		for (int i = 0; i < position_iteration; ++i)
		{
			//Vec3 separation = (Vec3(mBodyB->Position() - mBodyA->Position()) + mLocalAnchorB - mLocalAnchorA);

			Vec3 rA = mBodyA->Orientation().Rotate(mLocalAnchorA);
			Vec3 rB = mBodyB->Orientation().Rotate(mLocalAnchorB);

			invIA_x_rAx = Mat44(0.0f);
			invIB_x_rBx = Mat44(0.0f);

			//recompute properties
			if (bodyA_nonstatic)
			{
				Mat44 rAx = Mat44::SkewSymmetric3x3(rA);
				invIA_x_rAx = invIA.Multiply3x3(rAx);
			}
			if (bodyB_nonstatic)
			{
				Mat44 rBx = Mat44::SkewSymmetric3x3(rB);
				invIB_x_rBx = invIB.Multiply3x3(rBx);
			}

			///recompute error 
			Vec3 pA = mBodyA->Position() + rA;
			Vec3 pB = mBodyB->Position() + rB;
			Vec3 separation = pB - pA;

			Vec3 axis = separation.Normalised();
			Vec3 new_err = separation - (axis * mErrorTreshold);
			if (Vec3::Greater(new_err.Abs(), Vec3(kEpsilon)))
				//if (!separation.IsZero())
				//if(Vec3::Greater(separation.Abs(), axis * mErrorTreshold))
			{
				separation = new_err;

				Vec3 lambda = eff_M.Multiply3x3(-baumgarte * separation);

				if (dyn_a)
				{
					Vec3 x = lambda * inv_massA;
					mBodyA->ApplyLinearDisplacement(-x);
					mBodyA->ApplyAngularDisplacement(-(invIA_x_rAx.Multiply3x3(lambda)));
				}
				if (dyn_b)
				{
					Vec3 x = lambda * inv_massB;
					mBodyB->ApplyLinearDisplacement(x);
					mBodyB->ApplyAngularDisplacement(invIB_x_rBx.Multiply3x3(lambda));
				}
			}
		}
	}



	void PointConstraint::QuickSolveSplit1DJacobians(float dt, int velocity_iteration, int position_iteration, float baumgarte)
	{
		Linear1DRow rows[3];
		BuildSplit1DJacobians(rows, dt);


		bool dyn_a = mBodyA->IsDynamic();
		bool dyn_b = mBodyB->IsDynamic();

		Vec3 lin_velA = mBodyA->LinearVelocity();
		Vec3 lin_velB = mBodyB->LinearVelocity();

		Vec3 ang_velA = mBodyA->AngularVelocity();
		Vec3 ang_velB = mBodyB->AngularVelocity();

		float inv_massA = mBodyA->InverseMass();
		float inv_massB = mBodyB->InverseMass();

		//velocity iteration
		for (int i = 0; i < velocity_iteration; ++i)
		{
			///
			///effective mass along X axis 

			for (int x = 0; x < 3; ++x)
			{
				Vec3 axis = Vec3::LoadFloat3Raw(rows[x].axis).Normalised();

				Vec3 rAXaxis = Vec3::LoadFloat3Raw(rows[x].rAXn);
				Vec3 rBXaxis = Vec3::LoadFloat3Raw(rows[x].rBXn);

				Vec3 invIrAXaxis = Vec3::LoadFloat3Raw(rows[x].invIrAXn);
				Vec3 invIrBXaxis = Vec3::LoadFloat3Raw(rows[x].invIrBXn);

				float jv;
				if (dyn_a && dyn_b) ///if constexpr (
					jv = (lin_velA - lin_velB).Dot(axis);
				else if (dyn_a)
					jv = lin_velA.Dot(axis);
				else if (dyn_b)
					jv = (-lin_velB).Dot(axis);
				else
				{
					VX_LOG_ERROR("Static vs static this should not be possible");
					jv = 0.0f;
				}

				if (dyn_a)
					jv += rAXaxis.Dot(ang_velA);
				if (dyn_b)
					jv -= rBXaxis.Dot(ang_velB);


				float impluse = (jv - rows[x].bias) * rows[x].effMass;
				if (dyn_a)
				{
					lin_velA -= impluse * inv_massA * axis;
					ang_velA -= impluse * invIrAXaxis;
				}
				if (dyn_b)
				{
					lin_velB += impluse * inv_massB * axis;
					ang_velB += impluse * invIrBXaxis;
				}
			}
		}

		//write back to body 
		mBodyA->SetLinearVelocity(lin_velA);
		mBodyA->SetAngularVelocity(ang_velA);

		mBodyB->SetLinearVelocity(lin_velB);
		mBodyB->SetAngularVelocity(ang_velB);

		///cache for position constrint props

		bool bodyA_nonstatic = !mBodyA->IsStatic();
		bool bodyB_nonstatic = !mBodyB->IsStatic();

		Mat44 invIA = mBodyA->ComputeInvInteriaWorld();
		Mat44 invIB = mBodyB->ComputeInvInteriaWorld();

		for (int i = 0; i < position_iteration; ++i)
		{
			//Vec3 separation = (Vec3(mBodyB->Position() - mBodyA->Position()) + mLocalAnchorB - mLocalAnchorA);


			Vec3 axes[3] = { Vec3::Right(), Vec3::Up(), Vec3::Forward() };

			for (int j = 0; j < 3; ++j)
			{
				Vec3 axis = axes[i];

				Vec3 rA, rB;
				Vec3 dispW = ComputeConstraintPropertiesDisplacement(rA, rB);
				float seperation_along_axis = axis.Dot(dispW);
				//if (seperation_along_axis < kEpsilon)
				//	continue;

				Vec3 invIArAXn = Vec3(0.0f);
				Vec3 invIBrBXn = Vec3(0.0f);

				float inv_eff_mass = 0.0f;
				if (bodyA_nonstatic)
				{
					Vec3 rAXn = rA.Cross(axis);
					invIArAXn = invIA.Multiply3x3(rAXn);

					inv_eff_mass += mBodyA->InverseMass() + invIArAXn.Dot(rAXn);
				}
				if (bodyB_nonstatic)
				{
					Vec3 rBXn = rB.Cross(axis);
					invIBrBXn = invIB.Multiply3x3(rBXn);

					inv_eff_mass += mBodyB->InverseMass() + invIBrBXn.Dot(rBXn);
				}

				float lambda = -(1.0f / inv_eff_mass) * baumgarte * seperation_along_axis;

				if (dyn_a)
				{
					Vec3 x = lambda * inv_massA * axis;
					mBodyA->ApplyLinearDisplacement(-x);
					mBodyA->ApplyAngularDisplacement(-lambda * invIArAXn);
				}
				if (dyn_b)
				{
					Vec3 x = lambda * inv_massB * axis;
					mBodyB->ApplyLinearDisplacement(x);
					mBodyB->ApplyAngularDisplacement(lambda * invIBrBXn);
				}
			}

		}
	}




	Vec3 PointConstraint::ComputeConstraintPropertiesDisplacement(Vec3& o_rA, Vec3& o_rB)
	{
		//lets take into consideration that 
		// that the achor point is not COM
		o_rA = mBodyA->Orientation().Rotate(mLocalAnchorA);
		o_rB = mBodyB->Orientation().Rotate(mLocalAnchorB);

		Vec3 pA = o_rA + mBodyA->Position();
		Vec3 pB = o_rB + mBodyB->Position();

		return pB - pA;
	}

	void PointConstraint::BuildSplit1DJacobians(Linear1DRow* rows, float dt)
	{
		Vec3 rA, rB;
		Vec3 dispW = ComputeConstraintPropertiesDisplacement(rA, rB);

		BuildAxis1DJacobian(&rows[0], mAccumulatedLambda[0], Vec3::Right(), rA, rB, dispW, dt);
		BuildAxis1DJacobian(&rows[1], mAccumulatedLambda[1], Vec3::Up(), rA, rB, dispW, dt);
		BuildAxis1DJacobian(&rows[2], mAccumulatedLambda[2], Vec3::Forward(), rA, rB, dispW, dt);
	}

	void PointConstraint::BuildAxis1DJacobian(Linear1DRow* o_row, float accumulated_lambda, const Vec3& axis,
		const Vec3& rA, const Vec3& rB, const Vec3& dispW, float dt)
	{
		axis.Store(o_row->axis);
		//lets take into consideration that 
		// that the achor point is not COM

		bool bodyA_nonstatic = !mBodyA->IsStatic();
		bool bodyB_nonstatic = !mBodyB->IsStatic();


		o_row->gamma = 0.0f;
		o_row->minLambda = -kMaxf;
		o_row->maxLambda = kMaxf;

		float inv_eff_mass = 0.0f;

		if (bodyA_nonstatic)
		{
			Vec3 rAXn = rA.Cross(axis);
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
			Vec3 rBXn = rB.Cross(axis);
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
		}


		o_row->effMass = 1.0f / inv_eff_mass;


		if (mHasVelocityBias)
		{
			Vec3 error = dispW;

			//project error along axis 
			float axis_error = error.Dot(axis);
			float bias = (VxAbs(axis_error) > mErrorTreshold) ? axis_error / dt : 0.f;
			o_row->bias = 0.2f * bias;
		}
		else
			o_row->bias = 0.0f;

		o_row->lambda = accumulated_lambda;
		o_row->minLambda = -kMaxf;
		o_row->maxLambda = kMaxf;
	}

} //namespace vx 