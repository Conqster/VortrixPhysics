#pragma once


#include "Vortrix.h"
#include "PhysicsWorldSettings.h"

#include "Collision/Broadphase/BroadphasePair.h"
#include "Collision/Broadphase/Broadphase.h"

#include "Dynamics/Body/BodyManager.h"

#include "Dynamics/ConstraintSolver/ContactConstraintSolver.h"
#include "Dynamics/Body/BodyID.h"

#include "Collision/RayCast.h"
#include "Collision/WorldQuery.h"

#include "Dynamics/ConstraintCoordinator.h"

#include "Collision/Narrowphase/NarrowphaseQuery.h"

#include "SimulationContexts.h"

#include <mutex>

////Things to do 
/// Implement 
///		MotionDynamics
/// fix shapes; especially box; function needs to be in local space
/// 
/// 

/// 
/// a deadly bug, you can just set the desnsity of a shap ein body manage 
/// the shape might be shared between multiple bodies

/// 
/// 
/// Later moving to Island building 
/// So do not make/setup constraint per success narrowphase
/// 
/// 
/// check bvh insight
/// 
/// 
/// FIX 
/// dynamic reallocation for BVH rebuild, 
/// ways centralise approapiate data to tree - broadphase system 
/// 
/// i.e 2 trees - both preallocated memotry.
/// then rebuild happen on tree2; no reallocation; just buiilf with 
/// avaliable resources.  
/// 
/// then swap main tree after construction. 
/// 
/// this would help to waste of memory
/// 
/// 
/// MULTIPLE SPHERE AT A TIME, CAUSE NAN IS SOLVER 
/// 
/// 
/// might need to fix this as 1k box then spawn capsule all 1k becomes capsile 
/// 					if(hack_shape == nullptr)
//{
//	body_settings.shape = mShapeArena.AllocateObject<BoxShape>(vx::Vec3::LoadFloat3Raw(mNewPhyObjectSettings.halfExtents));
//	hack_shape = body_settings.shape;
//					}
//					else
//						body_settings.shape = hack_shape;
//				}
//				else if (mNewPhyObjectSettings.bodyShape == vx::EShapeType::Capsule)
//				{
//					auto& he = mNewPhyObjectSettings.halfExtents;
//					body_settings.shape = mShapeArena.AllocateObject<CapsuleShape>(he.x, he.y);
//				}
//				else
//					//body_settings.shape = mShapeArena.AllocateObject<SphereShape>(mNewPhyObjectSettings.halfExtents.x);
//						body_settings.shape = new SphereShape(mNewPhyObjectSettings.halfExtents.x);


///
/// CONVERT MOUSE INTERACTION TO CONSTRAINT BASED
/// MEMOPRY ALLOC
// make body shape ref counted 



class DebugGizmosRenderer;
class Renderer;
namespace vx
{
	class Body;
	class BodySettings;
	class Broadphase;

	class TaskCoordinator;

	class AABB;
	template<typename T>
	class BVHBroadphase;

	class NarrowphaseQuery;

	enum class EShapeType : uint8;
	struct RenderSettings;
	//enum class ERenderInstanceFlags : uint32;

	class VX_API PhysicsWorld : NonCopyable
	{
	public:
		PhysicsWorld(PhysicsWorldSettings* in_settings);
		~PhysicsWorld();

		static void CreateSimpleWorld(PhysicsWorld* io_world);

		static void GenerateWorldDefaultConfig(int& o_max_bodies, int& o_max_body_pairs, int& o_max_contact_constraint);
		void Init(float max_bodies, float max_body_pairs, float max_contact_constraint);

		const PhysicsWorldSettings* Settings() const { return mSettings; }
		PhysicsWorldSettings* Settings() { return mSettings; }

		const PhysicsStepContext* Context() const { return &mContext; }

		const TaskCoordinator* GetTaskCoordinator() const { return mTaskCoordinator; }

		const ScratchAllocator* GetScratchAllocator() const { return mScratchAllocator; }

		////////////////////////////////////////////////////////////////////////
		/// Step Simulation
		/// 
		/// a single step into physics frame 
		void StepSimulation(float dt);

		static uint32 CurrentSimStepIndex() { return mStepIndex; }

		const IslandCoordinator* GetIslandCoordinator() const { return mIslandCoordinator; }
		/// Post Physics world query (ray casting)
		WorldQuery GetWorldQuery() const { return mWorldQuery; }

		void SetFrictionCombineMode(ECombineMode friction_combine_mode)
		{
			mSettings->solver.frictionCombineMode = friction_combine_mode;
			mContactConstraintSolver.SetFrictionCombineMode(friction_combine_mode);
		}
		void SetRestitutionCombineMode(ECombineMode restitution_combine_mode)
		{
			mContactConstraintSolver.SetRestitutionCombineMode(restitution_combine_mode);
			mSettings->solver.restitutionCombineMode = restitution_combine_mode;
		}





		/// Bodies Management
		const BodyManager& GetBodyManager() const { return mBodyManager; }
		BodyManager& GetBodyManager() { return mBodyManager; }

		Body* CreateBody(const BodySettings& body_setting, bool activate_body = true);
		void RemoveBody(const BodyID& id);
		void RemoveBodies(const BodyID* ids, uint32 count);

		BodyVector& Bodies() { return mBodyManager.GetBodies(); }

		BodyID* ActiveBodies() const { return mBodyManager.ActiveBodies(); }
		uint32 NumActiveBodies() const { return mBodyManager.NumActiveBodies(); }

		void ActivateBodies(const BodyID* body_ids, uint32 count) { mBodyManager.ActivateBodies(body_ids, count); }

		///interface bodies; add mutex later
		void ApplyImpulse(const BodyID body_id, const Vec3& impulse)
		{
			Body& body = mBodyManager.GetBody(body_id);

			if (body.IsDynamic())
			{
				body.ApplyImpulse(impulse);
				if (body.IsSleeping())
					mBodyManager.ActivateBodies(&body_id, 1);
			}
		}

		void AddForce(const BodyID body_id, const Vec3& force)
		{
			Body& body = mBodyManager.GetBody(body_id);

			if (body.IsDynamic())
			{
				body.AddForce(force);
				if (body.IsSleeping())
					mBodyManager.ActivateBodies(&body_id, 1);
			}
		}


		/// Constraints 
		template<typename T>
		T* CreateConstraintT(const T& constraint) { return mConstraintCoordinator.CreateT(constraint); }
		template<typename T>
		void CreateConstraintsT(const T* constraint_Ts, uint32 count) { return mConstraintCoordinator.CreateT(constraint_Ts, count); }
		void AddConstraint(Constraint* constraint) { return mConstraintCoordinator.Add(&constraint, 1); }
		void AddConstraints(Constraint** constraints, uint32 count) { return mConstraintCoordinator.Add(constraints, count); }
		void RemoveConstraint(Constraint* constraint) { return mConstraintCoordinator.Remove(&constraint, 1); }
		void RemoveConstraints(Constraint** constraints, uint32 count) { return mConstraintCoordinator.Remove(constraints, count); }

		Constraints& NonContactConstraints() { return mConstraintCoordinator.GetConstraints(); }



		/// Broadphase
		void SetBroadphaseNodeBoundThreshold(float treshold) { mBroadphase->SetBoundThreshold(treshold); }
		//TODO(Jay): Later dont return broadphase only state
		VX_INLINE Broadphase* GetBroadphase() { return mBroadphase; }
		//VX_INLINE BVHBroadphase<AABB>* GetBVH_AABB_Broadphase() { return mBroadphase->AsBVH_AABB(); }
		const BroadphasePair* BroadphasePairsPtr() const { return mSimStep.broadphasePair; }
		const uint32 BroadphasePairsCount() const { return mSimStep.broadphasePairCount; }


		/// Narrowphase
		const CollisionResolutionStat NarrowphaseStats() const;

		/// Solver
		const ContactConstraintSolver* ContactConstraintCoordinator() const { return &mContactConstraintSolver; }
		ConstraintSolver* GetConstraintSolver() const { return mConstraintSolver; }
		const ContactConstraintSolver::ContactConstraintSolverStat& ContactConstraintSolverStats() const 
		{ return mContactConstraintSolver.Stats(); }


		/// Visuals 
		void SetContextDebugGizmos(DebugGizmosRenderer* debug_renderer) 
		{ 
			mContext.mDebugRenderer = debug_renderer; 
			mWorldQuery.SetDebugRender(debug_renderer);
		}

		template<EShapeType Type>
		void OnDrawBody(const Body& body, Renderer* draw_renderer,
			const RenderSettings& setting, const Colour& c = Colour::sMagenta);
		void OnDrawBodies(Renderer* draw_renderer, const RenderSettings& setting);
		void OnDebugDraw(DebugGizmosRenderer* debug_renderer);
		void DrawBodyInertia(DebugGizmosRenderer* debug_renderer);


	private:
		Colour BodySimphaseDebugColour(const Body& body) const;
		void UpdateBodiesIslandActivationState(float dt);


		PhysicsStepContext mContext;
		SimStep mSimStep;
		PhysicsWorldSettings* mSettings;
		static uint32 mStepIndex;

		class ScratchAllocator* mScratchAllocator = nullptr;
		BodyManager mBodyManager;

		TaskCoordinator* mTaskCoordinator = nullptr;
		std::vector<class Task*> mProcessPairAndTrySetupContactConstraintTasks;

		Broadphase* mBroadphase = nullptr;
		NarrowphaseQuery mNarrowphaseQuery;


		class IslandCoordinator* mIslandCoordinator = nullptr;

		ContactConstraintSolver mContactConstraintSolver;
		/// non contact corrdinator 
		ConstraintCoordinator mConstraintCoordinator;
		class ConstraintSolver* mConstraintSolver = nullptr;


		WorldQuery mWorldQuery;


	public:
		VelocitySolveProfile mVelocitySolveProfile;
#if defined(VX_DEBUG_ALLOCATOR)
		///debug allocator
		void* testAllocation;
		size_t testAllocationSize;
#endif // defined(VX_DEBUG_ALLOCATOR)
	};
}