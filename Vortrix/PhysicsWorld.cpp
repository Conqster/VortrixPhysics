#include "PhysicsWorld.h"

#include "Core/Profiler.h"

#include "Dynamics/Body/Body.h"
#include "Dynamics/Body/BodySimStats.h"

#include "Collision/Shapes/Shape.h"
#include "Collision/Shapes/SphereShape.h"
#include "Collision/Shapes/BoxShape.h"
#include "Collision/Shapes/PlaneShape.h"
#include "Collision/Shapes/CapsuleShape.h"

#include "Collision/BoxBoxContactDebug.h"

#include "Geometry/AABB.h"
#include "Geometry/OBB.h"
#include "Collision/Broadphase/BVHBroadphase.h"

#include "Vortrix/Visuals/Renderers.h"


#include "Collision/Broadphase/BruteforceBroadphase.h"

#include "Collision/Narrowphase/NarrowphaseQuery.h"
#include "Collision/Narrowphase/NarrowphaseQueryStat.h"

#include "Visuals/ERenderInstanceFlags.h"
#include "Visuals/RenderSettings.h"

#include "SimulationContexts.h"

#include "Dynamics/ConstraintCoordinator.h"
#include "Dynamics/ConstraintSolver.h"

#include "Core/ScratchAllocator.h"

#include "Core/TaskCoordinator.h"
#include "Dynamics/IslandCoordinator.h"

#include "Vortrix/Core/Colours.h"


namespace vx
{
	uint32 PhysicsWorld::mStepIndex = 0;

	PhysicsWorld::PhysicsWorld(PhysicsWorldSettings* in_settings) :
		mSettings(in_settings)
	{
	}

	PhysicsWorld::~PhysicsWorld()
	{
		delete mBroadphase;

		delete mConstraintSolver;

		delete mIslandCoordinator;

#if defined(VX_DEBUG_ALLOCATOR)
		mScratchAllocator->Free(testAllocation, testAllocationSize);
		delete mScratchAllocator;
#endif // defined(VX_DEBUG_ALLOCATOR)

		mTaskCoordinator->Quit();
		delete mTaskCoordinator;
	}

	void PhysicsWorld::CreateSimpleWorld(PhysicsWorld* io_world)
	{
		VX_ASSERT(io_world, "Physics World is null");

		static float no_static_bodies = 0.0f;

		bool create_capsules = true;
		bool create_boxes = true;

		BodySettings dyn_bodies_settings = BodySettings::DefaultDynamicConstruct();
		BodySettings static_bodies_settings = BodySettings::DefaultStaticConstruct();
		if (create_boxes)
		{
			Ref<BoxShape> unit_box = MakeRef<BoxShape>(0.5f);
			dyn_bodies_settings.position = Vec3(-2.0f, 5.5f, 0.0f);
			dyn_bodies_settings.debug_name = "box";
			dyn_bodies_settings.shape = MakeRef<BoxShape>(0.5f, 0.25f, 0.5f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(2.0f, 3.0f, 0.0f);
			dyn_bodies_settings.shape = MakeRef<BoxShape>(0.15f, 0.6f, 0.35f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.shape = unit_box;
			dyn_bodies_settings.position = Vec3(-4.0f, 5.5f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(-4.0f, 10.0f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(1.0f, 7.5f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(0.0f, 12.5f, 0.0f);
			dyn_bodies_settings.shape = MakeRef<BoxShape>(1.5f, 0.15f, 0.15f);
			io_world->CreateBody(dyn_bodies_settings);
		}

		vx::SphereShapeSettings shape_settings(0.5f);
		shape_settings.SetDensity(0.0f);
		Ref<SphereShape> static_unit_sphere = MakeRef<SphereShape>(shape_settings);
		static_bodies_settings = BodySettings::DefaultStaticConstruct();
		static_bodies_settings.position = Vec3(3.0f, 5.5f, 0.0f);
		static_bodies_settings.debug_name = "sphere";
		static_bodies_settings.shape = static_unit_sphere;
		io_world->CreateBody(static_bodies_settings);

		io_world->CreateBody(static_bodies_settings);
		static_bodies_settings.position = Vec3(6.0f, 5.5f, 0.1f);
		io_world->CreateBody(static_bodies_settings);
		static_bodies_settings.position = Vec3(4.5f, 8.35f, 0.0f);
		io_world->CreateBody(static_bodies_settings);
		static_bodies_settings.position = Vec3(4.5f, 3.3f, -0.1f);
		io_world->CreateBody(static_bodies_settings);


		/// Ground plane
		vx::PlaneShapeSettings plane_shape_settings(vx::Vec3::Up(), 100.0f);
		plane_shape_settings.SetDensity(0.0f);
		static_bodies_settings = BodySettings::DefaultStaticConstruct();
		static_bodies_settings.debug_name = "ground";
		static_bodies_settings.shape = MakeRef<PlaneShape>(plane_shape_settings);
		io_world->CreateBody(static_bodies_settings);

		/// Create side planes as well
		///front 
		/// back
		/// right
		/// left
		/// 
		/// front offset along z and face -z (rotate around -x)
		BodySettings body_settings = BodySettings::DefaultStaticConstruct();
		body_settings.debug_name = "front plane";
		body_settings.shape = static_bodies_settings.shape;///*MakeRef<PlaneShape>(Vec3::Up(), 100.0f);*/
		
		float world_size = 100.0f;
		body_settings.position = Vec3(0.0f, 0.0f, world_size);
		body_settings.orientation.SetAxisAngle(Vec3::Right(), DegToRad(-90.0f));
		io_world->CreateBody(body_settings);

		/// front offset along -z and face z
		body_settings.debug_name = "back plane";
		body_settings.position = Vec3(0.0f, 0.0f, -world_size);
		body_settings.orientation.SetAxisAngle(Vec3::Right(), DegToRad(90.0f));
		io_world->CreateBody(body_settings);

		/// front offset along x and face -x (rotate around z
		body_settings.debug_name = "right plane";
		body_settings.position = Vec3(world_size, 0.0f, 0.0f);
		body_settings.orientation.SetAxisAngle(Vec3::Forward(), DegToRad(90.0f));
		io_world->CreateBody(body_settings);

		/// front offset along -x and face x (rotate around z)
		body_settings.debug_name = "left plane";
		body_settings.position = Vec3(-world_size, 0.0f, 0.0f);
		body_settings.orientation.SetAxisAngle(Vec3::Forward(), DegToRad(-90.0f));
		io_world->CreateBody(body_settings);

		if(create_capsules)
		{
			dyn_bodies_settings = BodySettings::DefaultDynamicConstruct();
			dyn_bodies_settings.debug_name = "Capsule";
			dyn_bodies_settings.shape = MakeRef<CapsuleShape>(0.5f, 0.5f);
			dyn_bodies_settings.position = Vec3(4.0f, 10.0f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(7.5f, 10.0f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(-4.5f, 10.0f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(-7.5f, 12.0f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(0.5f, 12.0f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);
		}


		
		if(create_boxes)
		{
			dyn_bodies_settings.position = Vec3(-2.0f, 5.5f, 0.0f);
			dyn_bodies_settings.debug_name = "box";
			dyn_bodies_settings.shape = MakeRef<BoxShape>(0.5f, 0.25f, 0.5f);
			io_world->CreateBody(dyn_bodies_settings);


			dyn_bodies_settings.position = Vec3(1.0f, 7.0f, 0.0f);
			dyn_bodies_settings.shape = MakeRef<BoxShape>(0.15f, 0.6f, 0.35f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(0.0f, 5.5f, 0.0f);
			dyn_bodies_settings.shape = MakeRef<BoxShape>(0.25f, 0.125, 0.25f);
			io_world->CreateBody(dyn_bodies_settings);


			dyn_bodies_settings.position = Vec3(3.0f, 7.5f, 0.0f);
			dyn_bodies_settings.shape = MakeRef<BoxShape>(1.0f, 0.25f, 0.25f);
			io_world->CreateBody(dyn_bodies_settings);


			dyn_bodies_settings.position = Vec3(0.0f, 12.5f, 0.0f);
			dyn_bodies_settings.shape = MakeRef<BoxShape>(1.5f, 0.15f, 0.15f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(0.0f, 10.0f, 0.0f);
			dyn_bodies_settings.shape = MakeRef<BoxShape>(1.0f, 0.25f, 1.0f);
			io_world->CreateBody(dyn_bodies_settings);
		}

	}

	void PhysicsWorld::GenerateWorldDefaultConfig(int& o_max_bodies, int& o_max_body_pairs, int& o_max_contact_constraint)
	{
		o_max_bodies = 10240;// 16384;

		int avg_max_pair = 4;//8;
		o_max_body_pairs = avg_max_pair * o_max_bodies;

		int avg_contact_per_body = 4;
		o_max_contact_constraint = avg_contact_per_body * o_max_bodies;
	}

	void PhysicsWorld::Init(float max_bodies, float max_body_pairs, float max_contact_constraint)
	{
		VX_PROFILE_FUNCTION();
		//heavily using point to data in vector in Broadphase 
		//at the moment so let reseve 
		mBodyManager.Init(max_bodies);
		mContext.bodyManager = &mBodyManager;
		mContext.mStepIndex = 0;

		BroadphaseInitInfo broad_init;
		broad_init.maxDirtyBodies = mBodyManager.MaxBodies() * 0.6f;
		mBroadphase = new BVHBroadphase<AABB>();
		mBroadphase->SetBoundThreshold(mSettings->collision.boundsMargin);
		mBroadphase->Init(&mBodyManager, broad_init);

		mSettings->collision.maxPairs = max_body_pairs;

		mContactConstraintSolver.Init(max_contact_constraint);
		mContactConstraintSolver.SetPhysicsContext(&mContext);

		mContactConstraintSolver.SetRestitutionCombineMode(mSettings->solver.restitutionCombineMode);
		mContactConstraintSolver.SetFrictionCombineMode(mSettings->solver.frictionCombineMode);

		mConstraintSolver = new ConstraintSolver;
		mConstraintSolver->Init(mBodyManager);

		mWorldQuery.Init(mBroadphase);

		VX_ASSERT(mSettings->scratchAllocationMiB > 0.0f);
		mScratchAllocator = new ScratchAllocator(static_cast<size_t>(mSettings->scratchAllocationMiB * 1024.0f * 1024.0f));


#if defined(VX_DEBUG_ALLOCATOR)
		testAllocationSize =
			sizeof(Linear1DRow) * 2000 +
			//sizeof(BroadphasePair) * max_body_pairs +
			sizeof(Constraint*) * 250;
		testAllocation = mScratchAllocator->Allocate(testAllocationSize);
#endif // defined(VX_DEBUG_ALLOCATOR)

		int num_threads = 0;
		num_threads = std::thread::hardware_concurrency() - 1;
		mTaskCoordinator = new TaskCoordinatorMt(VxMin(mSettings->maxConcurrency, num_threads));


		mIslandCoordinator = new IslandCoordinator;
		mIslandCoordinator->Init(mBodyManager.MaxBodies(), mContactConstraintSolver.MaxConstraints(), 10800);
	}

	Body* PhysicsWorld::CreateBody(const BodySettings& body_setting, bool activate_body)
	{
		const BodyID body_id = mBodyManager.AddBody(body_setting);
		
		if (!body_id.IsValid())
			return nullptr;

		if (body_setting.inBroadphase)
		{
			VX_ASSERT(mBroadphase);
			mBroadphase->InsertBody(&mBodyManager.GetBody(body_id));

			if (activate_body && body_setting.motionType != EMotionType::Static)
				ActivateBodies(&body_id, 1);

			mBodyManager.GetBodyDebugInfo(body_id).bodyInBroadphase = true;
		}
		else
			VX_ASSERT(!activate_body, "To activate Body has to participate in broadphase");


		return &mBodyManager.GetBody(body_id);
	}

	void PhysicsWorld::RemoveBody(const BodyID& id)
	{
		mBroadphase->RemoveBody(id);
		mBodyManager.RemoveBody(id);
	}

	void PhysicsWorld::RemoveBodies(const BodyID* ids, uint32 count)
	{
		VX_ASSERT(count > 0);

		for (const BodyID* id = ids, *ids_end = ids + count;
			id < ids_end; ++id)
			RemoveBody((*id));
	}


	void PhysicsWorld::StepSimulation(float dt)
	{
		//VX_PROFILE_FUNCTION();
		VX_PROFILE_SCOPE("Simulation Step");
		Vec3 sample_gravity_vel = mSettings->gravity * mSettings->gravityScale;
		sample_gravity_vel *= dt;
		mSettings->frameGravityVelocity = sample_gravity_vel.ToFloat3();
		mContext.mDeltaTime = dt;
		mContext.mGravity = mSettings->gravity * mSettings->gravityScale;
		mContext.mSettings = mSettings;
		mContext.mScratchAllocator = mScratchAllocator;
		mContext.mIslandCoordinator = mIslandCoordinator;

		mContext.velocityIterations = mSettings->solver.velocityIterations;
		mContext.enableContact = mSettings->solver.enableContact;

		//new 
		mContext.mPhysicsWorld = this;
		mContext.constraintSolver = mConstraintSolver;
		mContext.mDrawSettings = mSettings->drawSettings;

		mWorldQuery.SetDrawBroadphaseNodesWalked(mSettings->drawSettings->drawWalkedTreeQuery);

#if defined(VX_DEBUG_ALLOCATOR)
		mScratchAllocator->ResetDebugAlloc();
#endif // defined VX_DEBUG_ALLOCATOR

		{
			VX_PROFILE_SCOPE("Reset Simulation State");
			for (auto& body_debug : mBodyManager.GetBodiesDebug())
				body_debug.simulationStats.Reset();
		}

		mSimStep.mPhysicsStepContext = &mContext;
		mSimStep.broadphasePair = reinterpret_cast<BroadphasePair*>(mScratchAllocator->Allocate(sizeof(BroadphasePair) * mSettings->collision.maxPairs));
		mSimStep.broadphasePairCount = 0;
		mSimStep.nextProcessPairIdx = { 0 };
		mSimStep.solveVelocityNextIslandIdx = { 0 };
		mSimStep.solvePositionNextIslandSortedIdx = { 0 };
		////////////////////////////////////
		// Broadphase
		////////////////////////////////////

		{
			VX_PROFILE_SCOPE("Broadphase acceleration Integration");
			VX_ASSERT(mBroadphase != nullptr);
			//// broadphase could happen before gravity + acceleration intergration 
			/// broadphase only reads bodies (position, and type), and write debug data to body manager
			/// a thread start work immediately
			mTaskCoordinator->ConstructTask([broadphase = mBroadphase, ctx = &mContext, sim_step = &mSimStep]()
				{
					VX_PROFILE_SCOPE("Broadphase Computing Pairs");
					broadphase->ComputeCollidingPair(*ctx, *sim_step);
				}, 0);


			

			/// clamp acceleration & gravity tasks count to max concurrency - 1 (leaving the a thread for working broadphase)
			const int max_acceleration_workers = VxMax(1, int(mTaskCoordinator->MaxConcurrency() - 1));
			uint32 num_acceleration_gravity_tasks = VxMin(int((NumActiveBodies() + (SimStep::kAcclerationGravityTaskBatch - 1)) / SimStep::kAcclerationGravityTaskBatch), max_acceleration_workers);

			std::atomic<uint32> next_active_body_idx = 0;
			for (uint32 i = 0; i < num_acceleration_gravity_tasks; ++i)
			{
				mTaskCoordinator->ConstructTask([step_ctx = &mContext, &next_active_body_idx,
					world_gravity = mSettings->gravity, gravity_scale = mSettings->gravityScale]()
					{
						VX_PROFILE_SCOPE("Integrate bodies acceleration");
						uint32 num_active_bodies = step_ctx->bodyManager->NumActiveBodies();
						BodyID* active_bodies = step_ctx->bodyManager->ActiveBodies();

						const vx::Vec3 gravity = world_gravity * gravity_scale;

						/// atomically fetch batch and process
						for (;;)
						{
							const uint32 active_body_begin = next_active_body_idx.fetch_add(SimStep::kAcclerationGravityTaskBatch);

							/// at the end of broadphase pair
							if (active_body_begin >= num_active_bodies)
								break;

							uint32 active_body_end = VxMin(num_active_bodies, active_body_begin + SimStep::kAcclerationGravityTaskBatch);

							for (uint32 i = active_body_begin; i < active_body_end; ++i)
							{
								Body& body = step_ctx->bodyManager->GetBody(active_bodies[i]);
								if (body.IsDynamic())
								{
									body.IntegrateAcceleration(step_ctx->mDeltaTime, gravity);
									body.ClearAccumulatedForces();
								}
							}
						}
					}, 0);
			}



			/// Allow main thread to compute this before waiting 
			//////////////////////////////////
			// Reset/clear sub systems
			//////////////////////////////////
			mContactConstraintSolver.PreFrameSetup(); //for per frame transient allcation for now
			/// for now need to invalidate previous frame local bodies 
			/// so the bodies could be update for use by narrowphase handshake 
			/// with contact constraint, fix later 
			{
				VX_PROFILE_SCOPE("Clear Non contact constraint");
				mConstraintSolver->HackClear();
			}
			mIslandCoordinator->PrepareIslands((uint32)mBodyManager.GetBodies().size());

			/////wait for acceleration, gravity, broadphase, clearing sub systems 
			mTaskCoordinator->WaitForTasks();
		}

#if USE_MULTITHREAD
		{
			VX_PROFILE_SCOPE("Narrowphase Process Pair and Island Building");

			/// use max possible concurrency
			uint32 num_process_pair_try_setup_constraint_tasks = VxMin(int((mSimStep.broadphasePairCount + (SimStep::kProcessBodyPairBatch - 1)) / SimStep::kProcessBodyPairBatch), int(mTaskCoordinator->MaxConcurrency()));
			for (uint32 i = 0; i < num_process_pair_try_setup_constraint_tasks; ++i)
			{
				mTaskCoordinator->ConstructTask([sim_step = &mSimStep]()
					{
						uint32 total_broadphase_pair_count = sim_step->broadphasePairCount;

						/// atomically fetch batch and process
						for (;;)
						{
							const uint32 process_batch_begin = sim_step->nextProcessPairIdx.fetch_add(SimStep::kProcessBodyPairBatch);

							/// at the end of broadphase pair
							if (process_batch_begin >= total_broadphase_pair_count)
								break;

							uint32 process_batch_end = VxMin(total_broadphase_pair_count, process_batch_begin + SimStep::kProcessBodyPairBatch);

							for (uint32 i = process_batch_begin; i < process_batch_end; ++i)
							{
								auto& pair = sim_step->broadphasePair[i];

								///probably make this a static function
								///also seperate processing narrow phase away and setup 
								/// 
								/// 
								sim_step->mPhysicsStepContext->mPhysicsWorld->mNarrowphaseQuery.ProcessPairAndTrySetupContactConstraint(
									pair.a,
									pair.b,
									sim_step->mPhysicsStepContext->mPhysicsWorld->mContactConstraintSolver, sim_step);
							}
						}
					}, 0);
			}


			mTaskCoordinator->WaitForTasks();
		}
#else
		mNarrowphaseQuery.ProcessPairs(mBroadphaseBuffer.data, mContactConstraintSolver, sim_step);
#endif // USE_MULTITHREAD

#if TEST_CONTACT_CONSTRAINT_MT
		mContactConstraintSolver.FinaliseWriteManifoldCache();
#endif // TEST_CONTACT_CONSTRAINT_MT


		mContext.contactConstraintCount = mContactConstraintSolver.NumContactConstraints();
		mContext.nonContactConstraintCount = 0;
		/// free broadphase data straight after narrowphase
		mScratchAllocator->Free(mSimStep.broadphasePair, sizeof(BroadphasePair) * mSettings->collision.maxPairs);
		mSimStep.broadphasePair = nullptr;
		/// Prepare non contact constraint 
		/// contact constraint should be done
		/// time to prep joint constraints 
		const uint32 active_joint_constraint_count = mConstraintCoordinator.PrepConstraintSolving(*mConstraintSolver, mContext);
		/// Active Body should have been linked 
		/// Body Contact Constraint 
		/// Body Non Contact Constraint
		mIslandCoordinator->FinaliseIslands(*mConstraintSolver, mContactConstraintSolver.NumContactConstraints(), active_joint_constraint_count, mBodyManager, mScratchAllocator);



	
		if(mSettings->splitLargeIsland)
		{
			VX_PROFILE_SCOPE("Splitting Islands");
			mIslandCoordinator->GetSplitter().Prepare(mBodyManager.NumActiveBodies(), *mIslandCoordinator, mScratchAllocator);

			for (uint32 i = 0; i < mBodyManager.NumActiveBodies(); ++i)
			{
				auto& body = mBodyManager.GetBody(mBodyManager.GetActiveBodyID(i));
				body.mIslandConstraintGroupMask = 0;
			}

			for (uint32 i = 0; i < mIslandCoordinator->IslandCount(); ++i)
			{
				/// split larger island ffirst
				uint32 island = mIslandCoordinator->SortedIslandIndices()[i];
				mIslandCoordinator->GetSplitter().SplitIsland(island, *mIslandCoordinator, 
					&mContactConstraintSolver, mConstraintSolver, 
					&mBodyManager, mScratchAllocator);
			}
		}


		if (mSettings->solver.enable)
		{

			/// Requirements 
			///
			/// availabe parameters
			///	- BodyID/actual body
			/// - SolverBody & SolverIndex
			/// 
			/// - ContactConstraint
			/// - Constraint (Non-Contact)
			/// >>>>
			/// 
			/// CONTACT section
			/// - broadphase pair processed in narrowphase
			/// - constructs contact constraint + solver body + LinkBodies() &/LinkContact()
			/// 
			/// requirement for solving
			/// * get constraint to solve for (this helps for selective island specific solving) task based
			/// 
			/// 
			/// * sort bodies, for instance 
			/// | 0 | 1 | 1 | 1 | 0 | 1 | 0 | 0 | 0 
			///  - to become 
			/// | 0 | 0 | 0 | 0 | 0 | 1 | 1 | 1 | 1
			///  - for easy GetIslandBodies
			/// * sort constraint, similar to bodies for GetIslandNonContactConstraints & GetIslandContactConstraints
			/// 
			/// * importantly its important to sort SolverBodyIdx to adhere with the island it leaves in
			/// 



#define SOLVER_CONSTRAINT_VIA_ISLAND 1



#if CONTACT_USE_SOLVERBODY

			Linear1DRow* constraint_solver_rows = mConstraintSolver->GetLinearRowPtr();
			uint32 constraint_solver_row_count = mConstraintSolver->LinearRowCount();
			SolverBody* solver_bodies = mConstraintSolver->GetBodiesPtr();

			/// perform warm starts
			if (mSettings->solver.warmstart)
			{
				ConstraintSolver::WarmStart(constraint_solver_rows, 0, constraint_solver_row_count, solver_bodies);
				//fix this bad nested if branches
				if (mSettings->solver.enableContact)
					ContactConstraintSolver::WarmStart(mContactConstraintSolver.ContactConstraintsPtr(), mContactConstraintSolver.NumContactConstraints(), solver_bodies);
			}

#if SOLVER_CONSTRAINT_VIA_ISLAND
			//////solve islands multicore 
			{
				VX_PROFILE_SCOPE("Solving Velocity Constraints");

#define TEST_SOLVER_CONSTRAINT_MT 1
#if TEST_SOLVER_CONSTRAINT_MT

				/// create task as the count; thread workers
				for(uint32 i = 0; i < mTaskCoordinator->MaxConcurrency(); ++i)
				{
					mTaskCoordinator->ConstructTask([io_physics_ctx = &mContext, io_step = &mSimStep]()
						{

#define VX_SPLIT_ISLAND 1
							const uint32 velocity_iteration_count = io_physics_ctx->velocityIterations;

							uint32 total_jacobian_row_solved = 0;

#if VX_SPLIT_ISLAND
							const uint32 island_count = io_physics_ctx->mIslandCoordinator->IslandCount();
							const bool warm_start = io_physics_ctx->mPhysicsWorld->mSettings->solver.warmstart;
							bool normal_island_complete = false;
							const bool sim_split_large_island = io_physics_ctx->mPhysicsWorld->mSettings->splitLargeIsland;
							bool large_island_complete = !sim_split_large_island;
							///NEW+
							for (;;)
							{
								bool worked = false;

								if (large_island_complete && normal_island_complete)
									break;


								//auto* solver_bodies = solving_island_ctx.constraintSolver.GetBodiesPtr();
								SolverBody* solver_bodies = io_physics_ctx->mPhysicsWorld->mConstraintSolver->GetBodiesPtr();

								//if(solving_island_ctx.splitLargeIsland)
								//if(!large_island_complete && sim_split_large_island)
								if(sim_split_large_island)
								{
									///try splitting island 
									IslandCoordinator::IslandRange<uint32> island_contact_range(nullptr, nullptr), island_noncontact_range(nullptr, nullptr);
									uint32 solving_split_island_idx;
									int batch_first_iteration = -1;
									uint32 debug_bin = uint32(-1);
	/*								IslandCoordinator::Splitter::EStatus status = solving_island_ctx.islandCoordinator->GetSplitter().NextConstactConstraintBatchRange(solving_split_island_idx,
										island_count, island_contact_range, island_noncontact_range, debug_bin, solving_island_ctx.islandCoordinator->SortedIslandIndices());*/
									IslandCoordinator::Splitter::EStatus status = io_physics_ctx->mIslandCoordinator->GetSplitter().NextConstactConstraintBatchRange(solving_split_island_idx,
										island_count, island_contact_range, island_noncontact_range, batch_first_iteration, debug_bin, io_physics_ctx->mIslandCoordinator->SortedIslandIndices());

									switch (status)
									{
									case vx::IslandCoordinator::Splitter::EStatus::Complete:
										large_island_complete = true;
										break;
									case vx::IslandCoordinator::Splitter::EStatus::WaitingForBatches:
										break;
									case vx::IslandCoordinator::Splitter::EStatus::RetrievedBatch:
									{
										VX_PROFILE_SCOPE("Solving Large Island Batch");
										worked = true;

										///// perform warm starts
										//if ((batch_first_iteration == 0) && warm_start)
										//{
										//	if (island_noncontact_range.Valid())
										//	{
										//		ConstraintSolver::WarmStart(island_noncontact_range.begin, island_noncontact_range.Size(),
										//			io_physics_ctx->constraintSolver->GetLinearRowPtr(), io_physics_ctx->constraintSolver->LinearRowCount(),
										//			solver_bodies, uint32(io_physics_ctx->constraintSolver->GetBodiesCount()));
										//	}



										//	//fix this bad nested if branches
										//	if (io_physics_ctx->enableContact && island_contact_range.Valid())
										//	{
										//		ContactConstraintSolver::WarmStart(island_contact_range.begin, island_contact_range.Size(),
										//			io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.ContactConstraintsPtr(), io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.NumContactConstraints(),
										//			solver_bodies, uint32(io_physics_ctx->constraintSolver->GetBodiesCount()));
										//	}
										//}

										//solve batch
										uint32 processed_count = 0;
										if (island_noncontact_range.Valid())
										{
											io_physics_ctx->constraintSolver->SolverVelocityLinear1DRowsIndices(island_noncontact_range.begin, island_noncontact_range.Size());
											processed_count += island_noncontact_range.Size();
											total_jacobian_row_solved += island_noncontact_range.Size();
										}


										if (io_physics_ctx->enableContact && island_contact_range.Valid())
										{
											io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.SolveVelocityConstraint(island_contact_range.begin, island_contact_range.Size(), solver_bodies);
											processed_count += island_contact_range.Size();
											total_jacobian_row_solved += island_contact_range.Size() * ContactConstraintSolver::kContactConstraintAxisCount;
										}

										/// couple of issues 
										/// 1. range of value, null return 2 as size instead of 1
										/// 2. submitting/marking batch but bin as already processed

										///mark batch as processed 
										bool batch_last_iteration = false;
										io_physics_ctx->mIslandCoordinator->GetSplitter().MarkConstactConstraintBatchRangeComplete(solving_split_island_idx, processed_count, velocity_iteration_count, batch_last_iteration, debug_bin);


										if (batch_last_iteration && warm_start)
										{

											if (island_noncontact_range.Valid())
											{
												ConstraintSolver::CommitStateConstraint(island_noncontact_range.begin, island_noncontact_range.Size(),
													io_physics_ctx->constraintSolver->GetLinearRowPtr(), io_physics_ctx->constraintSolver->LinearRowCount(),
													io_physics_ctx->mPhysicsWorld->mConstraintCoordinator.GetConstraints().data(), io_physics_ctx->mPhysicsWorld->mConstraintCoordinator.GetConstraints().size());
											}

											if (io_physics_ctx->enableContact && island_contact_range.Valid())
											{
												ContactConstraintSolver::WriteBackImplusesManifoldCache(island_contact_range.begin, island_contact_range.Size(),
													io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.ContactConstraintsPtr(), io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.NumContactConstraints());
										
											}
										}



										continue;
									}
									break;
									default:
										break;
									}
								}




								///solve island as small island
								/// find the first not large island to solve
								/// 
								uint32 normal_island_indices_idx = io_step->solveVelocityNextIslandIdx.load(std::memory_order_relaxed);
								if (normal_island_indices_idx >= island_count)
								{
									normal_island_complete = true;
									continue;
								}

								const uint32 normal_island_idx = io_physics_ctx->mIslandCoordinator->SortedIslandIndices()[normal_island_indices_idx];
								if (!sim_split_large_island || !io_physics_ctx->mIslandCoordinator->GetSplitter().IsIslandLarge(normal_island_idx))
								{
									VX_PROFILE_SCOPE("Solving Island");
									/// race condition
									if (io_step->solveVelocityNextIslandIdx.compare_exchange_strong(normal_island_indices_idx, normal_island_indices_idx + 1))
									{
										worked = true;
										IslandCoordinator::IslandRange<uint32> constraint_island_indices_range = io_physics_ctx->mIslandCoordinator->IslandNonContactConstraintRowIndicesRange(normal_island_idx);
										IslandCoordinator::IslandRange<uint32> contact_constraint_island_indices_range = io_physics_ctx->mIslandCoordinator->ContactConstraintIndicesIslandRange(normal_island_idx);

										///// perform warm starts
										//if (warm_start)
										//{
										//	if (constraint_island_indices_range.Valid())
										//	{
										//		ConstraintSolver::WarmStart(constraint_island_indices_range.begin, constraint_island_indices_range.Size(),
										//			io_physics_ctx->constraintSolver->GetLinearRowPtr(), io_physics_ctx->constraintSolver->LinearRowCount(),
										//			solver_bodies, uint32(io_physics_ctx->constraintSolver->GetBodiesCount()));
										//	}

										//	//fix this bad nested if branches
										//	if (io_physics_ctx->enableContact && contact_constraint_island_indices_range.Valid())
										//	{
										//		ContactConstraintSolver::WarmStart(contact_constraint_island_indices_range.begin, contact_constraint_island_indices_range.Size(),
										//			io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.ContactConstraintsPtr(), io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.NumContactConstraints(),
										//			solver_bodies, uint32(io_physics_ctx->constraintSolver->GetBodiesCount()));
										//	}
										//}


										for (int i = 0; i < velocity_iteration_count; ++i)
										{
											if (constraint_island_indices_range.Valid())
											{
												io_physics_ctx->constraintSolver->SolverVelocityLinear1DRowsIndices(constraint_island_indices_range.begin, constraint_island_indices_range.Size());
												total_jacobian_row_solved += constraint_island_indices_range.Size();
											}


											if (io_physics_ctx->enableContact && contact_constraint_island_indices_range.Valid())
											{
												io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.SolveVelocityConstraint(contact_constraint_island_indices_range.begin, contact_constraint_island_indices_range.Size(), solver_bodies);
												total_jacobian_row_solved += contact_constraint_island_indices_range.Size() * ContactConstraintSolver::kContactConstraintAxisCount;
											}
										}


										/// commit state 
										if (warm_start)
										{
											if (constraint_island_indices_range.Valid())
											{
												ConstraintSolver::CommitStateConstraint(constraint_island_indices_range.begin, constraint_island_indices_range.Size(),
													io_physics_ctx->constraintSolver->GetLinearRowPtr(), io_physics_ctx->constraintSolver->LinearRowCount(),
													io_physics_ctx->mPhysicsWorld->mConstraintCoordinator.GetConstraints().data(), io_physics_ctx->mPhysicsWorld->mConstraintCoordinator.GetConstraints().size());
											}

											if (io_physics_ctx->enableContact && contact_constraint_island_indices_range.Valid())
											{
												ContactConstraintSolver::WriteBackImplusesManifoldCache(contact_constraint_island_indices_range.begin, contact_constraint_island_indices_range.Size(),
													io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.ContactConstraintsPtr(), io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.NumContactConstraints());
											}
										}
									}
								}
								else
								{
									///increment for next iteration
									/// only increment is some else as not 
									io_step->solveVelocityNextIslandIdx.compare_exchange_strong(normal_island_indices_idx, normal_island_indices_idx + 1);
								}


								//if (large_island_complete && normal_island_complete)
								//	break;

								if (!worked)
								{
									std::this_thread::yield();
								}
							}


							///complete solve 
							///writing back order does not matter
							//uint32 step_total_jacobian_solved[12]; ///later only thread in flight for current step =task count
							//std::atomic<uint32> write_total_jacobian_solved;

							//uint32 write_jacobian = write_total_jacobian_solved.load(std::memory_order_acquire);
							//for (;;)
							//{
							//	if (write_total_jacobian_solved.compare_exchange_weak(write_jacobian, write_jacobian + 1, std::memory_order_release))
							//		step_total_jacobian_solved[write_jacobian] = total_jacobian_row_solved;

							//	_mm_pause();
							//	write_jacobian = write_total_jacobian_solved.load(std::memory_order_acquire);
							//}


							/////after all threads complete
							///// sort and write final
							//std::sort(step_total_jacobian_solved[0], step_total_jacobian_solved[11], [&](uint32 a, uint32 b)
							//	{
							//		return a < b;
							//	});


							if(total_jacobian_row_solved > 0)
							{
								uint32 claimed_write_idx = io_step->mSimStepProfiling.writeStepTotalJacobianSolved.fetch_add(1, std::memory_order_acq_rel);
								io_step->mSimStepProfiling.total_jacobian_solved[claimed_write_idx] = total_jacobian_row_solved;
							}

#else
						VX_ASSERT(false);
							for (;;)
							{
								const uint32 island_idx = next_island.fetch_add(1, std::memory_order_relaxed);

								if (island_idx >= island_count)
									break;

								IslandCoordinator::IslandRange<uint32> constraint_island_indices_range = solving_island_ctx.islandCoordinator->IslandNonContactConstraintRowIndicesRange(island_idx);
								IslandCoordinator::IslandRange<uint32> contact_constraint_island_indices_range = solving_island_ctx.islandCoordinator->ContactConstraintIndicesIslandRange(island_idx);

								auto* solver_bodies = solving_island_ctx.constraintSolver.GetBodiesPtr();


								/// perform warm starts
								if (warm_start)
								{
									if (constraint_island_indices_range.Valid())
										ConstraintSolver::WarmStart(constraint_island_indices_range.begin, constraint_island_indices_range.Size(),
											io_physics_ctx->constraintSolver->GetLinearRowPtr(), io_physics_ctx->constraintSolver->LinearRowCount(),
											solver_bodies, uint32(io_physics_ctx->constraintSolver->GetBodiesCount()));

									//fix this bad nested if branches
									if (io_physics_ctx->enableContact && contact_constraint_island_indices_range.Valid())
										ContactConstraintSolver::WarmStart(contact_constraint_island_indices_range.begin, contact_constraint_island_indices_range.Size(),
											io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.ContactConstraintsPtr(), io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.NumContactConstraints(),
											solver_bodies, uint32(io_physics_ctx->constraintSolver->GetBodiesCount()));
								}

								for (int i = 0; i < velocity_iteration_count; ++i)
								{
									if (constraint_island_indices_range.Valid())
										solving_island_ctx.constraintSolver.SolverVelocityLinear1DRowsIndices(constraint_island_indices_range.begin, constraint_island_indices_range.Size());


									if (solving_island_ctx.enableContact && contact_constraint_island_indices_range.Valid())
										solving_island_ctx.contactConstraintSolver.SolveVelocityConstraint(contact_constraint_island_indices_range.begin, contact_constraint_island_indices_range.Size(), solver_bodies);

								}


								/// commit state 
								if (warm_start)
								{
									if (constraint_island_indices_range.Valid())
									{
										ConstraintSolver::CommitStateConstraint(constraint_island_indices_range.begin, constraint_island_indices_range.Size(),
											io_physics_ctx->constraintSolver->GetLinearRowPtr(), io_physics_ctx->constraintSolver->LinearRowCount(),
											io_physics_ctx->mPhysicsWorld->mConstraintCoordinator.GetConstraints().data(), io_physics_ctx->mPhysicsWorld->mConstraintCoordinator.GetConstraints().size());
									}

									if (io_physics_ctx->enableContact && contact_constraint_island_indices_range.Valid())
									{
										ContactConstraintSolver::WriteBackImplusesManifoldCache(contact_constraint_island_indices_range.begin, contact_constraint_island_indices_range.Size(),
											io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.ContactConstraintsPtr(), io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.NumContactConstraints());
									}
								}
								//if (next_island.load(std::memory_order_relaxed) >= island_count)
								//	break;
							}
#endif // VX_SPLIT_ISLAND


						}, 0);
				}
				
				mTaskCoordinator->WaitForTasks();


				uint32 total_step_workers = mSimStep.mSimStepProfiling.writeStepTotalJacobianSolved.load(std::memory_order_relaxed);
				
				if(total_step_workers > 0)
				{
					std::sort(mSimStep.mSimStepProfiling.total_jacobian_solved,
						mSimStep.mSimStepProfiling.total_jacobian_solved + total_step_workers, std::greater<uint32>());

					const uint32 max_step_rows = mSimStep.mSimStepProfiling.total_jacobian_solved[0];

					uint32 total_step_rows = 0;
					for (uint32 i = 0; i < total_step_workers; ++i)
					{
						const uint32 rows = mSimStep.mSimStepProfiling.total_jacobian_solved[i];

						total_step_rows += rows;

						mVelocitySolveProfile.total_jacobian_solved[i] += rows;
						mVelocitySolveProfile.contributionSampleCount[i]++;

						/// reset
						mSimStep.mSimStepProfiling.total_jacobian_solved[i] = 0;

						VX_ASSERT(max_step_rows >= rows);
					}

					mVelocitySolveProfile.sampleCount++;

					//const uint32 max_step_rows = mSimStep.mSimStepProfiling.total_jacobian_solved[0];

					const double step_efficiency =
						(max_step_rows > 0) ?
						double(total_step_rows) / (double(mTaskCoordinator->MaxConcurrency()) * double(max_step_rows)) :
						0.0;


					const double step_efficiency_workers =
						(max_step_rows > 0) ?
						double(total_step_rows) / (double(total_step_workers) * double(max_step_rows)) :
						0.0;


					mVelocitySolveProfile.totalLoadBalanceEff += step_efficiency;
				}
				mSimStep.mSimStepProfiling.writeStepTotalJacobianSolved.store(0, std::memory_order_relaxed);


				//const uint32 max_currency = mTaskCoordinator->MaxConcurrency();
				//const uint32 worker_count = VxMin(total_step_workers, max_currency);
				//if (worker_count > 0)
				//{
				//	uint64 total_rows = 0;

				//	uint32 max_rows = 0;
				//	uint32 min_rows = UINT32_MAX;

				//	for (uint32 i = 0; i < worker_count; ++i)
				//	{
				//		const uint32 rows = mSimStep.mSimStepProfiling.total_jacobian_solved[i];

				//		total_rows += rows;
				//		max_rows = VxMax(max_rows, rows);
				//		min_rows = VxMin(min_rows, rows);
				//	}

				//	const double mean_rows =
				//		static_cast<double>(total_rows) / static_cast<double>(worker_count);

				//	const double efficiency =
				//		(mean_rows > 0.0) ?
				//		static_cast<double>(max_rows - min_rows) / mean_rows :
				//		0.0;
				//}


				if(mSettings->splitLargeIsland)
					mIslandCoordinator->GetSplitter().ReleaseMemAllocation(mScratchAllocator, mBodyManager.NumActiveBodies(), mIslandCoordinator->IslandCount());
#else

				///later use sorted island to solver bigger island first
				for (uint32 island = 0; island < mIslandCoordinator->IslandCount(); ++island)
				{
					IslandCoordinator::IslandRange<uint32> constraint_island_indices_range = mIslandCoordinator->IslandNonContactConstraintRowIndicesRange(island);
					IslandCoordinator::IslandRange<uint32> contact_constraint_island_indices_range = mIslandCoordinator->ContactConstraintIndicesIslandRange(island);
					
					/// perform warm starts
					if (warm_start)
					{
						if (constraint_island_indices_range.Valid())
							ConstraintSolver::WarmStart(constraint_island_indices_range.begin, constraint_island_indices_range.Size(),
								io_physics_ctx->constraintSolver->GetLinearRowPtr(), io_physics_ctx->constraintSolver->LinearRowCount(),
								solver_bodies, uint32(io_physics_ctx->constraintSolver->GetBodiesCount()));

						//fix this bad nested if branches
						if (io_physics_ctx->enableContact && contact_constraint_island_indices_range.Valid())
							ContactConstraintSolver::WarmStart(contact_constraint_island_indices_range.begin, contact_constraint_island_indices_range.Size(),
								io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.ContactConstraintsPtr(), io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.NumContactConstraints(),
								solver_bodies, uint32(io_physics_ctx->constraintSolver->GetBodiesCount()));
					}

					for (int i = 0; i < mSettings.solver.velocityIterations; ++i)
					{
						///solve non constraint first
						if (constraint_island_indices_range.Valid())
							mConstraintSolver->SolverVelocityLinear1DRowsIndices(constraint_island_indices_range.begin, constraint_island_indices_range.Size());


						if (mSettings.solver.enableContact && contact_constraint_island_indices_range.Valid())
							mContactConstraintSolver.SolveVelocityConstraint(contact_constraint_island_indices_range.begin, contact_constraint_island_indices_range.Size(), solver_bodies);
					}

					/// commit state 
					if (io_physics_ctx->mPhysicsWorld->mSettings->solver.warmstart)
					{
						if (constraint_island_indices_range.Valid())
						{
							ConstraintSolver::CommitStateConstraint(constraint_island_indices_range.begin, constraint_island_indices_range.Size(),
								io_physics_ctx->constraintSolver->GetLinearRowPtr(), io_physics_ctx->constraintSolver->LinearRowCount(),
								io_physics_ctx->mPhysicsWorld->mConstraintCoordinator.GetConstraints().data(), io_physics_ctx->mPhysicsWorld->mConstraintCoordinator.GetConstraints().size());
						}

						if (io_physics_ctx->enableContact && contact_constraint_island_indices_range.Valid())
						{
							ContactConstraintSolver::WriteBackImplusesManifoldCache(contact_constraint_island_indices_range.begin, contact_constraint_island_indices_range.Size(),
								io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.ContactConstraintsPtr(), io_physics_ctx->mPhysicsWorld->mContactConstraintSolver.NumContactConstraints());
						}
					}
				}


#endif // TEST_SOLVER_CONSTRAINT_MT

#else

			Linear1DRow* constraint_solver_rows = mConstraintSolver->GetLinearRowPtr();
			uint32 constraint_solver_row_count = mConstraintSolver->LinearRowCount();
			SolverBody* solver_bodies = mConstraintSolver->GetBodiesPtr();
			
			/// perform warm starts
			if (mSettings->solver.warmstart)
			{
				ConstraintSolver::WarmStart(constraint_solver_rows, 0, constraint_solver_row_count, solver_bodies);
				//fix this bad nested if branches
				if (mSettings->solver.enableContact)
					ContactConstraintSolver::WarmStart(mContactConstraintSolver.ContactConstraintsPtr(), mContactConstraintSolver.NumContactConstraints(), solver_bodies);
			}

			for (int i = 0; i < mSettings.solver.velocityIterations; ++i)
			{
				ConstraintSolver::SolverVelocityLinear1DRows(constraint_solver_rows, 0, constraint_solver_row_count, solver_bodies);
				//fix this bad nested if branches
				if (mSettings.solver.enableContact)
					mContactConstraintSolver.SolveVelocityConstraint(solver_bodies);
			}

			//commit solver state to constraint
			if (mSettings->solver.warmstart)
			{
				mConstraintSolver->CommitStateConstraint(mConstraintCoordinator.GetConstraints().data(), mConstraintCoordinator.GetConstraints().size());

				//commit contact constraint state to constraint
				ContactConstraintSolver::WriteBackImplusesManifoldCache(
					mContactConstraintSolver.ContactConstraintsPtr(), mContactConstraintSolver.NumContactConstraints());
			}

#endif // SOLVER_CONSTRAINT_VIA_ISLAND

			} /// Solving velocity constraints


			//write back bodies 
			ConstraintSolver::WriteBackBodies(mConstraintSolver->GetBodiesPtr(), mConstraintSolver->GetBodiesCount(), mBodyManager);

#else
			mContactConstraintSolver.SolveVelocityConstraint(mSettings.solver);
			
			mConstraintSolver->SolverAll(mContext, mSettings.solver.velocityIterations, mConstraintCoordinator.GetConstraints().data(), mConstraintCoordinator.GetConstraints().size());
#endif // CONTACT_USE_SOLVERBODY



		}

#if USE_ISLAND_COORD
		UpdateBodiesIslandActivationState(dt);
#else
		mBodyManager.UpdateBodiesActiveState(dt, mSettings.sleeping); //prevent extra time update
#endif // USE_ISLAND_COORD




		{
			VX_PROFILE_SCOPE("Integrate bodies velocities and clear accumulated force");

			//uint32 num_active_bodies = NumActiveBodies();
			//BodyID* active_bodies = ActiveBodies();

			//for (int i = 0; i < num_active_bodies; ++i)
			//{
			//	Body& body = mBodyManager.GetBody(active_bodies[i]);
			//	body.IntegrateVelocity(dt);

			//	//simulation debugger
			//	mBodyManager.UpdateBodyVelocitySimStat(body);
			//}

			uint32 num_velocity_interreation_tasks = VxMin(int((NumActiveBodies() + (SimStep::kVelocityIntergrationTaskBatch - 1)) / SimStep::kVelocityIntergrationTaskBatch), int(mTaskCoordinator->MaxConcurrency()));

			std::atomic<uint32> next_active_body_idx = 0;
			for (uint32 i = 0; i < num_velocity_interreation_tasks; ++i)
			{
				mTaskCoordinator->ConstructTask([step_ctx = &mContext, &next_active_body_idx]()
					{
						uint32 num_active_bodies = step_ctx->bodyManager->NumActiveBodies();
						BodyID* active_bodies = step_ctx->bodyManager->ActiveBodies();

						/// atomically fetch batch and process
						for (;;)
						{
							const uint32 active_body_begin = next_active_body_idx.fetch_add(SimStep::kVelocityIntergrationTaskBatch);

							/// at the end of broadphase pair
							if (active_body_begin >= num_active_bodies)
								break;

							uint32 active_body_end = VxMin(num_active_bodies, active_body_begin + SimStep::kVelocityIntergrationTaskBatch);

							for (uint32 i = active_body_begin; i < active_body_end; ++i)
							{
								Body& body = step_ctx->bodyManager->GetBody(active_bodies[i]);
								body.IntegrateVelocity(step_ctx->mDeltaTime);

								//simulation debugger
								step_ctx->bodyManager->UpdateBodyVelocitySimStat(body);
							}
						}
					}, 0);
			}

			mTaskCoordinator->WaitForTasks();
		}

		if (mSettings->solver.enable)
		{
			VX_PROFILE_SCOPE("Resolve Position Correction");
			////might need a mapper from jocobian linear row to constraints 
			/// as a hack for now 
			/// because position correction is on the constraint itself
			/// 
			/// could get linear row then solve the ones with mapping to a constraint; but 
			/// the caveat is that a constraint might have rows overlapping two batches
			/// 
			/// collect constraints during velocity solving; but could still fall into the same issue
			/// 
			/// a way to oversome inssue is to use first linear row to determine not the whole
			/// so a buffer ewhen creating linear row 
			/// mLinearRowToConstraint[linear_row_count] = constraint_idx;
			/// linear_row_count += constraint_row_count. 
			/// the issue with this is it uses more memory 
			/// 
			float baumgarte = mSettings->solver.baumgarte;
//			Constraint** solve_constraint_position = mConstraintSolver->GetConstraintResolvePositionQueuePtr();
//			uint32 num_position_constraint = mConstraintSolver->ConstraintResolvePositionQueueCount();
//			for (int i = 0; i < mSettings->solver.positionIterations; ++i)
//			{
//				ConstraintSolver::SolveConstraintsPosition(solve_constraint_position, num_position_constraint, dt, baumgarte);
//
//#if CONTACT_USE_SOLVERBODY
//				//fix this bad nested if branches
//				if (mSettings->solver.enableContact)
//				{
//					mContactConstraintSolver.SolvePositionCorrections(
//						mConstraintSolver->GetBodiesPtr(), mBodyManager,
//						mSettings->solver.baumgarte, mSettings->solver.positionCorrectionSlop,
//						mSettings->solver.positionCorrectionGlobalLimits[0], mSettings->solver.positionCorrectionGlobalLimits[1],
//						mSettings->solver.positionCorrectionBodyLimitScale);
//				}
//#endif // CONTACT_USE_SOLVERBODY
//			}

			for (uint32 i = 0; i < mTaskCoordinator->MaxConcurrency(); ++i)
			{
				mTaskCoordinator->ConstructTask([physics_ctx = &mContext, sim_step = &mSimStep]()
					{


						const uint32 total_constraint = physics_ctx->mPhysicsWorld->mConstraintCoordinator.ConstraintCount();

						float slop = physics_ctx->mPhysicsWorld->mSettings->solver.positionCorrectionSlop;
						float baumgarte = physics_ctx->mPhysicsWorld->mSettings->solver.baumgarte;
						float min_limit = physics_ctx->mPhysicsWorld->mSettings->solver.positionCorrectionGlobalLimits[0];
						float max_limit = physics_ctx->mPhysicsWorld->mSettings->solver.positionCorrectionGlobalLimits[1];
						float limit_scale = physics_ctx->mPhysicsWorld->mSettings->solver.positionCorrectionBodyLimitScale;

						SolverBody* solver_bodies = physics_ctx->constraintSolver->GetBodiesPtr();

						const uint32 position_iterations = physics_ctx->mPhysicsWorld->mSettings->solver.positionIterations;
						const bool enable_contact = physics_ctx->mPhysicsWorld->mSettings->solver.enableContact;

						for (;;)
						{
							const uint32 island_sort_idx = sim_step->solvePositionNextIslandSortedIdx.fetch_add(1, std::memory_order_relaxed);

							if (island_sort_idx >= physics_ctx->mIslandCoordinator->IslandCount())
								break;

							/// non contact constraint 
							const uint32 _island_idx = physics_ctx->mIslandCoordinator->SortedIslandIndices()[island_sort_idx];
							IslandCoordinator::IslandRange<uint32> constraint_island_indices_range = physics_ctx->mIslandCoordinator->IslandNonContactConstraintRowIndicesRange(_island_idx);

							/// contact constraint
							IslandCoordinator::IslandRange<uint32> contact_island_indices_range = physics_ctx->mIslandCoordinator->ContactConstraintIndicesIslandRange(_island_idx);


							for (int j = 0; j < position_iterations; ++j)
							{

								/// non contact constraint 
								if (constraint_island_indices_range.Valid() && constraint_island_indices_range.Size() > 0)
								{
									const uint32* global_row_idx = constraint_island_indices_range.begin;
									do
									{
										const Linear1DRow& row = physics_ctx->constraintSolver->GetLinearRowPtr()[(*global_row_idx)];

										const auto& row_info = row.info;
										if (row_info.NeedPositionCorrection())
										{
											VX_ASSERT(row_info.ConstraintIndex() < total_constraint);
											VX_ASSERT(row_info.RowLocalIndex() == 0); //

											physics_ctx->mPhysicsWorld->mConstraintCoordinator.GetConstraints()[row_info.ConstraintIndex()]->SolvePositionConstraint(physics_ctx->mDeltaTime, baumgarte);

											global_row_idx += row_info.RowCount();
											continue;
										}

										global_row_idx++;
									} while (global_row_idx < constraint_island_indices_range.end);
								}



								/// contact constraint
								if (enable_contact && contact_island_indices_range.Valid())
								{

									for (const uint32* contact_idx = contact_island_indices_range.begin;
										contact_idx < contact_island_indices_range.end; ++contact_idx)
									{

										ContactConstraintSolver::ContactConstraint* constraint = physics_ctx->mPhysicsWorld->mContactConstraintSolver.GetContactConstraint(*contact_idx);

										SolverBodyIndex& body0 = constraint->BodyA();
										SolverBodyIndex& body1 = constraint->BodyB();

										if (!body0.Value() && !body1.Value())
										{
											VX_LOG_WARN("either bodies needs to be valid");
											continue;
										}

										SolverBody& sbA = solver_bodies[body0.Value()];
										SolverBody& sbB = solver_bodies[body1.Value()];


										Body* a = &physics_ctx->bodyManager->GetBody(sbA.bodyID);
										Body* b = &physics_ctx->bodyManager->GetBody(sbB.bodyID);

										//effective mass 
										float total_inv_mass = sbA.invMass + sbB.invMass;
										ContactConstraintSolver::SolvePositionCorrection(*constraint, a, b, total_inv_mass, baumgarte, slop, min_limit, max_limit, limit_scale);
									}
								}

							}
						}
					}, 0);
			}
			mTaskCoordinator->WaitForTasks();
#if !CONTACT_USE_SOLVERBODY
			mContactConstraintSolver.SolvePositionConstraint(mSettings.solver);
#endif // !CONTACT_USE_SOLVERBODY
		}

		mConstraintSolver->ReleaseAllocation(mScratchAllocator);

		mContactConstraintSolver.FinaliseStepManifoldCache(mBodyManager);

		mContext.mStepIndex++;
		PhysicsWorld::mStepIndex = static_cast<uint32>(mContext.mStepIndex);
	}


	template<EShapeType Type>
	inline void PhysicsWorld::OnDrawBody(const Body& body, Renderer* draw_renderer, const RenderSettings& settings, const Colour& c)
	{
		///most shapes are 1:2 physics size : rendering size on all axes
		/// with capsule as exception (1:1:1) : (2:1:2)

		
		Mat44 M = Mat44::RotationTranslation(body.Orientation(), body.Position());
		
		if constexpr (Type == EShapeType::Sphere)
		{
			ERenderInstanceFlags flags = settings.sphereInstanceFlags;
			Vec3 scale = body.GetShape()->HalfExtents() * 2.0f;
			M = M.MultiplyAffine(Mat44::Scale(scale));

			draw_renderer->SubmitSpherePrimitive(
				{nullptr, M, mSettings->drawSettings->drawBodiesAsSolid ,
				true, c,  (int(flags & ERenderInstanceFlags::UseTexture) == 0)}, flags);
		}
		else if constexpr (Type == EShapeType::Box)
		{
			ERenderInstanceFlags flags = settings.boxInstanceFlags;
			Vec3 scale = body.GetShape()->HalfExtents() * 2.0f;
			M = M.MultiplyAffine(Mat44::Scale(scale));

			draw_renderer->SubmitCubePrimitive(
				{ nullptr,
				M,
				mSettings->drawSettings->drawBodiesAsSolid , true,
				c, (int(flags & ERenderInstanceFlags::UseTexture) == 0) }, flags);

			//draw_renderer->DrawText3D("Mass", body.Position() + scale, settings.textScale, vx::Colour::sWhite, settings.textAlignment);
		}
		else if constexpr (Type == EShapeType::Capsule)
		{
			///most shapes are 1:2 physics size : rendering size on all axes
			/// with capsule as exception (1:1:1) : (2:1:2)
			
			ERenderInstanceFlags flags = settings.capsuleInstanceFlags;
			Vec3 scale = body.GetShape()->HalfExtents() * Vec3(2.0f, 1.0f, 2.0f);
			M = M.MultiplyAffine(Mat44::Scale(scale));

			draw_renderer->SubmitCapsulePrimitive(
				{ nullptr,M, 
				mSettings->drawSettings->drawBodiesAsSolid , true,
				c,  (int(flags & ERenderInstanceFlags::UseTexture) == 0)}, flags);

			//draw_renderer->DrawText3D("Mass", body.Position() + scale, settings.textScale, vx::Colour::sWhite, settings.textAlignment);
		}
		else if constexpr (Type == EShapeType::Plane)
		{
			ERenderInstanceFlags flags = settings.planeInstanceFlags;
			Vec3 scale = body.GetShape()->HalfExtents() * 2.0f;
			M = M.MultiplyAffine(Mat44::Scale(scale));

			draw_renderer->SubmitQuadXZPrimitive(
				{ nullptr,
				M,
				mSettings->drawSettings->drawBodiesAsSolid , false/*true*/,
				c,  (int(flags & ERenderInstanceFlags::UseTexture) == 0) }, flags);

			//draw_renderer->DrawText3D("Mass", body.Position() + scale, settings.textScale, vx::Colour::sWhite, settings.textAlignment);
		}



	}
	void PhysicsWorld::OnDrawBodies(Renderer* draw_renderer, const RenderSettings& settings)
	{
		VX_PROFILE_FUNCTION();
		VX_ASSERT(draw_renderer, "Draw Renderer is null");

		mContactConstraintSolver.OnDraw(draw_renderer, *mSettings->drawSettings);

		const bool draw_bodies_with_tex = false;

		using Draw_Func = void(PhysicsWorld::*)(const Body&, Renderer*, const RenderSettings&, const Colour&);
		static const Draw_Func draw_table[4] =
		{
			&PhysicsWorld::OnDrawBody<EShapeType::Sphere>,
			&PhysicsWorld::OnDrawBody<EShapeType::Box>,
			&PhysicsWorld::OnDrawBody<EShapeType::Capsule>,
			&PhysicsWorld::OnDrawBody<EShapeType::Plane>
		};


		/// move later ShapeType
		static const Colour shape_col_type[4] =
		{
			 Colour::sCyan, //sphere
			 Colour::sYellow, //box
			 Colour(0.4f, 1.0f, 0.4f), // capsule light green
			 Colour(0.5f), //plane
		};


		const DrawSettings& draw_settings = *mSettings->drawSettings;
		//return;
		for (auto it = Bodies().begin(); it != Bodies().end(); ++it)
		{
			if (!it->ID().IsValid())
				continue;

			/// support to colour bodies by 
			/// 1. State: Dynamic(Awake/Sleeping), Static
			/// 2. Collision: Colliding(Dyn-Dyn/Dyn-Static) , Not Colliding
			/// 3. Phases(or final stage): No detect(not in broad), in narrow(pair but fail), Colliding(Dyn-Dyn/Dyn-Static) , Not Colliding
			/// 4. Advance later when support island

			Colour c = Colour::sMagenta;


			switch (draw_settings.bodyColourMode)
			{
			case vx::EBodyColourMode::Instances:
				c = Colour::RandomColour(it->ID().ID());
				break;
			case vx::EBodyColourMode::MotionType:
				c = it->IsDynamic() ? draw_settings.dynamicColour : draw_settings.staticColour;
				break;
			case vx::EBodyColourMode::MotionState:
				c = it->IsDynamic() ?
					(it->IsSleeping() ? draw_settings.sleepingColour : draw_settings.dynamicColour) :
					draw_settings.staticColour;
				break;
			case vx::EBodyColourMode::ShapeType:
				c = shape_col_type[(int)it->GetShape()->Type()];
				break;
			case vx::EBodyColourMode::Phase:
				c = BodySimphaseDebugColour(*it);
				break;
			case vx::EBodyColourMode::MotionMaxLinearVelocity:
			case vx::EBodyColourMode::MotionMaxAngularVelocity:
				{
					float t;
					if (draw_settings.bodyColourMode == vx::EBodyColourMode::MotionMaxLinearVelocity)
					{
						float speed_sq = it->LinearVelocity().LengthSq();
						t = speed_sq / it->MaxLinearVelocity();
					}
					else
					{
						float speed_sq = it->AngularVelocity().LengthSq();
						t = speed_sq / it->MaxAngularVelocity();
					}
					if (it->IsDynamic())
					{
						Colour min_col = Colour::sCyan;
						Colour max_col = Colour::sRed;

						t = VxClamp01(t);

						auto Lerp_Col = [](Colour col_a, Colour col_b, float t)
							{
								float minus_t = 1.0f - t;
								float r = (minus_t * col_a.R()) + t * col_b.R();
								float g = (minus_t * col_a.G()) + t * col_b.G();
								float b = (minus_t * col_a.B()) + t * col_b.B();
								float a = (minus_t * col_a.A()) + t * col_b.A();

								return Colour(r, g, b, a);
							};

						c = Lerp_Col(min_col, max_col, t);
					}
					else
						c = draw_settings.staticColour;
						break;
				}
			case vx::EBodyColourMode::IslandIdx:
				{
					uint32 island_idx = it->GetIslandIndex();
					if (it->IsStatic())
						c = Colour(0.5f);
					else if (island_idx == Body::kInvalidIslandIdx)
						c = draw_settings.sleepingColour;
					else
						c = Colour::RandomColour(island_idx);
					break;
				}
			case vx::EBodyColourMode::IslandConstraintGroup:
				{
					uint32 island_idx = it->GetIslandIndex();
					if (it->IsStatic())
						c = Colour(0.5f);
					else if (island_idx == Body::kInvalidIslandIdx)
						c = draw_settings.sleepingColour;
					else
					{
						uint32 island_constraint_grp = it->mIslandConstraintGroupMask;


						Vec3 accumulated_colour = Vec3(0.0f);
						uint32 count = 0;
						for (uint32 i = 0; i < 16; ++i)
						{
							//if (Bit32(island_constraint_grp) & Bit32(i))
							if (island_constraint_grp & Bit32(i))
							{
								accumulated_colour += Colour::RandomColour(i);
								count++;
								//break;
							}
						}

						if (count > 0)
							accumulated_colour /= count;

						c = vx::Colour(accumulated_colour);
					}
					break;
				}
			default:
				VX_LOG_WARN("Unknow bodyColourMode");
				break;
			}



			c.SetAlpha(draw_settings.bodiesDrawColourAlpha);
			//int shape_enum_idx = (int)(it->GetShape()->Type());
			///// a quick hack to get sphere shape to render with tex 
			///// XOR, since Sphere is 0, so normalise
			//bool has_tex = draw_bodies_with_tex ^ (shape_enum_idx == 0);
			(this->*draw_table[(int)(it->GetShape()->Type())])(*it, draw_renderer, settings, c);



			if (draw_settings.drawBodiesMassText)
			{
				//StackString<8> test = ToStackString<>("%.2f", 2.12345f);
				float inv_mass = it->InverseMass();
				StackString<32> mass_text;
				mass_text << "Mass: " << ToStackString("%.2f", (inv_mass) ? (1.0f / inv_mass) : 0.0f).Data() << "kg";// << ToStackString<>("%.2f", 2.12345f).Data();
				//std::string mass_text = "Mass: " + std::to_string((inv_mass) ? (1.0f / inv_mass) : 0.0f) + "kg";
				//mass_text = std::to_string(0.0f);
				Vec3 he = it->GetShape()->HalfExtents();
				he = Vec3::Zero();
				(!settings.useTestDynamicScale) ? draw_renderer->DrawText3D(mass_text.Data(), it->Position() + he, settings.textScale, vx::Colour::sOrange, settings.textAlignment)
					: draw_renderer->DrawText3D_DynScale(mass_text.Data(), it->Position() + he, settings.textScale, vx::Colour::sOrange, settings.textAlignment);
			}
		}
	}

	void PhysicsWorld::OnDebugDraw(DebugGizmosRenderer* debug_renderer)
	{
		VX_PROFILE_FUNCTION();
		VX_ASSERT(debug_renderer, "Debug Renderer is null");

		if (mSettings->drawSettings->nonContactConstraintDrawSettings.drawConstraints)
			mConstraintCoordinator.DebugGizmos(debug_renderer, mSettings->drawSettings->nonContactConstraintDrawSettings);

		//debug_renderer->DrawLine(mExperimentRay.origin, mExperimentRay.End(), Colour::sGreen);

		const DrawSettings& draw_settings = *mSettings->drawSettings;
		if(mBroadphase)
			mBroadphase->DebugDraw(debug_renderer, draw_settings);


		mContactConstraintSolver.DebugDraw(debug_renderer, mConstraintSolver, &mBodyManager, draw_settings);

		if(mSettings->drawSettings->drawDebugInertia)
			QuickDebugDrawInertia(debug_renderer);
		//////DEBUG BOX BOX SAT 
		if(draw_settings.drawAABBContactManifoldInFrame)
		{
			for (const auto& box_box_sat : sBoxVsBoxSATDebugInstances)
			{
				//sample location 
				Vec3 c = vx::Vec3::LoadFloat3Raw(box_box_sat.manifoldCenter);
				//debug_renderer->DrawWireSphereDiscs(c, 0.1, vx::Colour::sMagenta);


				//Draw best axis 
				Vec3 best_axis = Vec3::LoadFloat3Raw(box_box_sat.best_axis);
				debug_renderer->DrawLine(c - best_axis * 2, c + best_axis * 2, vx::Colour::sGreen);

				vx::Vec2 plane_half_extent = box_box_sat.planeSize;
				//vx::Vec3 size_scaled_t0 = vx::Vec3::LoadFloat3Raw(box_box_sat.planeTangent) * plane_half_extent[0] * 2;
				//vx::Vec3 size_scaled_t1 = vx::Vec3::LoadFloat3Raw(box_box_sat.planeBiTangent) * plane_half_extent[1] * 2;


				//// DRAW PLANE
				if(draw_settings.drawAABBContactManifoldInFrameWcPlane)
				{
					///alright let create a proper tangetnt bitange to the normal
					Vec3 contactN = vx::Vec3::LoadFloat3Raw(box_box_sat.contactNormal);
					Vec3 tangentN = contactN.NormalisedPerpendicular();
					Vec3 bitangentN = contactN.Cross(tangentN);
					//ensure orthogonality
					tangentN = bitangentN.Cross(contactN);
					vx::Vec3 size_scaled_t0 = tangentN * plane_half_extent[0] * 2;
					vx::Vec3 size_scaled_t1 = bitangentN * plane_half_extent[1] * 2;

					//time to draw contact plane
					vx::Vec3 vert0 = c + size_scaled_t0 + size_scaled_t1;
					vx::Vec3 vert1 = c + size_scaled_t0 - size_scaled_t1;
					vx::Vec3 vert2 = c - size_scaled_t0 - size_scaled_t1;
					vx::Vec3 vert3 = c - size_scaled_t0 + size_scaled_t1;

					//draw triangles
					/// plane colour changes base on face contact / edge to edge
					Colour plane_col =
						(box_box_sat.best_axis_type == 2) ? Colour::sBlue :
						(box_box_sat.best_axis_type == 3) ? Colour::sDeepTeal : Colour::sCyan;
					debug_renderer->DrawSolidWireTriangle(vert0, vert2, vert1, plane_col);
					debug_renderer->DrawSolidWireTriangle(vert0, vert3, vert2, plane_col);
				}

				//draw ref'ing body 
				BoxShape* ref = box_box_sat.ref;
				if (ref)
				{
					VX_ASSERT_WARN(false, "Trying to use debug info that as incorrect");
					debug_renderer->DrawAABB(ref->ComputeWorldBounds(Mat44::Identity(), vx::Vec3::One()), vx::Colour::sOrange);
				}


				//draw contact normal 
				debug_renderer->DrawArrowCone(c, c + vx::Vec3::LoadFloat3Raw(box_box_sat.contactNormal) * 2, 0.05, 0.1, 0.05, 3, vx::Colour::sYellow);


				///draw all generated points
				for (int i = 0; i < box_box_sat.generatedPt; ++i)
					debug_renderer->DrawWireSphereDiscs(vx::Vec3::LoadFloat3Raw(box_box_sat.pointsGenerated[i]), 0.1f, vx::Colour::sRed);
				////draw contact point on a
				//debug_renderer->DrawWireDisc


				if (box_box_sat.isEdgeDetection)
				{
					vx::Vec3 dir;
					{
						vx::Vec3 eA0 = vx::Vec3::LoadFloat3Raw(box_box_sat.edgeA0);
						vx::Vec3 eA1 = vx::Vec3::LoadFloat3Raw(box_box_sat.edgeA1);

						dir = (eA1 - eA0).Normalised();
						debug_renderer->DrawArrowCone(eA0, eA0 + dir * 2.0f, 0.05, 0.1, 0.05, 3, vx::Colour::sTurquoise);
					}



					vx::Vec3 eB0 = vx::Vec3::LoadFloat3Raw(box_box_sat.edgeB0);
					vx::Vec3 eB1 = vx::Vec3::LoadFloat3Raw(box_box_sat.edgeB1);

					dir = (eB1 - eB0).Normalised();
					debug_renderer->DrawArrowCone(eB0, eB0 + dir * 2.0f, 0.05, 0.1, 0.05, 3, vx::Colour::sDeepTeal);


					debug_renderer->DrawWireSphereDiscs(vx::Vec3::LoadFloat3Raw(box_box_sat.edgeCenter0), 0.1, Colour::sYellow);
					debug_renderer->DrawWireSphereDiscs(vx::Vec3::LoadFloat3Raw(box_box_sat.edgeCenter1), 0.1, Colour::sYellow);
					
				}

			}
		}



		/// Draw bodies properties
		if (draw_settings.drawAABB || draw_settings.drawOBB ||
			draw_settings.drawBodiesPrincipalAxes ||
			draw_settings.drawShapeOrientedBoundCorners || draw_settings.drawBodiesVelocities)
		{
			for (const auto& body : Bodies())
			{
				if (!body.IsIDValid())
					continue;

				Colour c = ((mBodyManager.GetBodySimStats(body).phase & EBodySimphaseFlags::IsColliding) == EBodySimphaseFlags::IsColliding) ?
						draw_settings.contactWireColour : draw_settings.shapeColliderWireColour;
				if (draw_settings.drawAABB)
				{
					AABB bounds = body.GetAABBWorld();
					debug_renderer->DrawAABB(bounds.mMin, bounds.mMax, c);
				}
				if (draw_settings.drawOBB)
				{
					OBB obb = OBB(body.GetShape()->LocalBounds(), body.Orientation());
					debug_renderer->DrawBox(obb.ComputeCorners(body.Position()), c);
				}


				if (draw_settings.drawBodiesPrincipalAxes)
				{

					Vec3 start = body.Position();


					Vec3 half_extent = body.GetShape()->HalfExtents();
					if (body.GetShape()->Type() == EShapeType::Plane) half_extent.SetY(0.0f);

					Vec3 x_axis = body.Orientation().Rotate(Vec3(1.0f, 0.0f, 0.0f));
					debug_renderer->DrawLine(start,
						(start + x_axis * half_extent.X()),
						Colour(1.0f, 0.0f, 0.0f));

					Vec3 y_axis = body.Orientation().Rotate(Vec3(0.0f, 1.0f, 0.0f));
					debug_renderer->DrawLine(start,
						(start + y_axis * half_extent.Y()),
						Colour(0.0f, 1.0f, 0.0f));

					Vec3 z_axis = body.Orientation().Rotate(Vec3(0.0f, 0.0f, 1.0f));
					debug_renderer->DrawLine(start,
						(start + z_axis * half_extent.Z()),
						Colour(0.0f, 0.0f, 1.0f));
				}

				if (draw_settings.drawBodiesVelocities)
				{
					const Vec3 center = body.Position();

					const Vec3 lin_vel = body.LinearVelocity();
					const Vec3 ang_vel = body.AngularVelocity();

					debug_renderer->DrawArrowCone(center, center + lin_vel, 0.2f, 0.5f, 0.2f, 4, draw_settings.bodyLinearVelocityCol);
					debug_renderer->DrawArrowCone(center, center + ang_vel, 0.2f, 0.5f, 0.2f, 4, draw_settings.bodyAngularVelocityCol);

				}


				if (draw_settings.drawShapeOrientedBoundCorners)
				{
					Colour col = draw_settings.drawShapeCornersColour;
					Quat q = body.Orientation();

					OBB obb = OBB(body.GetShape()->LocalBounds(), q);

					Vec3 axis_x = q.RotateAxisX();
					Vec3 axis_y = q.RotateAxisY();
					Vec3 axis_z = q.RotateAxisZ();
					Vec3 center = body.Position();

					auto& corners = obb.ComputeCorners(center);
					///debug_renderer->DrawBox(corners, Colour::sMagenta);

					for (const auto& c : corners)
					{
						debug_renderer->DrawLine( 
							(c - Vec3(draw_settings.drawContactPointSize, 0.0f, 0.0f)),
							(c + Vec3(draw_settings.drawContactPointSize, 0.0f, 0.0f)),
							col);
						debug_renderer->DrawLine(
							(c - Vec3(0.0f, draw_settings.drawContactPointSize, 0.0f)),
							(c + Vec3(0.0f, draw_settings.drawContactPointSize, 0.0f)),
							col);

					}
				}

			}
		}


		

	}

	void PhysicsWorld::QuickDebugDrawInertia(DebugGizmosRenderer* debug_renderer)
	{
		VX_PROFILE_FUNCTION();
		for(const auto& body : mBodyManager.GetBodies())
		{
			if (body.InverseMass() <= 0.0f)
				continue;

			float mass = 1.0f / body.InverseMass();
			Vec3 com = body.Position();
			Quat q = body.Orientation();

			Vec3  rt = q * Vec3::Right();
			Vec3  up = q * Vec3::Up();
			Vec3  fwd = q * Vec3::Forward();

			Vec3 I_local = body.GetLocalInvInertiaDiagonal().Reciprocal();

			float axis_len = 1.0f;
			//principal axes
			debug_renderer->DrawLine(com, com + rt * axis_len, Colour::sRed);
			debug_renderer->DrawLine(com, com + up * axis_len, Colour::sGreen);
			debug_renderer->DrawLine(com, com + fwd * axis_len, Colour::sBlue);


			///equivalent inertia box
			float w = VxSqrt(VxMax(0.0f, 6.0f * (I_local.Y() + I_local.Z() - I_local.X()) / mass));
			float h = VxSqrt(VxMax(0.0f, 6.0f * (I_local.X() + I_local.Z() - I_local.Y()) / mass));
			float d = VxSqrt(VxMax(0.0f, 6.0f * (I_local.X() + I_local.Y() - I_local.Z()) / mass));

			float h_w = w * 0.5f;
			float h_h = h * 0.5f;
			float h_d = d * 0.5f;

			Vec3 diag = I_local * (12.0f / mass);
			Vec3 _size = Vec3(VxSqrt(0.5f * (-diag[0] + diag[1] + diag[2])),
				VxSqrt(0.5f * (diag[0] - diag[1] + diag[2])),
				VxSqrt(0.5f * (diag[0] + diag[1] - diag[2])));

			OBB obb = OBB(AABB(-0.5f * _size, 0.5f * _size), q);
			//debug_renderer->DrawBox(aabb.GetOBB(Mat44::RotationTranslation(q, com)), Colour::sYellow);
			debug_renderer->DrawBox(obb.ComputeCorners(com), mSettings->drawSettings->drawDebugInertiaColour);


			////momentum
			//	body.GetPointVelocityRelCOM
			Vec3 ang_vel = body.AngularVelocity();
			Vec3 local_omega(ang_vel.Dot(rt), ang_vel.Dot(up), ang_vel.Dot(fwd));
			Vec3 local_L(local_omega.X() * I_local.X(),
				local_omega.Y() * I_local.Y(),
				local_omega.Z() * I_local.Z());

			Vec3 world_L = (rt * local_L.X()) + (up * local_L.Y()) + (fwd * local_L.Z());
			float momentum_scale = 0.5f;
			debug_renderer->DrawLine(com, com + world_L * momentum_scale, Colour::sMagenta);
		}
	}

	const CollisionResolutionStat PhysicsWorld::NarrowphaseStats() const
	{
		return mNarrowphaseQuery.Stats();
	}



	void PhysicsWorld::UpdateBodiesIslandActivationState(float dt)
	{
		VX_PROFILE_FUNCTION();
		/// citeria to put bodies to sleep 
		/// for island to sleep all its bodies need to meet this citeria 
		/// 
		
		/// i.e if a body in island is a wake all is awake 

		///or maybe 2 loops 

		
		/// 1. check island state
		/// 2. update bodies in islands
		///
		if(mSettings->sleeping.enable)
		{
			//auto& islands = mIslandCoordinator->islands;

			std::vector<uint32> islands_to_sleep;
			std::vector<uint32> islands_awake;

			for (uint32 island = 0; island < mIslandCoordinator->IslandCount(); ++island)
			{
				IslandCoordinator::IslandRange<BodyID> body_island = mIslandCoordinator->IslandBodyIDsRange(island);

				bool put_to_sleep = true;

				for (const BodyID* body_id = body_island.begin; body_id < body_island.end; ++body_id)
				{
					Body& body = mBodyManager.GetBody(*body_id);
					//if (mSettings.sleeping.enable)

					body.UpdateSleepState(dt, mSettings->sleeping);

					if (body.IsAwake())
					{
						put_to_sleep = false;
						break;
					}
				}
				if (put_to_sleep)
					islands_to_sleep.push_back(island);
				else
					islands_awake.push_back(island);
			}

			//// haxk for bodies in the awake list force wake up 
			/// as some bodies are trying to sleep 
			for (auto& island_idx : islands_awake)
			{
				IslandCoordinator::IslandRange<BodyID> body_island = mIslandCoordinator->IslandBodyIDsRange(island_idx);
				for (const BodyID* body_id = body_island.begin; body_id < body_island.end; ++body_id)
				{
					Body& body = mBodyManager.GetBody(*body_id);
					body.WakeUp(false);
				}
			}

		}


		/// re-add bodies to active list
		mBodyManager.ResetActivateBodies();
		mBodyManager.AddBodiesToActivate(false);
	}

	Colour PhysicsWorld::BodySimphaseDebugColour(const Body& body) const
	{
		auto flags = mBodyManager.GetBodySimStats(body).phase;

		const bool is_colliding = Contains(flags, EBodySimphaseFlags::IsColliding);
		const bool touching_static = Contains(flags, EBodySimphaseFlags::IsTouchingStatic);
		const bool in_narrow = Contains(flags, EBodySimphaseFlags::InNarrowphase);
		const bool in_broad = Contains(flags, EBodySimphaseFlags::InBroadphase);

		const DrawSettings& draw_settings = *mSettings->drawSettings;
		if (is_colliding)
		{
			return Colour(1.0f, 0.5f, 0.0f);


			/// ignore dyn-sta collision
			/// later deep penetration solver 
			//if (touching_static)
			//{
			//	return body.IsDynamic() ?
			//		draw_settings.dynamicStaticCollisionColour :
			//		draw_settings.staticColour;
			//}
			//return body.IsDynamic() ?
			//	draw_settings.dynamicColour :
			//	draw_settings.staticColour;
		}


		if (in_narrow) return draw_settings.narrowPhaseColour;
		if (in_broad) return draw_settings.broadPhaseColour;
		return draw_settings.broadPhaseColour;// draw_settings.neuralPhaseColour;

	}

}