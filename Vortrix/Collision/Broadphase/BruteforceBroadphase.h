#pragma once

#include "Broadphase.h"
#include "BroadphasePair.h"

#include "Dynamics/Body/Body.h"
#include "Vortrix/Visuals/Renderers.h"

#include "Dynamics/Body/BodyManager.h"

#include "Vortrix/SimulationContexts.h"
#include "Vortrix/Core/Colours.h"

namespace vx
{
	
	// Naive Bruteforce Broadphase N squared
	// Simple O(n^2) implementation loops through every body pair
	// and checks for AABB
	class BruteforceBroadphase final : public Broadphase
	{

	public:

		void Init(BodyManager* in_body_manager, const BroadphaseInitInfo& info) override{}


		void InsertBody(Body* body) override {}
		void RemoveBody(const BodyID& id) override {}
		void ComputeCollidingPair(const PhysicsStepContext& physics_ctx, SimStep& io_sim_step) override
		{
			VX_PROFILE_FUNCTION();
			auto& bodies = physics_ctx.bodyManager->GetBodies();
			const size_t bodies_count = bodies.size();

			mFrameAABBs.clear();
			mFrameAABBs.reserve(bodies_count);

			for (size_t i = 0; i < bodies_count; ++i)
			{
				//only check dynamic can initalie collision
				//if (!bodies[i].IsDynamic()) continue;

				//bodies[i].bDebuggingContact = false;
				const Body& body_a = bodies[i];
				AABB& aAABB = mFrameAABBs.emplace_back(body_a.GetAABBWorld());
				aAABB.Grow(mBoundThreshold);


				for (size_t j = i + 1; j < bodies_count; ++j)
				{
					//bodies[j].bDebuggingContact = false;
					const Body& body_b = bodies[j];
					AABB& bAABB = mFrameAABBs.emplace_back(body_b.GetAABBWorld());
					bAABB.Grow(mBoundThreshold);

					if (aAABB.Overlaps(bAABB))
					{
						VX_ASSERT(io_sim_step.broadphasePairCount < physics_ctx.mSettings->collision.maxPairs);
						io_sim_step.broadphasePair[io_sim_step.broadphasePairCount++] = BroadphasePair(&bodies[i], &bodies[j]);
					}
				}
			}

			mLastStep = physics_ctx.mStepIndex;
		}

		void DebugDraw(DebugGizmosRenderer* debug_renderer, const DrawSettings& settings) override
		{
			VX_PROFILE_FUNCTION();
			if (!Contains(settings.braodphaseFlags, EBroadphaseDrawFlag::All))
				return;
			for (auto& aabb : mFrameAABBs)
				debug_renderer->DrawAABB(aabb.mMin,
					aabb.mMax, Colour(0.0f, 0.0f, 1.0f));
		}


	private:
		std::vector<AABB> mFrameAABBs;
	};

}