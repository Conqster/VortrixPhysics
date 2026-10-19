#pragma once

#include "Vortrix/PhysicsWorldSettings.h"

namespace vx {

	struct SimStep
	{
		/// kProcessBodyPairBatch
		/// broadphase Body pair batch count per thread processing 
		/// in narrowphase
		static constexpr uint32 kProcessBodyPairBatch = 32;
		static constexpr uint32 kAcclerationGravityTaskBatch = 64;
		static constexpr uint32 kVelocityIntergrationTaskBatch = 64;

		PhysicsStepContext* mPhysicsStepContext = nullptr;

		/// fix for debugging
		//struct BroadphaseBuffer
		//{
		//	BroadphasePair* data = nullptr;
		//	uint32 count = 0;
		//	uint32 maxPairs = 0;
		//}mBroadphaseBuffer;

		struct BroadphasePair* broadphasePair = nullptr;
		uint32 broadphasePairCount = 0;

		std::atomic<uint32> nextProcessPairIdx = 0;

		///island 
		std::atomic<uint32> solveVelocityNextIslandIdx = { 0 };

		std::atomic<uint32> solvePositionNextIslandSortedIdx = { 0 };




		struct SimulationStepProfilingData
		{
			static constexpr uint32 kMaxThreads = 20;
			std::atomic<uint32> writeStepTotalJacobianSolved{ 0 };
			uint32 total_jacobian_solved[kMaxThreads]; //<-- later max thread or hardware concurrency
		};

		SimulationStepProfilingData mSimStepProfiling;


		///probably hold pointer to active constraint (non contact)
		//Constraint** constraints
	};



	struct VelocitySolveProfile
	{
		static constexpr uint32 kMaxThreads = 20;

		uint32 total_jacobian_solved[kMaxThreads]{ 0 }; //<-- later max thread or hardware concurrency
		/// write on main thread, zero data races
		uint32 sampleCount = 0;
		uint32 contributionSampleCount[kMaxThreads] = { 0 };

		double totalLoadBalanceEff = 0;

		void ResetAccumulation()
		{
			for (uint32 i = 0; i < kMaxThreads; ++i)
			{
				total_jacobian_solved[i] = 0;
				contributionSampleCount[i] = 0;
			}

			sampleCount = 0;
			totalLoadBalanceEff = 0.0;
		}
	};



}