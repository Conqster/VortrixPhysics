#pragma once
#include "Vortrix/Core/Core.h"

#include "ConstraintSolver.h"

namespace vx {


	/// Requirements 
	///
	/// availabe parameters
	///	- BodyID/actual body
	/// - SolverBody & SolverIndex
	/// 
	/// - ContactConstraint
	/// - Constraint (Non-Contact)
	/// 
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

	class IslandCoordinator
	{
	public:
		~IslandCoordinator();

		void Init(uint32 max_bodies, uint32 max_contact_constraint, uint32 max_noncontact_constraint);

		void PrepareIslands(uint32 body_count);


		uint32 ComputeActiveBodyLowestIdx(uint32 idx);


		void LinkBodies(uint32 body_activeA, uint32 body_activeB);
		void LinkContactConstraint(uint32 constraint_idx, uint32 min_active_body_idx);

		///// using this for now 
		///// might later convert to a pipeline 
		///// where Row are not allocated during prep'ing
		///// so first loop; prep constraint (i.e active constraint) 
		///// then after building islands; jacobian rows are allocated 
		///// contigious to each other due island 
		//struct ConstraintRowEdge //data
		//{
		//	uint32 bodyLink; ///body index
		//	uint32 rowStart;
		//	uint32 rowCount;
		//};
		/// so this would held 
		/// on building island Linking
		/// it links to the lowest body sort island allocation
		/// the row start is the pointer index to the constraint row in constraint solver
		/// row count is the jacobian row for constraint 
		/// 
		/// by the end of island building constraint island indices would contain 
		/// tje constraint in order ends for constraint is determined per constraintRowEdge start + count 
		void LinkNonConstactConstraint(uint32 constraint_row_idx, uint32 row_count, uint32 min_active_body_idx);

		
		void FinaliseIslands(const ConstraintSolver& constraint_solver, uint32 contact_constraint_count,
			uint32 active_non_contact_constraint_count, BodyManager& body_manager, ScratchAllocator* scratchAllocator);


		

		uint32 IslandCount() const { return mIslandCount; }

		template<typename T>
		struct IslandRange
		{
			IslandRange(const T* _begin, const T* _end) : begin(_begin), end(_end) {}
			const T* begin; ///inclusive
			const T* end; ///inclusive

			bool Valid() const
			{
				return begin != nullptr &&
					end != nullptr;//&&
					end > begin; //
			}
			uint32 Size() const
			{
				VX_ASSERT(Valid());
				return static_cast<uint32>(end - begin);
			}
		};

		IslandRange<BodyID> IslandBodyIDsRange(uint32 island_idx) const;
		IslandRange<SolverBodyIndex> IslandSolverBodyIndicesRange(uint32 island_idx) const;
		IslandRange<uint32> ContactConstraintIndicesIslandRange(uint32 island_idx) const;
		IslandRange<uint32> IslandNonContactConstraintRowIndicesRange(uint32 island_idx) const;



		/// Raw
		/// mostly for debugging 
		const std::vector<uint32>& IslandsIndicesUnsorted() const { return mIslandIdxs; }
		const std::atomic<uint32>* ActiveBodyLinkIndices() const { return mActiveBodyLinkIndices; } /// mBodyLinkIndexs

		const BodyID* BodyIDIslands() const { return mBodyIDIslands; }
		const SolverBodyIndex* SolverBodyIndexIslands() const { return mSolverBodyIndexIslands; }

		const uint32* ContactConstraintBodyLinkIndices() const { return mContactConstraintBodyLinkIndices; }

		const uint32* SortedIslandIndices() const { return mSortedIslandIndices.data(); }

	private:
		void FinaliseBodyIslands(const ConstraintSolver& constraint_solver, BodyManager& body_manager, const uint32 active_bodies_count, uint32* io_temp_active_bodies_island_indices, ScratchAllocator* scratchAllocator);
		/// const i_temp_active_bodies_island_indices used to determine which island 
		/// constraubt fall into basecd on its body
		void FinaliseContactConstraint(const uint32 constraint_count, const uint32 island_count,
			const uint32* i_temp_active_bodies_island_indices, ScratchAllocator* scratchAllocator);
		void FinaliseNonContactConstraint(const uint32 constraint_count, const uint32 island_count,
			const uint32* i_temp_active_bodies_island_indices, ScratchAllocator* scratchAllocator);

		void SortIslands(ScratchAllocator* scratchAllocator);
	private:
		/// list would be invalid if not participanting or static 
		//uint32* mBodiesIdxs = nullptr;
		std::atomic<uint32>* mActiveBodyLinkIndices = nullptr; /// mBodyLinkIndexs
		/// this is correspond to the step 
		/// contact constraint in buffer 
		/// link to active body
		uint32* mContactConstraintBodyLinkIndices = nullptr;
		uint32 mMaxContactConstraint = 256;

		/// using this for now 
		/// might later convert to a pipeline 
		/// where Row are not allocated during prep'ing
		/// so first loop; prep constraint (i.e active constraint) 
		/// then after building islands; jacobian rows are allocated 
		/// contigious to each other due island 
		struct ConstraintRowEdge //data
		{
			uint32 bodyLink; ///body index
			uint32 rowStart;
			uint32 rowCount;
		};
		ConstraintRowEdge* mNonContactConstraintRowBodyLinkIndices = nullptr; ///later make this temporary step data; using scratch allocation
		uint32 mMaxNonContactConstraint = 256;

		std::atomic<uint32> mValidateStepMaxConstraint{ 0 };
		std::atomic<uint32> mValidateStepMaxNonConstraint{ 0 };


		//uint32* mIslandIdxs = nullptr;
		std::vector<uint32> mIslandIdxs;


		uint32 mIslandCount{ 0 };
		std::vector<uint32> mSortedIslandIndices;


		/// sorted per island data

		/// convert to solver body index
		BodyID* mBodyIDIslands = nullptr;
		SolverBodyIndex* mSolverBodyIndexIslands = nullptr;
		//Solver body index end is equal tobody id end
		//uint32* mSolverBodyIndexPerIslandIndexEnds = nullptr;
		uint32* mBodyIDPerIslandIndexEnds = nullptr;


		uint32* mConstraintIndicesIslands = nullptr;
		uint32* mConstraintIslandIndexEnds = nullptr;


		uint32* mNonConstactConstraintRowIndicesIslands = nullptr;
		uint32* mNonConstraintRowIslandIndexEnds = nullptr;

	public:
		class Splitter
		{
		public:

			static const uint32 kLargeIslandSpitThreshold = 8;//128;//64;

			using BinMask = uint32;
			static constexpr uint32 kMaxBin = 16; // toal mask bit for 4 bytes (32 bits) int
			static constexpr uint32 kBatchSize = 8;//32;//16;




			enum class EStatus
			{
				Complete,
				WaitingForBatches,
				RetrievedBatch
			};

			void Prepare(uint32 active_count, const IslandCoordinator& island_coord, ScratchAllocator* scratch_allocator);
			void ReleaseMemAllocation(ScratchAllocator* scratch_allocator, uint32 active_body_count, uint32 islands_count);




			void SplitIsland(uint32 island_idx, const IslandCoordinator& island_coord,
				const class ContactConstraintSolver* contact_coord, const class ConstraintSolver* constraint_solver,
				class BodyManager* body_manager, ScratchAllocator* scratch_allocator);




			bool IsIslandLarge(uint32 island_idx) const { return mIslandIsLarge[island_idx]; }
			EStatus NextConstactConstraintBatchRange(uint32& split_island_idx, uint32 island_count,
				IslandRange<uint32>& island_contact_range, IslandRange<uint32>& island_noncontact_range,
				int& batch_first_iteration, uint32& debug_bin, const uint32* sorted_island_indices);
			void MarkConstactConstraintBatchRangeComplete(uint32 island_idx, uint32 process_count, uint32 velocity_iteration, bool& last_iteration, uint32 debug_bin);



			class BinConstraintsOffsetRange
			{
			public:
				uint32 contactStart = 0;
				uint32 contactEnd = 0;

				uint32 nonContactStart = 0;
				uint32 nonContactEnd = 0;


				uint32 NumContactConstraint() const { return contactEnd - contactStart; }
				uint32 NumNonContactConstraint() const { return nonContactEnd - nonContactStart; }

				uint32 TotalConstraintCount() const { return NumContactConstraint() + NumNonContactConstraint(); }


				uint32 TotalBatchCount() const
				{
					uint32 v = TotalConstraintCount();
					return (v > 0) ? (TotalConstraintCount() / kBatchSize) : 0;
				}
			};





			class IslandSplitBins
			{
			public:
				uint32 NumBins() const { return mNumActiveBins; }

				BinConstraintsOffsetRange mBins[kMaxBin + 1]{};
				
				/// bin index must be less than the number of active bins 
				/// unless accessing the non parallel bin (last bin) if available
				/// 
				uint32 mNumActiveBins = 0;

				/// |a0|a1|a2|b0|b1|c0|c1|c2|c3|c4|non parallel
				///          ^     ^              ^
				/// a, b, c..... bins 
				/// ..0, ..1 batches in bin
				/// mTotalBatchProcessed spendle to progress the Bins
				/// if another thread mark this bin as processed then the total batch for bin equals bin_range.TotalConstraintCount()
				/// this is only true, if mtotalBathProcess is [0, sum bin constraint)
				/// but in occur state it is reset
				std::atomic<uint32> mTotalBatchProcessed{ 0 };
				//std::atomic<uint32> mNext{ 0 };
				//std::atomic<uint32> mCurrBin = 0;

				//std::atomic<uint32> mIterations = 0;
				uint32 mIterations = 0;

				std::atomic<bool> mComplete = false;
				std::atomic<bool> mFirstIteration = false;

				EStatus NextConstactConstraintBatchRange(
					uint32& o_contact_start, uint32& o_contact_end,
					uint32& o_noncontact_start, uint32& o_noncontact_end, int& first_iteration, uint32& debug_bin);

				void MarkConstraintBatchRangeComplete(uint32 process_constraint, uint32 velocity_iteration, bool& last_iteration, uint32 debug_bin);

				static uint64 MakeCurrBinNext(uint32 curr_bin, uint32 next)
				{
					return (uint64(curr_bin) << 32) | uint64(next);
				}

				static uint32 GetCurrBin(uint64 currbin_next)
				{
					return uint32(currbin_next >> 32);
				}

				static uint32 GetNextBatch(uint64 curr_bin_next)
				{
					return uint32(curr_bin_next);
				}
			//private:
				/// | curr bin | next batch | 

				std::atomic<uint64> mCurrBinNext{ 0 };
			};



			/// debug 
			const IslandSplitBins* IslandsSplitBins() const { return mIslandSplitBins2; }
#if VX_DEBUG_ISLAND_SPLITTER
			const uint32* ConstraintIndicesBuffer() const { return mConstraintIndices.data(); }
#else
			const uint32* ConstraintIndicesBuffer() const { return mConstraintIndices; }
#endif // VX_DEBUG_ISLAND_SPLITTER
			IslandRange<uint32> ContactConstraintIndicesIslandRange(uint32 island_idx, uint32 bin) const;
			IslandRange<uint32> NonContactConstraintIndicesIslandRange(uint32 island_idx, uint32 bin) const;

		private:
			unsigned int SplitParticipatingBodies(uint32 body_active_idxA, uint32 body_active_idxB);

			//IslandSplitBins& GetIslandSplitBin(uint32 island_idx)
			//{
			//	VX_ASSERT(island_idx < mIslandCacheBins.size());
			//	return mIslandCacheBins[island_idx];
			//}

			//const IslandSplitBins& GetIslandSplitBin(uint32 island_idx) const
			//{
			//	VX_ASSERT(island_idx < mIslandCacheBins.size());
			//	return mIslandCacheBins[island_idx];
			//}
			/// need to remove; just for debugging
			//std::vector<IslandSplitBins> mIslandCacheBins;

#if VX_DEBUG_ISLAND_SPLITTER
			std::vector<BinMask> mBodyBinMasks;

			std::vector<bool> mIslandIsLarge;
			//
						//uint32* mConstraintsIndicesBuffer
			std::vector<uint32> mConstraintIndices;
#else
			BinMask* mBodyBinMasks = nullptr;
			bool* mIslandIsLarge = nullptr;

			uint32* mConstraintIndices = nullptr;
			uint32 mConstaintIndicesCount = 0;
#endif // VX_DEBUG_SPLITTER



			uint32 mStepLargeIslandCount = 0;
			uint32 mConstraintAllocatedTail = 0;
			/// for now mIslandSplitBins2 == num of island; 
			/// later mIslandSplitBins2 should equal num of large island and 
			/// inorder of sorted island indices
			IslandSplitBins* mIslandSplitBins2 = nullptr;
			/// island count
			uint32 mIslandSplitBins2Size = 0;





			/// old need to remove
			///QUICK HACK
			/// REMOVE LATER, USE AS A TEMP ALLOCATION 
			/// TO MANAGE ISLAND DECOMPOSITION
			std::vector<uint32> mTempConstraintIndicesBins[kMaxBin + 1];
			/// old need to remove
			///QUICK HACK
			/// REMOVE LATER, USE AS A TEMP ALLOCATION 
			/// TO MANAGE ISLAND DECOMPOSITION
			std::vector<uint32> mTempNonConstraintIndicesBins[kMaxBin + 1];
		};

		const Splitter& GetSplitter() const { return mSplitter; }
		Splitter& GetSplitter() { return mSplitter; }

	private:
		Splitter mSplitter;
	};

} /// namespace vx 