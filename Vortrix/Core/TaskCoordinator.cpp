#include "TaskCoordinator.h"

#include "Assertion.h"

#include <windows.h> ///For void SetThreadNameVS(const char* threadName)  TASK COORD Multi threaded 
#include <string> ///For  TASK COORD Multi threaded 

#include "Profiler.h"
#include "Vortrix/Maths/ScalarMath.h"

#include "Assertion.h"
#include "StackString.h"

namespace vx {

#pragma region TASK
	bool Task::DecrementDependency()
	{
		uint32_t v = mDependencies.fetch_sub(1, std::memory_order_relaxed);
		uint32_t _v = v - 1;
		VX_ASSERT(v > _v);
		return _v == 0;
	}

	void Task::RemoveDependency()
	{
		if (DecrementDependency())
			mCoordinator->EnqueueTask(this);
	}
#pragma endregion ///TASK


#pragma region TASK COORD 
	void TaskCoordinator::RemoveTasksDependency(Task** tasks, uint32_t task_count)
	{
		// check if task is part of coordinator 
		/// const uint8* _addr = reinterpret_cast<const uint8*>(addr);
		/// return _addr >= mMemStart && _addr < mMemStart + mStackSize;
		/// 
		VX_ASSERT(task_count > 0);

		Task** enqueuing_tasks = (Task**)alloca(task_count * sizeof(Task*));
		Task** enqueuing_top = enqueuing_tasks;

		for (Task** task = tasks, **task_end = tasks + task_count;
			task < task_end; ++task)
		{
			if ((*task)->DecrementDependency())
				*(enqueuing_top++) = *task;
		}

		uint32_t task_to_queue = uint32_t(enqueuing_top - enqueuing_tasks);
		if (task_to_queue > 0)
			EnqueueTasks(enqueuing_tasks, task_to_queue);
	}
#pragma endregion ///TASK COORD

#pragma region TASK COORD Sequential
	void TaskCoordinatorSequential::EnqueueTasks(Task** tasks, uint32_t count)
	{
		VX_PROFILE_FUNCTION();
		for (Task** task = tasks, **task_end = tasks + count;
			task < task_end; ++task)
		{
			VX_PROFILE_SCOPE("Processing Task");
			Task* _task = *task;
			ImmediateTaskProcess(_task);
			delete _task;
		}
	}
#pragma endregion ///TASK COORD Sequential

#pragma region TASK COORD Multi threaded 
	void TaskCoordinatorMt::BeginThreads(int thread_count)
	{
		if (thread_count < 0)
			thread_count = std::thread::hardware_concurrency() - 1;

		mOnQuitThreads = false;

		VX_ASSERT(thread_count < kMaxThreads);
		VX_ASSERT(mWorkers.empty());

#if LOCKFREE_CAS_QUEUE
		for (auto& t : mPendingTasks)
			t = nullptr;

		mPendingTasksTail = 0;
		mAvailableTaskCount = 0;

		mNumWorkerThread = thread_count;
		mNumActiveWorkerThread = thread_count;

		/// ensure mThreadTaskHead data is int/ trivial 
		//std::fill(mThreadTaskHead, &mThreadTaskHead[kMaxThreads + 1], 0);
		std::memset(mThreadTaskHead, 0, sizeof(mThreadTaskHead));
#endif LOCKFREE_CAS_QUEUE



		mWorkers.reserve(thread_count);
		for (int i = 0; i < thread_count; ++i)
			mWorkers.emplace_back([this, i] {ThreadsMainLoop(i); });
	}

	void TaskCoordinatorMt::EndThreads()
	{
		///* done
			//flag for program completion, which is shared by all threads. 
			//That keeps them spinning/updating and operation their would stop when program is done*/
			////sleep = true;
		for (auto& worker : mWorkers)
		{
			if (worker.joinable())
				worker.join();
		}

		///Reset Workers / Clear
		mWorkers.clear();

		/// any left jobs 
#if LOCKFREE_CAS_QUEUE
		if (mAvailableTaskCount.load() > 0)
		{
			for (auto& t : mPendingTasks)
			{
				Task* _t = t.load();
				if (_t == nullptr) continue;
				_t->Process();
				delete _t;
				t = nullptr;
			}
		}

		mPendingTasksTail = 0;
		mAvailableTaskCount = 0;
		mNumWorkerThread = 0;
		mNumActiveWorkerThread = 0;

		std::memset(mThreadTaskHead, 0, sizeof(mThreadTaskHead));
#else
		for (auto& task : mTasks)
		{
			task->Process();
			delete task;
			task = nullptr;
		}

		mTasks.clear();
#endif // LOCKFREE_CAS_QUEUE
	}

	void SetThreadNameVS(const char* threadName)
	{
		const DWORD MS_VC_EXCEPTION = 0x406D1388;

#pragma pack(push,8)
		typedef struct tagTHREADNAME_INFO
		{
			DWORD dwType; // Must be 0x1000.
			LPCSTR szName; // Pointer to name (in user addr space).
			DWORD dwThreadID; // Thread ID (-1=caller thread).
			DWORD dwFlags; // Reserved for future use, must be zero.
		} THREADNAME_INFO;
#pragma pack(pop)

		// DWORD dwThreadID = ::GetThreadId( static_cast<HANDLE>( t.native_handle() ) );
		//DWORD dwThreadID = ::GetThreadId(); 
		DWORD dwThreadID = (DWORD)-1;

		THREADNAME_INFO info;
		info.dwType = 0x1000;
		info.szName = threadName;
		info.dwThreadID = dwThreadID;
		info.dwFlags = 0;

		__try
		{
			RaiseException(MS_VC_EXCEPTION, 0, sizeof(info) / sizeof(ULONG_PTR), (ULONG_PTR*)&info);
		}
		__except (EXCEPTION_EXECUTE_HANDLER)
		{
		}
	}

	void TaskCoordinatorMt::ThreadsMainLoop(int thread_worker_idx)
	{
		THREAD_LOG_MSG("About to commence worker thread idx: " << thread_worker_idx << " my os id " << std::this_thread::get_id() << "\n");

		{
			std::string _name = "Worker Thread ";
			_name += std::to_string(thread_worker_idx);

#if VX_USE_TRACY
			tracy::SetThreadName(_name.c_str());
#endif // VX_USE_TRACY
			SetThreadNameVS(_name.c_str());
		}


#if LOCKFREE_CAS_QUEUE
		while (!mOnQuitThreads)
		{
			///keep scanning insytead of going back to sleep on fail
			//wait no task, semaphore
			{
				std::unique_lock<std::mutex> no_task_lock(mTaskQueueMutex);

				mTaskAvailable.wait(no_task_lock, [this]()
					{
						if (mMainThreadWaitingTask.load(std::memory_order_relaxed) &&
							mProcessingTasks.load(std::memory_order_acquire) == 0) /// process_count - 1 <= 0
							SignalMainThread();

						return mQuickHackThreadCatchUp || 
							mAvailableTaskCount.load(std::memory_order_relaxed) > 0 || 
							mOnQuitThreads;
					});


				if (mOnQuitThreads)
					break;
			}


			{
				VX_PROFILE_SCOPE("Processing Tasks");
					///need to fix if awake, and there is task and not at tail 
					/// instead of sleeping, scan till tail before sleep
					while (mThreadTaskHead[thread_worker_idx] != mPendingTasksTail.load())
					{

#if PRESISTENT_TASK_SIGNAL
						uint32_t avail_task = 0;
						avail_task = mAvailableTaskCount.load(std::memory_order_acquire);
						/// after lock release, quick notify just incase of delay
						/// notify - 1; because might get the task 
						/// but its not guaranteed
						SignalAvailableTask(avail_task - 1);
#endif // PRESISTENT_TASK_SIGNAL


						Task* task = nullptr;
						task = mPendingTasks[WrapPowerof2(mThreadTaskHead[thread_worker_idx].load(), kMaxTaskQueue - 1)].exchange(nullptr);

						//null means no task at slot or other thread pickup 
						if (task)
						{
							{
								VX_PROFILE_SCOPE("Acquired New Task");
								/// if we actual get the task 
								/// quick consume work counter and was successful 
								/////but the means that the notify needs to be accurate 
								//mAvailableTaskCount.fetch_sub(1, std::memory_order_relaxed);

								///// we will be processing the so the waiting thread for processing task will be aware
								//mProcessingTasks.fetch_add(1, std::memory_order_relaxed);



								/// to prevent a race condition where 
								/// both avail & process = 0 and waiting decide to complete
								/// 
								/// 
								/// available: 1 -> 0
								/// processing: 0 
								/// 
								/// main might see available == 0 && processing == 0 {wait complete}
								/// 
								/// but with 
								/// processing: 0 -> 1
								/// available: 1 -> 0 
								/// 
								/// potential outcome: 0 & 1, 1 & 1, 1 & 0
								/// 
								mProcessingTasks.fetch_add(1, std::memory_order_relaxed);
								//mAvailableTaskCount.fetch_sub(1, std::memory_order_relaxed);
								mAvailableTaskCount.fetch_sub(1, std::memory_order_release);
							}

							task->Process();

							//we are done processing 
							//uint32_t process_count = mProcessingTasks.fetch_sub(1, std::memory_order_acq_rel);
							//mProcessingTasks.fetch_sub(1, std::memory_order_relaxed);
							uint32 processed = mProcessingTasks.fetch_sub(1, std::memory_order_release);

							/// signal main thread if waiting for task process complete 
			/*				if (mMainThreadWaitingTask && process_count < 1) /// process_count - 1 <= 0
								SignalMainThread();*/

								//might have to delete heap allocated 
#if USE_TASK_CONSTURCT_BUFF
							mTaskBuffer.Deconstruct(task);
#else
							delete task;
#endif // USE_TASK_CONSTURCT_BUFF

							if (processed == 1 && mMainThreadWaitingTask.load(std::memory_order_relaxed)) /// process_count - 1 <= 0
								SignalMainThread(); //might break 

							THREAD_LOG_MSG("____    Thread " << thread_worker_idx << " complete a Task. _____ Task Active: " << mProcessingTasks.load(std::memory_order_relaxed) << ".\n");
						}
						mThreadTaskHead[thread_worker_idx]++; //progress

						///remove 
						//SignalMainThread();

						//check reflection to prevent, unnecessary scanning 
		/*				avail_task = mAvailableTaskCount.load(std::memory_order_relaxed);
						if (avail_task <= 0)
							break;*/

					}
			}



			//if (mMainThreadWaitingTask && mProcessingTasks.load(std::memory_order_relaxed) <= 0) /// process_count - 1 <= 0
			if (mMainThreadWaitingTask.load(std::memory_order_relaxed) && 
				mProcessingTasks.load(std::memory_order_acquire) == 0) /// process_count - 1 <= 0
				SignalMainThread();

			//const int processing = mProcessingTasks.load(std::memory_order_acquire);

			//if(mProcessingTasks <= 0 && mMainThreadWaitingTask && )
			//VX_ASSERT(processing > -1, (StackString<16>("Value: ") << processing).Data());
		}
#else

		while (!mOnQuitThreads)
		{
			Task* task = nullptr;
			size_t task_count = 0;
			{
				std::unique_lock<std::mutex> queue_mutex(mTaskQueueMutex);

				const auto& tasks = &mTasks;
				mTaskAvailable.wait(queue_mutex, [this, tasks]() {
					return !tasks->empty() || mOnQuitThreads;
					//return !mJobs.empty() || mOnQuitThreads;
					});

				if (mOnQuitThreads && mTasks.empty())
					break;


				task = mTasks.front();
				//remove job 
				mTasks.pop_front();
				//mActiveJobs++; //lock freee atomic increment
				mProcessingTasks.fetch_add(1, std::memory_order_relaxed);

				task_count = mTasks.size();
			}

			//if (task_count > 0)
			//	mTaskAvailable.notify_one();
			SignalAvailableTask(task_count);

			//execute job
			task->Process();

			uint32_t process_count = mProcessingTasks.fetch_sub(1, std::memory_order_relaxed);

			/// signal main thread if waiting for task process complete 
			if (mMainThreadWaitingTask && process_count < 1) /// process_count - 1 <= 0
				SignalMainThread();


			//delete task;

#if USE_TASK_CONSTURCT_BUFF
			mTaskBuffer.Deconstruct(task);
#else
		//might have to delete heap allocated 
			delete task;
#endif // USE_TASK_CONSTURCT_BUFF
			task = nullptr;


			THREAD_LOG_MSG(mThreadLogMutex, "____    Thread " << thread_worker_idx << " complete a Task. _____ Task Active: " << mProcessingTasks.load(std::memory_order_relaxed) << ".\n");
		}

#endif // LOCKFREE_CAS_QUEUE
		THREAD_LOG_MSG("____    Thread " << thread_worker_idx << " done. _____\n");
	}

	Task* TaskCoordinatorMt::ConstructTask(const TaskFunction& task_func, uint32_t dependencies)
	{
		VX_PROFILE_FUNCTION();
		Task* task = ConstructTaskInternal(task_func, dependencies);
		if (dependencies == 0)
			EnqueueTask(task);
		return task;
	}

	void TaskCoordinatorMt::EnqueueTask(Task* task)
	{
		VX_PROFILE_FUNCTION();
		size_t task_count = 0;

#if LOCKFREE_CAS_QUEUE
		/// use the far head (min) and tail, to capture immediate task range
		/// 
		/// bitwise wrap is used
		/// keep adjusting pending tail, until success 
		/// 
		/// but if the range btw far back head (min) and tail greater then pendable task
		/// 
		/// recompute far head as worker should have completed some task which might have 
		/// reduce the range, so to continue adjusting the tail. 
		/// 
		/// but after recomputing, it must have mean that the buffer is completely full.
		///  poke all worker thread , then go to quick few microsecond sleep to retry
		/// 
		///
		uint32_t capture_min_head = ComputeMinThreadHead();
		for (;;)
		{
			uint32_t curr_task_tail = mPendingTasksTail;
			//far back head (min) and tail range
			if (curr_task_tail - capture_min_head >= kMaxTaskQueue)
			{
				//recapute far back head min
				capture_min_head = ComputeMinThreadHead();
				/// is the range still greater, wait for worker thread
				/// to catch up
				if (curr_task_tail - capture_min_head >= kMaxTaskQueue)
				{
					{
						VX_PROFILE_SCOPE("Waiting free solt EnqueueTasks");
						///this is a quick hack, before algortithm fix 
						/// thread gets stuck at max even when no task
						mQuickHackThreadCatchUp = mAvailableTaskCount.load(std::memory_order_relaxed) <= 0;
						PokeWorkers();
						//VX_LOG_INFO("Waiting for frww slot");
						std::this_thread::sleep_for(std::chrono::microseconds(10));
						continue;
					}
				}
			}

			Task* t_expect = nullptr;
			bool succuss_add = mPendingTasks[WrapPowerof2(curr_task_tail, kMaxTaskQueue - 1)].compare_exchange_strong(t_expect, task);

			/// move to the next if CAS failed
			/// 
			mPendingTasksTail.compare_exchange_strong(curr_task_tail, curr_task_tail + 1);

			/// new tail 
			/// task was add to curr end of pending task
			if (succuss_add)
			{
				//for debug
				mAvailableTaskCount.fetch_add(1, std::memory_order_release);
				break;
			}
		}
		//reload task available for trigging
		task_count = mAvailableTaskCount.load(std::memory_order_relaxed);


#else
		{
			std::lock_guard<std::mutex> queue_mutex(mTaskQueueMutex);
			mTasks.push_back(task);
			task_count = mTasks.size();
		}

#endif // LOCKFREE_CAS_QUEUE

		//signal/notify threads waiting on flag
		SignalAvailableTask(task_count);

		mQuickHackThreadCatchUp = false;
	}

	void TaskCoordinatorMt::EnqueueTasks(Task** tasks, const uint32_t count)
	{
		VX_PROFILE_FUNCTION();
		size_t task_count = 0;
#if LOCKFREE_CAS_QUEUE
		//for (Task** task = tasks, **task_end = tasks + count;
		//	task < task_end; ++task)
		//	EnqueueTask(*task);

		////each enqueue notifies

		/// use the far head (min) and tail, to capture immediate task range
		/// 
		/// bitwise wrap is used
		/// keep adjusting pending tail, until success 
		/// 
		/// but if the range btw far back head (min) and tail greater then pendable task
		/// 
		/// recompute far head as worker should have completed some task which might have 
		/// reduce the range, so to continue adjusting the tail. 
		/// 
		/// but after recomputing, it must have mean that the buffer is completely full.
		///  poke all worker thread , then go to quick few microsecond sleep to retry
		/// 


		//Task** curr_task = tasks;
		//Task** task_end = tasks + count;

		uint32_t queued_pointer = 0;
		uint32_t capture_min_head = ComputeMinThreadHead();
		for (;;)
		{
			uint32_t curr_task_tail = mPendingTasksTail;
			//far back head (min) and tail range
			if (curr_task_tail - capture_min_head >= kMaxTaskQueue)
			{
				//recapute far back head min
				capture_min_head = ComputeMinThreadHead();
				/// is the range still greater, wait for worker thread
				/// to catch up
				if (curr_task_tail - capture_min_head >= kMaxTaskQueue)
				{
					///hack as semaphore to wake 
					{
						VX_PROFILE_SCOPE("Waiting free solt EnqueueTasks");
						mAvailableTaskCount.fetch_add(1, std::memory_order_relaxed);
						PokeWorkers();
						std::this_thread::sleep_for(std::chrono::microseconds(100));
						continue;
					}
				}
			}

			Task* t_expect = nullptr;
			//bool succuss_add = mPendingTasks[Utils::WrapPowerof2(curr_task_tail, kMaxTaskQueue - 1)].compare_exchange_strong(t_expect, *curr_task);
			bool succuss_add = mPendingTasks[WrapPowerof2(curr_task_tail, kMaxTaskQueue - 1)].compare_exchange_strong(t_expect, tasks[queued_pointer]);

			/// move to the next if CAS failed
			/// 
			mPendingTasksTail.compare_exchange_strong(curr_task_tail, curr_task_tail + 1);

			/// new tail 
			/// task was add to curr end of pending task
			if (succuss_add)
			{
				//for debug
				mAvailableTaskCount.fetch_add(1, std::memory_order_release);

				queued_pointer++;
				if (queued_pointer >= count)
					break;
				//curr_task++;
				//if(curr_task >= task_end)
				//	break;
			}
		}
		//reload task available for trigging
		task_count = mAvailableTaskCount.load(std::memory_order_acquire);

		SignalAvailableTask(task_count);

#else
		{
			std::lock_guard<std::mutex> queue_mutex(mTaskQueueMutex);

			for (Task** task = tasks, **task_end = tasks + count;
				task < task_end; ++task)
			{
				mTasks.push_back(*task);
			}

			task_count = mTasks.size();
		}

		//signal/notify threads waiting on flag
	//mJobFlag.notify_one();
		SignalAvailableTask(task_count);
#endif // LOCKFREE_CAS_QUEUE
	}

	void TaskCoordinatorMt::ParallelFor(uint32 count, uint32 batch_size, const RangeTask& task)
	{
		for (uint32 i = 0; i < count; i += batch_size)
		{
			uint32 begin = i;
			uint32 end = VxMin(i + batch_size, count);

			ConstructTask(
			[task, begin, end]()
			{
				task.Execute(task.userData, begin, end);
			}, 0); /// no dependancies
		}
	}



	void TaskCoordinatorMt::WaitForTasks()
	{
		THREAD_LOG_MSG("Wating for tasks completion.......!\n");



		mMainThreadWaitingTask.store(true, std::memory_order_relaxed);
		//for helping out 
		Task* task;

		///later while waiting for task 
		///also support quit break as well
#if LOCKFREE_CAS_QUEUE

	/// for hack catch up 
		mThreadTaskHead[mWorkers.size()] = ComputeMinThreadHead();
		mNumActiveWorkerThread = mNumWorkerThread + 1;

		while (!mOnQuitThreads)
		{
			THREAD_LOG_MSG("____  Main Thread is waiting for " << mAvailableTaskCount.load(std::memory_order_relaxed) << " tasks and " << mProcessingTasks << " active tasks to complete_____\n");
			//wait no task, semaphore
			uint32_t avail_task = 0;
			{
				std::unique_lock<std::mutex> no_task_lock(mTaskQueueMutex);

				if (mProcessingTasks.load(std::memory_order_acquire) == 0 &&
					mAvailableTaskCount.load(std::memory_order_acquire) == 0)
					break;

				mMainWaitFlag.wait(no_task_lock, [this]()
					{
						return (mAvailableTaskCount.load(std::memory_order_relaxed) > 0 || 
									mProcessingTasks.load(std::memory_order_relaxed) == 0);
						//return avail_task > 0 || processing_task <= 0;
					});

				avail_task = mAvailableTaskCount.load(std::memory_order_acquire);

				if (avail_task == 0 && mProcessingTasks.load(std::memory_order_acquire) == 0) //check the job job quueue just incase of race condition 
					break;
			}

			//quick hack, if no work 
			if (avail_task <= 0) 
			{
				_mm_pause();
				continue;
			}

			///// after lock release, quick notify just incase of delay
			///// notify - 1; because might get the task 
			///// but its not guaranteed
			//SignalAvailableTask(avail_task - 1);
			//if (mMainThreadWaitingTask)
			//	SignalMainThread(avail_task - 2); /// task_count - 1, has another could have be notified


			//Task* task = nullptr;
			//task = mPendingTasks[Utils::WrapPowerof2(mThreadTaskHead[mNumWorkerThread].load(std::memory_order_relaxed), kMaxTaskQueue - 1)].exchange(nullptr);
			////null means no task at slot or other thread pickup 
			//if (task)
			//{
			//	/// if we actual get the task 
			//	/// quick consume work counter and was successful 
			//	///but the means that the notify needs to be accurate 
			//	mAvailableTaskCount.fetch_sub(1, std::memory_order_acq_rel);

			//	/// we will be processing the so the waiting thread for processing task will be aware
			//	mProcessingTasks.fetch_add(1, std::memory_order_relaxed);

			//	task->Process();

			//	//we are done processing 
			//	mProcessingTasks.fetch_sub(1, std::memory_order_relaxed);

			//	//might have to delete heap allocated 
			//	delete task;
			//	THREAD_LOG_MSG(mThreadLogMutex, "____    MainThread complete a task._____\n");
			//}
			//mThreadTaskHead[mNumWorkerThread]++; //progress



			{
				VX_PROFILE_SCOPE("Processing Tasks");
				///need to fix if awake, and there is task and not at tail 
				/// instead of sleeping, scan till tail before sleep
				while (mThreadTaskHead[mNumWorkerThread] != mPendingTasksTail.load())
				{
					/// after lock release, quick notify just incase of delay
					/// notify - 1; because might get the task 
					/// but its not guaranteed
					SignalAvailableTask(avail_task - 1);


					Task* task = nullptr;
					task = mPendingTasks[WrapPowerof2(mThreadTaskHead[mNumWorkerThread].load(), kMaxTaskQueue - 1)].exchange(nullptr);

					//null means no task at slot or other thread pickup 
					if (task)
					{
						{
							VX_PROFILE_SCOPE("Acquired New Task");
							/// if we actual get the task 
							/// quick consume work counter and was successful 
							///but the means that the notify needs to be accurate 
							//mAvailableTaskCount.fetch_sub(1, std::memory_order_acq_rel);
							//mAvailableTaskCount.fetch_sub(1, std::memory_order_relaxed);

							///// we will be processing the so the waiting thread for processing task will be aware
							//mProcessingTasks.fetch_add(1, std::memory_order_relaxed);


							/// to prevent a race condition where 
							/// both avail & process = 0 and waiting decide to complete
							/// 
							/// 
							/// available: 1 -> 0
							/// processing: 0 
							/// 
							/// main might see available == 0 && processing == 0 {wait complete}
							/// 
							/// but with 
							/// processing: 0 -> 1
							/// available: 1 -> 0 
							/// 
							/// potential outcome: 0 & 1, 1 & 1, 1 & 0
							/// 
							mProcessingTasks.fetch_add(1, std::memory_order_relaxed);
							mAvailableTaskCount.fetch_sub(1, std::memory_order_relaxed);
						}

						task->Process();

						//we are done processing 
						mProcessingTasks.fetch_sub(1, std::memory_order_release);

#if USE_TASK_CONSTURCT_BUFF
						mTaskBuffer.Deconstruct(task);
#else
						//might have to delete heap allocated 
						delete task;
#endif // USE_TASK_CONSTURCT_BUFF
						THREAD_LOG_MSG("____    MainThread complete a task._____\n");
					}
					mThreadTaskHead[mNumWorkerThread]++; //progress

					//check reflection to prevent, unnecessary scanning 
					//avail_task = mAvailableTaskCount.load(std::memory_order_relaxed);
					//if (avail_task <= 0)
					//	break;
				}
			}

			//if (mProcessingTasks.load(std::memory_order_acquire) <= 0)
			//	break;

			if (mProcessingTasks.load(std::memory_order_acquire) == 0 &&
				mAvailableTaskCount.load(std::memory_order_acquire) == 0)
				break;
		}

		mNumActiveWorkerThread = mNumWorkerThread;

#else
	//wait to .2 of a sec for a signal. 
	//std::unique_lock<std::mutex> queue_mutex(m_JobPool.m_PoolMutex);
	//std::unique_lock<std::mutex> queue_mutex(mJobQueueMutex);
	//std::memory_order
		while (!mTasks.empty() || mProcessingTasks.load(std::memory_order_acquire) > 0)
		{
			THREAD_LOG_MSG(mThreadLogMutex, "____  Main Thread is waiting for " << mTasks.size() << " tasks and " << mProcessingTasks << " active tasks to complete_____\n");
			//std::this_thread::sleep_for(std::chrono::microseconds(5));

			size_t task_count = 0;

			///help out 
			{
				std::unique_lock<std::mutex> queue_mutex(mTaskQueueMutex);
				////this is bad main thread does not release mutex lock, when job and other thread are processing

				mMainWaitFlag.wait(queue_mutex, [this]() {
					return !mTasks.empty() || (mProcessingTasks.load(std::memory_order_relaxed) <= 0);
					});

				if (mTasks.empty() && mProcessingTasks.load(std::memory_order_relaxed) <= 0) //check the job job quueue just incase of race condition 
					break;

				//excute jobs 
				if (!mTasks.empty()) //check the job job quueue just incase of race condition 
				{
					task = mTasks.front();
					//remove job 
					mTasks.pop_front();
					//mActiveJobs++; //lock freee atomic increment
					mProcessingTasks.fetch_add(1, std::memory_order_relaxed);

					task_count = mTasks.size();
				}
				else
				{
					//if another thread as pick up work before this 
					task = nullptr;
					//maybe later have a cascade wrap for multiple cv
					//sleep if no job, but is new job (rare) or active job finised
					//mMainWaitFlag.wait(queue_mutex, [this]() {
					//	return !mTasks.empty() || (mProcessingTasks.load(std::memory_order_relaxed) <= 0);
					//	});

					//wrap check
					continue;
				}
			}
			///unique_lock mutex lock scope

			//if (task_count > 0)
			//	mTaskAvailable.notify_one();
			SignalAvailableTask(task_count);


			if (task)
			{
				//mThreadLogMutex.lock();
				//job();
				task->Process();
				//later ref count smart pointeer
				//delete task;

#if USE_TASK_CONSTURCT_BUFF
				mTaskBuffer.Deconstruct(task);
#else
			//might have to delete heap allocated 
				delete task;
#endif // USE_TASK_CONSTURCT_BUFF
				task = nullptr;

				//mThreadLogMutex.unlock();
				mProcessingTasks.fetch_sub(1, std::memory_order_relaxed);

				THREAD_LOG_MSG(mThreadLogMutex, "____    MainThread complete a task._____\n");
			}

		}
#endif LOCKFREE_CAS_QUEUE
		mMainThreadWaitingTask.store(false, std::memory_order_relaxed);
		THREAD_LOG_MSG("Thread Work finished !!!!!!!\n");

		const int processing = mProcessingTasks.load(std::memory_order_acquire);
		const uint32 avail = mAvailableTaskCount.load(std::memory_order_acquire);
		//VX_ASSERT(processing == 0, (StackString<32>("Value: ") << processing << "avil: " << avail).Data());
		VX_ASSERT(avail == 0, (StackString<16>("Value: ") << avail).Data());
	}



	void TaskCoordinatorMt::TaskBuffer::Deconstruct(uint32_t slot_idx)
	{
		//easy to swap next first free

		VX_ASSERT(slot_idx < mBufferTop);

		BufferData& deconstruct_buff_data = mBuffer[slot_idx];
		deconstruct_buff_data.taskData.~Task();

		///so new deconstruting buff need to point to an invlaid 
		/// next free/ 
		/// later could, point to self
		VX_ASSERT(deconstruct_buff_data.nextFree == slot_idx);

		for (;;)
		{
			////try get the next free if vaild to ensure no race condition 
			uint32_t capture_prev_nxt_free = mFirstNextFree.load(std::memory_order_acquire);

			//link node internallt first whill it is private 
			deconstruct_buff_data.nextFree.store(capture_prev_nxt_free, std::memory_order_relaxed);
			//if (capture_prev_nxt_free != kInvalidSlot)
			//{
			//	BufferData& nxt_free_buff = mBuffer[capture_prev_nxt_free];

			//}
			//mFirstNextFree.store(slot_idx, std::memory_order_release);
			if (mFirstNextFree.compare_exchange_weak(capture_prev_nxt_free, slot_idx, std::memory_order_release, std::memory_order_relaxed))
				return;
		}
	}
	void TaskCoordinatorMt::TaskBuffer::Deconstruct(Task* task)
	{
		//check if we own address
		const uint8_t* _addr = reinterpret_cast<const uint8_t*>(task);
		const uint8_t* buff_addr = reinterpret_cast<const uint8_t*>(mBuffer);
		VX_ASSERT(_addr >= buff_addr && _addr < buff_addr + (mMax * sizeof(BufferData)));

		///actuall if the address of Task == its BufferData 
		///then if the next free points to self then we can get the slot idx 
		BufferData* buff_data = reinterpret_cast<BufferData*>(task);
		uint32_t slot = buff_data->nextFree.load(std::memory_order_relaxed);
		Deconstruct(slot);
	}
#pragma endregion ///TASK COORD Multi threaded 

}//namespace vx