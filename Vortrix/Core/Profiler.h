#pragma once

#include "Core.h"


#if VX_PROFILING
#define VX_ENABLE_PROFILING
#endif // VX_PROFILING


#ifdef VX_ENABLE_PROFILING
#include <chrono>
#include "Logger.h"
#include "EProfileMode.h"
#include <unordered_map>

#include "Vortrix/Core/Atomics.h"

#if defined(TRACY_ENABLE)
#if VX_USE_TRACY
#include "external/TracyProfiler/tracy/Tracy.hpp"
#endif // VX_USE_TRACY

#endif // defined(TRACY_ENABLE)

#endif // VPHX_ENABLE_PROFILING
				

namespace vx
{
	namespace Profiler
	{
#ifdef VX_ENABLE_PROFILING

		struct alignas(32) ProfileData
		{
			static constexpr float kEwmaOldWeight = 0.9f;
			static constexpr float kEwmaNewWeight = 0.1f;

			/// Lifetime profile statistics
			uint64 samples = 0;
			double totalMs = 0.0;

			/// most recent sample
			float lastMs = 0.0f;

			/// smoothed real-time profiler v
			float avgMs = 0.0f;

			/// lifetime maxmimum
			float maxMs = 0.0f;

			double ArithmeticAvg() const { return (samples > 0) ? totalMs / double(samples) : 0.0; }
			void ResetAccumulation()
			{
				samples = 0;
				totalMs = 0.0;
			}
		};

		class ProfilerCollector
		{
		public:
			static ProfilerCollector& Instance()
			{
				static ProfilerCollector inst;
				return inst;
			}


			void AddSample(const std::string_view& name, float ms)
			{
				std::string key(name);

				bool found = false;
				std::atomic<ProfileData>* target_profile_slot = nullptr;

				/// attempt to read 
				do
				{

					if (mSafeRead.load(std::memory_order_acquire))
					{
						auto it = mProfiles.find(key);
						if (it != mProfiles.end())
						{
							target_profile_slot = it->second.get();
							found = true;
						}
						else
							found = false;

						break;
					}

					_mm_pause(); ///overspinnign cpu clock, slice time
				}
				///try again if we cant read
				while (true);


				if (!found)
				{
					/// extra caveat, more than one thread might fail to find a slot 
					/// so one should attempt allocation while other loops, for turn
					/// 
					
					bool safe_read_write = true;
					for (;;)
					{

						if (mSafeRead.compare_exchange_weak(safe_read_write, false, std::memory_order_acquire))
						{
							/// new profile data
							/// one thread, could particpate at a time
							/// 
							/// but a caveat, a thread could have beaten us to new allocation
							/// 
							//ProfileData& data = mProfiles[name]; //<- support caveat, since one thread a time, this gets/create based on condition

							auto it = mProfiles.find(key);
							if (it == mProfiles.end())
							{
								/// new data
								ProfileData data;
								data.totalMs = double(ms);
								data.lastMs = ms;
								data.avgMs = ms;
								data.samples = 1;
								data.maxMs = ms;

								/// could allocate & assign value 
								/// since this would be the only thread during allocation
								/// during 
								mProfiles[name] = vx::MakeScope<std::atomic<ProfileData>>(data);

								/// no we publish, so other threads can attempt read/write
								//re open atomic gate for all subsequent concureent readers
								mSafeRead.store(true, std::memory_order_release);
								return; ///allocation and sample adding complete
							}
							
							/// another thread beat us, 
							/// break out for sample update
							target_profile_slot = it->second.get();
							mSafeRead.store(true, std::memory_order_release);
							break;
						}
						safe_read_write = true;
						_mm_pause(); ///overspinnign cpu clock, slice time
					}
				}


				VX_ASSERT(target_profile_slot);
				ProfileData old_profile_data = target_profile_slot->load(std::memory_order_acquire);
				for (;;)
				{
					ProfileData data;
					data.lastMs = ms;
					data.totalMs = old_profile_data.totalMs + double(ms);
					data.avgMs = old_profile_data.avgMs * ProfileData::kEwmaOldWeight + ms * ProfileData::kEwmaNewWeight;
					data.samples = old_profile_data.samples + 1;
					data.maxMs = vx::VxMax(old_profile_data.maxMs, ms);

					if(target_profile_slot->compare_exchange_weak(old_profile_data, data, std::memory_order_release))
						return;

					/// failed 
					_mm_pause(); ///overspinnign cpu clock, slice time
					old_profile_data = target_profile_slot->load(std::memory_order_acquire);
				}
				
			}

			const std::unordered_map<std::string_view, vx::Scope<std::atomic<ProfileData>>>& UnsafeProfiles() const { return mProfiles; }

			void ResetProfiles()
			{
				mProfiles.clear();
			}
			std::unordered_map<std::string_view, ProfileData> CopyProfiles()
			{
				bool safe_read = true;
				std::unordered_map<std::string_view, ProfileData> snapshots;
				snapshots.reserve(mProfiles.size());
				for (;;)
				{
					if (mSafeRead.compare_exchange_weak(safe_read, false, std::memory_order_acquire))
					{

						for (const auto& [name, atomic_profile_ptr] : mProfiles)
						{
							if (atomic_profile_ptr)
							{
								snapshots[name] = atomic_profile_ptr->load(std::memory_order_relaxed);
							}
						}


						//re open atomic gate for all subsequent concureent readers
						mSafeRead.store(true, std::memory_order_release);
						return snapshots;
					}

					safe_read = true;
					_mm_pause(); ///cpu yield
				}
			}

			struct ExportSnapshot
			{
				std::string_view name;
				ProfileData metrics;
			};
			
			std::vector<ExportSnapshot> SnapshotProfiles()
			{
				bool safe_read = true;
				std::vector<ExportSnapshot> snapshots;
				snapshots.reserve(mProfiles.size());
				for (;;)
				{
					if (mSafeRead.compare_exchange_weak(safe_read, false, std::memory_order_acquire))
					{

						for (const auto& [name, atomic_profile_ptr] : mProfiles)
						{
							if (atomic_profile_ptr)
								snapshots.push_back({ name, atomic_profile_ptr->load(std::memory_order_relaxed) });
						}


						//re open atomic gate for all subsequent concureent readers
						mSafeRead.store(true, std::memory_order_release);
						return snapshots;
					}

					safe_read = true;
					_mm_pause(); ///cpu yield
				}
			}

		private:
			std::atomic<bool> mSafeRead = true;
			std::unordered_map<std::string_view, vx::Scope<std::atomic<ProfileData>>> mProfiles;
		};



		class TimeTaken
		{
		public:
			explicit TimeTaken(const char* name,
				float* duration,
				bool log,
				EProfileMode mode = EProfileMode::Deterministic) :
				mStart({}),
				mName(name),
				mDurationMS(duration),
				bLog(log),
				mSampleRate(sDefaultGlobalSampleRate),
				bActive(false),
				mMode(mode)
			{
				Query();
			}

			//TimeTaken(const char* name,
			//	float* duration,
			//	bool log,
			//	EProfileMode mode = EProfileMode::Deterministic) :
			//	mStart({}),
			//	mName(name),
			//	mDurationMS(duration),
			//	bLog(log),
			//	mSampleRate(sDefaultGlobalSampleRate),
			//	bActive(false),
			//	mMode(mode)
			//{
			//	Query();
			//}
			explicit TimeTaken(const char* name,
				float* duration, EProfileMode mode = EProfileMode::Deterministic) :
				mStart({}),
				mName(name),
				mDurationMS(duration),
				bLog(true),
				mSampleRate(sDefaultGlobalSampleRate),
				bActive(false),
				mMode(mode)
			{
				Query();
			}

			explicit TimeTaken(const char* name,
				uint32_t sample_rate, EProfileMode mode = EProfileMode::Deterministic) :
				mStart({}),
				mName(name),
				mDurationMS(nullptr),
				bLog(true),
				mSampleRate(sample_rate),
				bActive(false),
				mMode(mode)
			{
				Query();
			}


			explicit TimeTaken(const char* name, EProfileMode mode = EProfileMode::Deterministic) :
				mStart({}),
				mName(name),
				mDurationMS(nullptr),
				bLog(true),
				mSampleRate(sDefaultGlobalSampleRate),
				bActive(false),
				mMode(mode)
			{
				Query();
			}

			//void Query(const char* name, float* duration, bool b_log, 
			//	EProfileMode mode, uint32_t sample_rate) 
			void Query()
			{
				//Only profile if mode matches filter
				if (!Contains(sActiveFilters, mMode))
				{
					bActive = false;
					return;
				}

				switch (mMode)
				{
				case EProfileMode::Deterministic:
					//increment the counter for this scope
					bActive = (++sScopeCounter[mName] % mSampleRate == 0);
					break;

				case EProfileMode::VariableRate:
					auto now = Clock::now();
					float elapsed = std::chrono::duration<float>(now - sLastSampleTime[mName]).count();
					if (elapsed > sSampleIntervalSeconds)
					{
						sLastSampleTime[mName] = now;
						bActive = true;
					}
					break;
				}

				if (bActive)
					mStart = Clock::now();

			}

			~TimeTaken()
			{
				if (!bActive)
					return;

				auto end = Clock::now();
				float ms = std::chrono::duration<float, std::milli>(end - mStart).count();

				if (mDurationMS)
					*mDurationMS = ms;
				if (sAllowConsoleLog && bLog)
					VX_LOG_INFO(mName, " - time: ", ms, "ms.");

				ProfilerCollector::Instance().AddSample(mName, ms);
			}

			//set global rate
			static void SetDefaultGlobalSampleRate(const uint32_t value) { sDefaultGlobalSampleRate = VxMax(value, uint(1)); }
			static void SetSampleIntervalSeconds(const float value) { sSampleIntervalSeconds = value; }
			static void SetProfileFilter(EProfileMode filter) { sActiveFilters = filter; }
			static void SetAllowConsoleLog(bool value) { sAllowConsoleLog = value; }

		private:
			using Clock = std::chrono::steady_clock;
			//using ClockDuration = std::chrono::duration;
			std::chrono::time_point<Clock> mStart;
			const char* mName;
			float* mDurationMS = nullptr;
			bool bLog = true;
			uint32_t mSampleRate = 1;
			bool bActive = false;
			EProfileMode mMode = EProfileMode::Deterministic;


			//static counter Per scope counter
			inline static std::unordered_map<std::string_view, uint32_t> sScopeCounter;
			inline static std::unordered_map<std::string_view, Clock::time_point> sLastSampleTime;
			inline static uint32_t sDefaultGlobalSampleRate = 30; //every 30 physics step
			inline static float sSampleIntervalSeconds = 0.5f;
			inline static EProfileMode sActiveFilters = EProfileMode::All;
			inline static bool sAllowConsoleLog = true;
		}; //TimeTaken struct


		//#define CORE_FUNCTION_NAME __FUNCTION__
		//argument scope name, 2nd arg memory address (float) to write scope duration
	#define VX_SELECT_2(arg1, arg2, TARGET_MACRO, ...) TARGET_MACRO
	
	#define VX_PROFILER_FUNCTION_NO_PARAM(name, mode) \
		Profiler::TimeTaken scope_time(name, mode)
	
	#define VX_PROFILER_FUNCTION_WITH_PARAM(name, mode, ...) \
		Profiler::TimeTaken scope_time(name, __VA_ARGS__, mode)
	
	
	
	#if VX_USE_TRACY
	
		#define VX_MARK_NEW_FRAME FrameMark
		
		#define VX_PROFILE_SCOPE(name, ...) ZoneScopedN(name)
		
		#define VX_PROFILE_FUNCTION(...) ZoneScoped
		
		#define VX_VARIABLE_PROFILE_SCOPE(name, ...) ZoneScopedN(name)
		
		#define VX_VARIABLE_PROFILE_FUNCTION(...) ZoneScoped
	
	#else
	
		#define VX_MARK_NEW_FRAME
		
		#define VX_PROFILE_SCOPE(name, ...) \
			VX_SELECT_2(name, VX_PROFILER_FUNCTION_NO_PARAM, VX_PROFILER_FUNCTION_WITH_PARAM) \
			(name, Profiler::EProfileMode::Deterministic, __VA_ARGS__) 
		
		
		#define VX_PROFILE_FUNCTION(...) \
			EXPAND_MACRO(VX_SELECT_2(__FUNCTION__, VX_PROFILER_FUNCTION_NO_PARAM, VX_PROFILER_FUNCTION_WITH_PARAM) \
			(__FUNCTION__, Profiler::EProfileMode::Deterministic, __VA_ARGS__) )
		
		
		
		#define VX_VARIABLE_PROFILE_SCOPE(name, ...) \
			VX_SELECT_2(name, VX_PROFILER_FUNCTION_NO_PARAM, VX_PROFILER_FUNCTION_WITH_PARAM) \
				(name, Profiler::EProfileMode::VariableRate, __VA_ARGS__)
		
		#define VX_VARIABLE_PROFILE_FUNCTION(...) \
			EXPAND_MACRO(VX_SELECT_2(__FUNCTION__, VX_PROFILER_FUNCTION_NO_PARAM, VX_PROFILER_FUNCTION_WITH_PARAM) \
				(__FUNCTION__, Profiler::EProfileMode::VariableRate, __VA_ARGS__) )
	
	#endif // VX_USE_TRACY
	
	#define SET_VX_PROFILER_GLOBAL_SAMPLE_RATE(rate) Profiler::TimeTaken::SetDefaultGlobalSampleRate(rate)
	#define SET_VX_PROFILER_SAMPLE_INTERVAL_SECONDS(rate) Profiler::TimeTaken::SetSampleIntervalSeconds(rate)
	#define SET_VX_PROFILER_FILTER(filter) Profiler::TimeTaken::SetProfileFilter(filter)
	#define SET_VX_PROFILER_ALLOW_CONSOLE_LOG(value) Profiler::TimeTaken::SetAllowConsoleLog(value)
	
	#define VX_PROFILER_DETERMINISTIC Profiler::EProfileMode::Deterministic
	#define VX_PROFILER_VARIABLERATE Profiler::EProfileMode::VariableRate

#else

	#define VX_MARK_NEW_FRAME

	#define VX_PROFILE_SCOPE(...) ((void)0)
	#define VX_PROFILE_FUNCTION(...) ((void)0)

	#define VX_VARIABLE_PROFILE_FUNCTION(...) ((void)0)
	#define VX_VARIABLE_PROFILE_SCOPE(...) 

	#define PROFILE_FUNCTION_SAMPLE(...) ((void)0)
	#define SET_VX_PROFILER_GLOBAL_SAMPLE_RATE(rate) ((void)0)
	#define SET_VX_PROFILER_SAMPLE_INTERVAL_SECONDS(rate) ((void)0)
	#define SET_VX_PROFILER_FILTER(filter) ((void)0)
	#define SET_VX_PROFILER_ALLOW_CONSOLE_LOG(value) ((void)0)
	
	#define VX_PROFILER_DETERMINISTIC 0
	#define VX_PROFILER_VARIABLERATE 0
#endif // VX_ENABLE_PROFILING
	} //Profiler namespace
} //vx namespace