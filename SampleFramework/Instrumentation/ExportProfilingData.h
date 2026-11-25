#pragma once

#include "Vortrix/Core/Profiler.h"

#include <fstream>
#include <string>
#include <vector>

#include <format>
#include <chrono>

static void ExportProfilingDataToCSV(const std::string& file_name,
	std::uint32_t active_threads, std::uint32_t iterations, std::uint32_t constraint_count);
namespace vx {
	struct VelocitySolveProfile;
	struct SolverWorkloadStats;
}

static void ExportProfilingDataToCSV(
	const std::string& file_name, std::uint32_t active_threads,
	std::uint32_t iterations, const SolverWorkloadStats& work_load
);

static void ExportSolverLoadBalancingCSV(
	const std::string& file_name,
	uint32 active_threads, uint32 iterations,
	const vx::VelocitySolveProfile& profile,
	uint32 max_concurrency);
static void ExportSortedLoadBalancingToCSV(const std::string& file_name, const char* scene_name, const vx::VelocitySolveProfile& profile, uint32 max_concurrency);

class ProfilingSession
{
public:
	static constexpr const char* kParentFolder = "Profiling";

	ProfilingSession()
	{
		MakeSessionName();
	}
	void EnsureSessionDir()
	{
		static bool created = false;
		if (!created)
		{
			std::filesystem::create_directories(std::string(kParentFolder) + "/" + mSessionName);
			created = true;
		}
	}

	void ExportProfileDataCSV(const std::string& file_name,
		std::uint32_t active_threads, std::uint32_t iterations, std::uint32_t constraint_count)
	{
		//EnsureSessionDir();
		ExportProfilingDataToCSV(RelativeDir() + "/" + file_name + ".csv",
			active_threads, iterations, constraint_count);
	}

	void ExportSolverWorkloadCSV(
		const std::string& file_name, std::uint32_t active_threads,
		std::uint32_t iterations, const SolverWorkloadStats& work_load
	)
	{
		ExportProfilingDataToCSV(RelativeDir() + "/" + file_name + ".csv",
			active_threads, iterations, work_load);
	}

	void ExportSolverLoadBalancingCSV(
		const std::string& file_name,
		uint32 active_threads, uint32 iterations,
		const vx::VelocitySolveProfile& profile,
		uint32 max_concurrency) 
	{
		::ExportSolverLoadBalancingCSV(
			RelativeDir() + "/" + file_name + ".csv", active_threads,
			iterations, profile, max_concurrency);
	}

	void ExportSortedLoadBalancingToCSVII(const std::string& file_name, const char* scene_name,
		const vx::VelocitySolveProfile& profile, uint32 max_concurrency)
	{
		ExportSortedLoadBalancingToCSV(RelativeDir() + "/" + file_name + ".csv",
			scene_name, profile, max_concurrency);
	}

	void WriteNote(const std::string& file_name, const std::string& note)
	{
		VX_ASSERT_WARN_RETURN(!file_name.empty() && !note.empty());
		std::ofstream file(RelativeDir() + "/" + file_name + ".txt");
		file << note;
	}

	std::string RelativeDir()
	{
		EnsureSessionDir();
		return std::string(kParentFolder) + "/" + mSessionName;
	}
private:
	void MakeSessionName()
	{
		auto _now = std::chrono::system_clock::now();
		std::time_t t = std::chrono::system_clock::to_time_t(_now);

		std::tm time_info{};
		localtime_s(&time_info, &t);

		//"Session_22-11-26_2230"
		char buff[16] = "Session_";
		std::strftime(buff, sizeof(buff), "%d-%m-%Y_%H%M", &time_info);
		mSessionName = std::string("Session_") + buff;
	}
	std::string mSessionName;
};




static void ExportProfilingDataToCSV(const std::string& file_name, std::uint32_t active_threads, std::uint32_t iterations, std::uint32_t constraint_count)
{
	const auto& profiles = vx::Profiler::ProfilerCollector::Instance().CopyProfiles();

	const char* simulation_step_key = "Simulation Step";

	const char* broadphase_integration_key = "Broadphase acceleration Integration";

	const char* broadphase_key = "Broadphase Computing Pairs";

	const char* intergrate_bodies_acc_key = "Integrate bodies acceleration";
	const char* narrowphase_island_building_key = "Narrowphase Process Pair and Island Building";

	const char* split_island_key = "Splitting Islands";

	const char* velocity_solving_key = "Solving Velocity Constraints";
	const char* position_solving_key = "Resolve Position Correction";

	auto get_stats = [&](const char* key) -> vx::Profiler::ProfileData {
		auto it = profiles.find(std::string_view(key));
		if (it != profiles.end())
			return it->second;
		return vx::Profiler::ProfileData{};
	};


	auto sim_step_stats = get_stats(simulation_step_key);

	auto broadphase_intergration_stats = get_stats(broadphase_integration_key);
	
	auto broadphase_stats = get_stats(broadphase_key);

	auto integrate_bodies_acc_stats = get_stats(intergrate_bodies_acc_key);
	auto narrowphase_island_stats = get_stats(narrowphase_island_building_key);

	auto split_island_stats = get_stats(split_island_key);

	auto velocity_solving_stats = get_stats(velocity_solving_key);
	auto position_solving_stats = get_stats(position_solving_key);

	std::ofstream csv_file(file_name, std::ios::app);

	if (!csv_file.is_open()) return;

	csv_file.seekp(0, std::ios::end);
	if (csv_file.tellp() == 0)
	{
		csv_file << "Threads_P,Iterations_Niter,TotalConstraints,"
			<< "SimulationStep_AvgMs,SimulationStep_MaxMs,"
			<<"BroadphaseAccIntegration_AvgMs,BroadphaseAccIntegration_MaxMs,"
			<<"Broadphase_AvgMs,Broadphase_MaxMs,"
			<<"IntegrateBodies_AvgMs, IntegrateBodies_MaxMs,"
			<< "BinningPacking_O_Ck_AvgMs,BinningPacking_O_Ck_MaxMs,"
			<<"NarrowphaseIsland_AvgMs, NarrowphaseIsland_MaxMs,"
			<<"Solver_Parallel_AvgMs,Solver_Parallel_MaxMs,"
			<<"Position_AvgMs,Position_MaxMs\n";
	}


	csv_file << active_threads << ","
		<< iterations << ","
		<< constraint_count << ","
		<< sim_step_stats.ArithmeticAvg() << "," << sim_step_stats.maxMs << ","
		<< broadphase_intergration_stats.ArithmeticAvg() << "," << broadphase_intergration_stats.maxMs << ","
		<< broadphase_stats.ArithmeticAvg() << "," << broadphase_stats.maxMs << ","
		<< integrate_bodies_acc_stats.ArithmeticAvg() << "," << integrate_bodies_acc_stats.maxMs << ","
		<< split_island_stats.ArithmeticAvg() << "," << split_island_stats.maxMs << ","
		<< narrowphase_island_stats.ArithmeticAvg() << "," << narrowphase_island_stats.maxMs << ","
		<< velocity_solving_stats.ArithmeticAvg() << "," << velocity_solving_stats.maxMs << ","
		<< position_solving_stats.ArithmeticAvg() << "," << position_solving_stats.maxMs << "\n";

	csv_file.close();
}



static void ExportSortedLoadBalancingToCSV(const std::string& file_name, const char* scene_name, const vx::VelocitySolveProfile& profile, uint32 max_concurrency)
{

	if (profile.sampleCount == 0)
		return;

	std::ofstream csv_file(file_name, std::ios::app);

	if (!csv_file.is_open()) return;

	csv_file.seekp(0, std::ios::end);
	if (csv_file.tellp() == 0)
	{
		csv_file <<
			"Scene," <<
			"Rank_0_Max";

		for (uint32 i = 1; i < (max_concurrency - 1);++i)
			csv_file << ",Rank_" << std::to_string(i);

		csv_file << ",Rank_" << std::to_string(max_concurrency - 1) << "_Min\n";
	}



	csv_file << "Jacobian Solved" << ",";
	for (uint32 i = 0; i < max_concurrency; ++i)
		csv_file << profile.total_jacobian_solved[i] << (i == (max_concurrency - 1) ? "\n" : ",");

	csv_file << "Average Jacobian Row Processed" << ",";
	for(uint32 i = 0; i < max_concurrency; ++i)
		csv_file << profile.total_jacobian_solved[i] / profile.sampleCount << (i == (max_concurrency - 1) ? "\n" : ",");


	csv_file << "Contribution Sample Count" << ",";
	for (uint32 i = 0; i < max_concurrency; ++i)
		csv_file << profile.contributionSampleCount[i] << (i == (max_concurrency - 1) ? "\n" : ",");

	csv_file << "Contribution Ratio" << ",";
	for (uint32 i = 0; i < max_concurrency; ++i)
		csv_file << ((profile.contributionSampleCount[i] != 0) ? (profile.total_jacobian_solved[i] / profile.contributionSampleCount[i]) : 0)  << (i == (max_concurrency - 1) ? "\n" : ",");

	csv_file.close();
}





#include "Vortrix/SimulationStats.h"




static void ExportProfilingDataToCSV(
	const std::string& file_name, std::uint32_t active_threads,
	std::uint32_t iterations, const SolverWorkloadStats& work_load
	)//,bool split_large_island, std::uint32_t large_island_threshold)
{
	const auto& profiles = vx::Profiler::ProfilerCollector::Instance().CopyProfiles();

	const char* simulation_step_key = "Simulation Step";

	const char* broadphase_integration_key = "Broadphase acceleration Integration";

	const char* broadphase_key = "Broadphase Computing Pairs";

	const char* intergrate_bodies_acc_key = "Integrate bodies acceleration";
	const char* narrowphase_island_building_key = "Narrowphase Process Pair and Island Building";

	const char* split_island_key = "Splitting Islands";

	const char* velocity_solving_key = "Solving Velocity Constraints";
	const char* position_solving_key = "Resolve Position Correction";

	auto get_stats = [&](const char* key) -> vx::Profiler::ProfileData {
		auto it = profiles.find(std::string_view(key));
		if (it != profiles.end())
			return it->second;
		return vx::Profiler::ProfileData{};
		};


	auto sim_step_stats = get_stats(simulation_step_key);

	auto broadphase_intergration_stats = get_stats(broadphase_integration_key);

	auto broadphase_stats = get_stats(broadphase_key);

	auto integrate_bodies_acc_stats = get_stats(intergrate_bodies_acc_key);
	auto narrowphase_island_stats = get_stats(narrowphase_island_building_key);

	auto split_island_stats = get_stats(split_island_key);

	auto velocity_solving_stats = get_stats(velocity_solving_key);
	auto position_solving_stats = get_stats(position_solving_key);

	std::ofstream csv_file(file_name, std::ios::app);

	if (!csv_file.is_open()) return;

	csv_file.seekp(0, std::ios::end);
	if (csv_file.tellp() == 0)
	{
		csv_file
			<< "Threads_P,"
			<< "Iterations_Niter,"
			<< "SplitLargeIsland,"
			<< "IslandCount_M,"
			<< "TotalConstraints,"
			<< "MaxIslandConstraints_Ck,"
			<< "NormalIslandCount,"
			<< "LargeIslandCount,"
			<< "SimulationStep_AvgMs,"
			<< "SimulationStep_MaxMs,"
			<< "Broadphase_AvgMs,"
			<< "Broadphase_MaxMs,"
			<< "Integration_AvgMs,"
			<< "Integration_MaxMs,"
			<< "NarrowphaseIsland_AvgMs,"
			<< "NarrowphaseIsland_MaxMs,"
			<< "Splitting_AvgMs,"
			<< "Splitting_MaxMs,"
			<< "Solver_AvgMs,"
			<< "Solver_MaxMs,"
			<< "Position_AvgMs,"
			<< "Position_MaxMs,"
			<< "\n";
	}


	csv_file 
		<< active_threads << ","
		<< iterations << ","
		<< (work_load.splitLargeIsland ? 1 : 0) << ","

		<< work_load.islandCount << ","
		<< work_load.totalConstraints << ","
		<< work_load.maxIslandConstraint << ","
		<< work_load.normalIslandCount << ","
		<< work_load.largeIslandCount << ","

		<< sim_step_stats.avgMs << "," 
		<< sim_step_stats.maxMs << ","

		<< broadphase_intergration_stats.avgMs << ","
		<< broadphase_intergration_stats.maxMs << ","

		<< integrate_bodies_acc_stats.avgMs << ","
		<< integrate_bodies_acc_stats.maxMs << ","

		<< narrowphase_island_stats.avgMs << ","
		<< narrowphase_island_stats.maxMs << ","

		<< split_island_stats.avgMs << ","
		<< split_island_stats.maxMs << ","

		<< velocity_solving_stats.avgMs << ","
		<< velocity_solving_stats.maxMs << ","

		<< position_solving_stats.avgMs << ","
		<< position_solving_stats.maxMs 
		<< "\n";

	csv_file.close();
}





static void ExportSolverLoadBalancingCSV(
	const std::string& file_name,
	uint32 active_threads, uint32 iterations,
	const vx::VelocitySolveProfile& profile,
	uint32 max_concurrency)
{
	std::ofstream csv_file(file_name, std::ios::app);

	if (!csv_file.is_open()) return;

	csv_file.seekp(0, std::ios::end);
	if (csv_file.tellp() == 0)
	{
		csv_file
			<< "Threads_P,"
			<< "Iterations_Niter";

		for (uint32 i = 0; i < max_concurrency; ++i)
			csv_file << ",Rank_" << i;

		csv_file << ",LoadBalanceEfficiency\n";
	}


	if (profile.sampleCount == 0)
		return;

	csv_file
		<< active_threads << ","
		<< iterations;

	///uint64 total_rows = 0;

	//csv_file << "JacobianRowsAverage";
	for (uint32 i = 0; i < max_concurrency; ++i)
	{
		const double avg =
			(profile.sampleCount > 0) ?
			double(profile.total_jacobian_solved[i]) /
			double(profile.sampleCount) : 0.0;

		csv_file << "," << avg;
	}
	//csv_file << "\n";

	/*csv_file << "RankContributionSamples";
	for (uint32 i = 0; i < max_concurrency; ++i)
		csv_file << "," << profile.contributionSampleCount[i];
	csv_file << "\n";

	csv_file << "RankContributionAverage";
	for (uint32 i = 0; i < max_concurrency; ++i)
	{
		const double avg =
			(profile.contributionSampleCount[i] > 0) ?
			double(profile.total_jacobian_solved[i]) /
			double(profile.contributionSampleCount[i]) : 0.0;

		csv_file << "," << avg;
	}
	csv_file << "\n";*/


	///load efficiency
	//csv_file << "Load Efficiency" << ",";
	const double avg_eff = profile.totalLoadBalanceEff / profile.sampleCount;
	csv_file << "," << avg_eff << "\n";

	//for (uint32 i = 0; i < max_concurrency; ++i)
	//	csv_file << ",";

	//csv_file << "\n";

	csv_file.close();
}