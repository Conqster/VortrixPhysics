#pragma once
#include "Application.h"

#include <SampleFramework/SampleFramework.h>


static void CustomAssertHandler(const char* expr, const char* message,
	const unsigned int lvl, const char* file, unsigned int line, const char* func)
{
	VX_LOG_ERROR("Assertion Failed (",
		expr, ") in Function: ", func, "\nMessage: ",
		((message) ? message : ""),
		".\nFile: ", file,
		".\nLine: ", line, ".\n\n");
}

extern Application* CreateApplication(const ApplicationSpecification& app_spec);


#include <thread>

int main(int argc, char** argv)
{
	VX_LOG_INFO(
		"Usage:\n"
		"--win_size x,y\n"
		"--win_pos x,y\n"
		"--full_screen\n"
		"--maximise_win\n"
		"--min_debug_lvl (Debug, Info, Warning, Error)"
	);

	vx::uint32 num_threads = std::thread::hardware_concurrency();
	VX_LOG_INFO("Number of hardware threads: ", num_threads);


	uint32 full_hd[2] = { 1920, 1080 };
	uint32 hd[2] = { 1280, 720 };

	bool is_full_hd = true;

	uint32* screen_reso = (is_full_hd) ? full_hd : hd;

	ApplicationSpecification app_spec;
	app_spec.name = "Vortrix Physics";
	app_spec.disableBindlessSupport = false;
	app_spec.launchFullScreen = false;
	app_spec.windowSize[0] = screen_reso[0];
	app_spec.windowSize[1] = screen_reso[1];
	app_spec.windowPos[0] = 0;
	app_spec.windowPos[1] = 0;
	app_spec.centralisedWindow = false;
	app_spec.maximiseWindow = false;
	///Parse command line parameter
	///Learning about "int main(int argc, char** argv)"
	for (int arg_idx = 0; arg_idx < argc; ++arg_idx)
	{
		const char* arg = argv[arg_idx];

		if (strncmp(arg, "--full_screen", 14) == 0)
		{
			VX_LOG_WARN("Enable Full Screen");
			app_spec.launchFullScreen = true;
		}
		if (strncmp(arg, "--maximise_win", 15) == 0)
		{
			VX_LOG_WARN("Maximise Window");
			app_spec.maximiseWindow = true;
		}
		///Window size 
		if (strncmp(arg, "--win_size", 11) == 0)
		{
			//might have found something
			//parse window size
			const char* info = arg + 11;
			int x, y;
			if (sscanf(info, "%d,%d", &x, &y) == 2)
			{
				app_spec.windowSize[0] = x;
				app_spec.windowSize[1] = y;
			}
			else
				VX_LOG_INFO("Invalid Format, \n\t Usage --win_size x,y");
		}

		///Window pos
		if (strncmp(arg, "--win_pos", 10) == 0)
		{
			//might have found something
			//parse window size
			const char* info = arg + 10;
			int x, y;
			if (sscanf(info, "%d,%d", &x, &y) == 2)
			{
				app_spec.windowPos[0] = x;
				app_spec.windowPos[1] = y;
			}
			else
				VX_LOG_INFO("Invalid Format, \n\t Usage --win_pos x,y \n\t Captured argv", arg, " ", info);
		}

		if (strncmp(arg, "--disable_gfx_bindless", 23) == 0)
			app_spec.disableBindlessSupport = true;

		if (strncmp(arg, "--centre_screen", 16) == 0)
		{
			VX_LOG_WARN("Enable Centered Screen");
			app_spec.centralisedWindow = true;
		}
	}

	auto app = CreateApplication(app_spec);


	try
	{
		app->Run();
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		delete app;
		return EXIT_FAILURE;
	}

	delete app;
	return EXIT_SUCCESS;
}

