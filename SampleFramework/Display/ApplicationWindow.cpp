#include <SampleFramework/SampleFramework.h>
#include "ApplicationWindow.h"

#include <GL/glew.h>
#include <glfw/glfw3.h>


#include "Vortrix/Core/Profiler.h"

unsigned int ApplicationWindow::mWindowWidth;
unsigned int ApplicationWindow::mWindowHeight;


void QuickErrorCallback(int error, const char* desc)
{
	fprintf(stderr, "GLFW ERROR %d: %s\n", error, desc);
}



//void QuickDebugCallback(GLenum src, GLenum type,
//	GLuint id, GLenum severity,
//	GLsizei length, const GLchar* msg,
//	const void* user_param)
//{
//	if (severity == GL_DEBUG_SEVERITY_NOTIFICATION)
//		return;
//	printf("OpenGL DEBUG: %s\n", msg);
//}

//---------------------------------------------------------------------
#define ANSI_RED_COLOUR "\033[1;31m"	   //4
#define ANSI_GREEN_COLOUR "\033[1;32m"	   //2
#define ANSI_YELLOW_COLOUR "\033[1;33m"	   //3
#define ANSI_BLUE_COLOUR "\033[1;34m"      //1
#define ANSI_RESET_COLOUR "\033[0m"


enum class ESeverityLvl : uint8_t
{
	High = 3,
	Meduim = 2,
	Low = 1,
	Notification = 0
};


constexpr const char* GL_SeverityEnumToText(GLenum severity)
{
	switch (severity)
	{
	case GL_DEBUG_SEVERITY_NOTIFICATION: return "GL_DEBUG_NOTIFICATION";
	case GL_DEBUG_SEVERITY_MEDIUM: return "GL_DEBUG_WARNING";
	case GL_DEBUG_SEVERITY_LOW: return "GL_DEBUG_MINOR_ISSUE";
	case GL_DEBUG_SEVERITY_HIGH: return "GL_DEBUG_ERROR";
	default: return "GL_DEBUG_UNKNOWN";
	}
}

constexpr const char* GL_TypeEnumToText(GLenum type)
{
	switch (type)
	{
	case GL_DEBUG_TYPE_ERROR: return "API or shader error";
	case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: return "Using deprecated features";
	case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR: return "Undefined behaviour";
	case GL_DEBUG_TYPE_PORTABILITY: return "Non-portable use (vendor-specific feature)";
	case GL_DEBUG_TYPE_PERFORMANCE: return "Performance warning";
	case GL_DEBUG_TYPE_OTHER: return "Misc driver message";
	case GL_DEBUG_TYPE_MARKER: return "Debug marker by application";
	case GL_DEBUG_TYPE_PUSH_GROUP: return "Group push event";
	case GL_DEBUG_TYPE_POP_GROUP: return "Group pop event";
	default: return "Unknown";
	}
}


constexpr const char* GL_SourceEnumToText(GLenum src)
{
	switch (src)
	{
	case GL_DEBUG_SOURCE_API: return "OpenGL API";
	case GL_DEBUG_SOURCE_WINDOW_SYSTEM: return "Window system";
	case GL_DEBUG_SOURCE_SHADER_COMPILER: return "shader compiler";
	case GL_DEBUG_SOURCE_THIRD_PARTY: return "Third party/driver";
	case GL_DEBUG_SOURCE_APPLICATION: return "Custom Application";
	case GL_DEBUG_SOURCE_OTHER: return "Misc driver source";
	default: return "Unknown";
	}
}


inline constexpr const char* ANSI_GL_DebugCol(GLenum severity)
{
	switch (severity)
	{
	case GL_DEBUG_SEVERITY_NOTIFICATION: return ANSI_BLUE_COLOUR;
	case GL_DEBUG_SEVERITY_LOW: return ANSI_GREEN_COLOUR;
	case GL_DEBUG_SEVERITY_MEDIUM: return ANSI_YELLOW_COLOUR;
	case GL_DEBUG_SEVERITY_HIGH: return ANSI_RED_COLOUR;
	default: return ANSI_GREEN_COLOUR;
	}
}

bool ApplicationWindow::sSupportsBindless = false;

void APIENTRY ErrorMessageCallback(GLenum source, GLenum type, GLuint id,
	GLenum severity, GLsizei length,
	const GLchar* message, const void* userParam)
{

	std::cout << ANSI_GL_DebugCol(severity) << "GL Debug [" << GL_SeverityEnumToText(severity) <<
		"] (" << GL_SourceEnumToText(source) << ": " << GL_TypeEnumToText(type) <<
		"): " << message << ".\n" << ANSI_RESET_COLOUR;
}

bool ApplicationWindow::Init(const char* base_name, const WindowSpecification& win_spec, bool full_screen, const char* name_detail)
{
	mBaseTitle = base_name;

	//glfwSetErrorCallback(QuickErrorCallback);

	if (!glfwInit())
	//if (!glfw_state)
	{
		VX_LOG_ERROR("Failed to initialise GLFW!!!!!!");
		return false;
	}

	mWindowWidth = win_spec.windowSize[0];
	mWindowHeight = win_spec.windowSize[1];

	int win_pos_x = win_spec.windowPos[0];
	int win_pos_y = win_spec.windowPos[1];

	if (win_spec.centerWindow)
	{
		//get screen size
		int screen_width = GetSystemMetrics(SM_CXSCREEN);
		int screen_height = GetSystemMetrics(SM_CYSCREEN);

		int hw = float(screen_width) * 0.5f;
		int hh = float(screen_height) * 0.5f;

		int win_hw = win_spec.windowSize[0] * 0.5f;
		int win_hh = win_spec.windowSize[1] * 0.5f;

		win_pos_x = hw - win_hw;
		win_pos_y = hh - win_hh;
	}


	if (!CreateDisplayWindow((std::string(base_name) + name_detail).c_str(), full_screen))
		return false;

	VX_LOG_INFO("OpenGL version supported: ", (const char*)(glGetString(GL_VERSION)));

	GLenum GlewInitResult = glewInit();
	if (GlewInitResult != GLEW_OK)
	{
		VX_LOG_ERROR("Glew Init failed, ERROR: ", glewGetErrorString(GlewInitResult));
		glfwDestroyWindow(mWindow);
		glfwTerminate();
		return false;
	}

	bool bindless_tex_was_support = glfwExtensionSupported("GL_ARB_bindless_texture");
	(bindless_tex_was_support) ? VX_LOG_INFO("Device supports bindless_texture") : VX_LOG_INFO("Device doesnt supports bindless_texture");
	sSupportsBindless = (win_spec.disableBindlessSupport) ? false : bindless_tex_was_support;
	VX_ASSERT_WARN(!win_spec.disableBindlessSupport, "Bindless Support was disabled");
	VX_ASSERT_WARN(bindless_tex_was_support, "GL_ARB_bindless_texture not supported");

	int frame_top_egde;
	int frame_left_egde;
	glfwGetWindowFrameSize(mWindow, &frame_left_egde, &frame_top_egde, nullptr, nullptr);
	glfwSetWindowPos(mWindow, win_pos_x + frame_left_egde, win_pos_y + frame_top_egde);



	if(win_spec.iconPixel)
	{
		GLFWimage img;
		img.width = win_spec.iconWidth;
		img.height = win_spec.iconHeight;
		img.pixels = win_spec.iconPixel;
		glfwSetWindowIcon(mWindow, 1, &img);
	}

	glfwSetInputMode(mWindow, GLFW_CURSOR, (mLockCursor) ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);

	//int flags;
	//glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
	//if (flags & GL_CONTEXT_FLAG_DEBUG_BIT)
	//{
	//	glEnable(GL_DEBUG_OUTPUT);
	//	glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
	//	glDebugMessageCallback(ErrorMessageCallback, nullptr);

		glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_NOTIFICATION, 0, nullptr, GL_FALSE);
	//}
	glEnable(GL_DEBUG_OUTPUT);
	glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
	glDebugMessageCallback(ErrorMessageCallback, nullptr);

	if (full_screen != IsFullScreen()) FullScreen(full_screen);
	if (!full_screen && win_spec.maximise) Maximise();

	return true;
}

void ApplicationWindow::ToggleLockCursor()
{
	glfwSetInputMode(mWindow, GLFW_CURSOR, (mLockCursor = !mLockCursor) ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
}

void ApplicationWindow::ChangeWindowTitle(const char* name)
{
	glfwSetWindowTitle(mWindow, name);
}

const char* ApplicationWindow::FullTitle() const
{
	return glfwGetWindowTitle(mWindow);
}

void const ApplicationWindow::SetVSync(bool value)
{
	if (mVSync != value)
	{
		mVSync = value;
		glfwSwapInterval(int(mVSync));
	}
}

vx::Vec2 ApplicationWindow::MouseCursorPosition() const
{
	double x, y;
	glfwGetCursorPos(mWindow, &x, &y);
	return vx::Vec2(x, y);
}

vx::Vec2 ApplicationWindow::ComputePosition() const
{
	int x, y;
	glfwGetWindowPos(mWindow, &x, &y);
	return vx::Vec2(float(x), float(y));
}

void ApplicationWindow::FlushAndSwapBuffer()
{
	glfwSwapBuffers(mWindow);
	glfwPollEvents();
}

void ApplicationWindow::SwapBuffer() const
{
	VX_VARIABLE_PROFILE_FUNCTION();
	glfwSwapBuffers(mWindow);
}

void ApplicationWindow::PollEvents() const
{
	VX_VARIABLE_PROFILE_FUNCTION();
	glfwPollEvents();
}

bool ApplicationWindow::ProgramActive() const
{
	return !glfwWindowShouldClose(mWindow);
}

bool ApplicationWindow::Maximise()
{
	VX_ASSERT_WARN_RETURN(mWindow != nullptr, false, "Non existing window handle to maximise");

	glfwMaximizeWindow(mWindow);
	return true;
}

bool ApplicationWindow::Maximised() const
{
	int maximised = glfwGetWindowAttrib(mWindow, GLFW_MAXIMIZED);
	return (maximised == GLFW_TRUE);
}

void ApplicationWindow::Close()
{
	glfwSetWindowShouldClose(mWindow, true);
}

void ApplicationWindow::Destroy() const
{
	glfwDestroyWindow(mWindow);
	glfwTerminate();
}



bool ApplicationWindow::CreateDisplayWindow(const char* name, bool full_screen)
{
	mWindow = glfwCreateWindow(mWindowWidth, mWindowHeight, name, nullptr, nullptr);

	int num_monitors;
	GLFWmonitor** monitors = glfwGetMonitors(&num_monitors);


	//glfwGetError
	//GLenum GlewInitResult = glewInit();
	//if (GlewInitResult != GLEW_OK)
		//VX_ERROR("Glew Init failed, ERROR: ", glewGetErrorString(GlewInitResult))
	const char* err_desc;
	if (!mWindow)
	{
		glfwGetError(&err_desc);
		VX_LOG_ERROR("Failed to create GLFW Window: ", err_desc);
		glfwTerminate();
		return false;
	}


	glfwMakeContextCurrent(mWindow);

	glfwSwapInterval(int(mVSync));

	glfwSetInputMode(mWindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	glfwSetFramebufferSizeCallback(mWindow, OnWindowResizeCallback);
	glfwSetWindowPosCallback(mWindow, OnWindowPosCallback);
	glfwSetWindowIconifyCallback(mWindow, OnWindowMinimisedCallback);
	glfwSetWindowUserPointer(mWindow, this);


	return true;
}

bool ApplicationWindow::InitGraphicsInterface()
{
	//opengl hint for glfw
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	//glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	return false;
}

int ApplicationWindow::ComputeWindowCurrentMonitorIdx() const
{
	int win_x, win_y;
	glfwGetWindowPos(mWindow, &win_x, &win_y);

	int num_monitors;
	GLFWmonitor** monitors = glfwGetMonitors(&num_monitors);

	size_t best = -1;
	int best_overlap = 0;

	for (size_t i = 0; i < (size_t)num_monitors; i++)
	{
		GLFWmonitor* curr_monitor = monitors[i];
		const GLFWvidmode* mode = glfwGetVideoMode(curr_monitor);
		int mx, my;
		glfwGetMonitorPos(curr_monitor, &mx, &my);

		int width =mode->width, height = mode->height;

		//compute overlap area btw window rect and monitor rect
		int x1 = vx::VxMax(win_x, mx);
		int y1 = vx::VxMax(win_y, my);
		int x2 = vx::VxMin(win_x + int(mWindowWidth), mx + width);
		int y2 = vx::VxMin(win_y + int(mWindowHeight), my + height);

		int overlap = vx::VxMax(0, x2 - x1) * vx::VxMax(0, y2 - y1);
		if (overlap > best_overlap)
		{
			best_overlap = overlap;
			best = i;
		}
	}
	return int(best);
}

void ApplicationWindow::FullScreen(bool value)
{
	if (IsFullScreen() != value)
	{

		///cache old pos & size 
		static int win_x = 0, win_y = 0,
			width = 1280, height = 720;

		if (value == false)
		{
			glfwSetWindowMonitor(mWindow, nullptr,
				win_x, win_y,
				width, height, 0);

			return;
		}

		int curr_monitor_idx = ComputeWindowCurrentMonitorIdx();
		GLFWmonitor* curr_monitor;
		if (curr_monitor_idx == -1)
			curr_monitor = glfwGetPrimaryMonitor();
		else
		{
			int num_monitors;
			GLFWmonitor** monitors = glfwGetMonitors(&num_monitors);
			curr_monitor = monitors[curr_monitor_idx];
		}

		/// cache old data
		glfwGetWindowPos(mWindow, &win_x, &win_y);
		glfwGetWindowSize(mWindow, &width, &height);

		const GLFWvidmode* mode = glfwGetVideoMode(curr_monitor);
		glfwSetWindowMonitor(mWindow, curr_monitor,
			0, 0,
			mode->width, mode->height, mode->refreshRate);
	}
}

bool ApplicationWindow::IsFullScreen() const
{
	GLFWmonitor* monitor = glfwGetWindowMonitor(mWindow);
	return (monitor != nullptr);
}

void ApplicationWindow::OnWindowResizeCallback(GLFWwindow* window, int width, int height)
{
	ApplicationWindow* win = (ApplicationWindow*)(glfwGetWindowUserPointer(window));
	if (!win) return;

	mWindowHeight = height;
	mWindowWidth = width;

	if (width <= 0 || height <= 0)
	{
		win->mMinimised = true;
		return;
	}
	
	win->mMinimised = false;

	if (win->mWindowResizeListener)
		win->mWindowResizeListener(width, height);
}

void ApplicationWindow::OnWindowPosCallback(GLFWwindow* window, int x, int y)
{
	if (ApplicationWindow* win = (ApplicationWindow*)glfwGetWindowUserPointer(window))
	{
		//win->mWindowPos[0] = x;
		//win->mWindowPos[1] = y;

	}
}

void ApplicationWindow::OnWindowMinimisedCallback(GLFWwindow* window, int iconified)
{
	if (ApplicationWindow* win = (ApplicationWindow*)glfwGetWindowUserPointer(window))
		win->mMinimised = (iconified == GLFW_TRUE) ? true : false;
	VX_LOG_INFO("Minimised: ", iconified);
}
