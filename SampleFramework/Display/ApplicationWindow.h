#pragma once

#include <vector>
#include <functional>
#include <string>

struct WindowSpecification
{
	int windowSize[2] = { 1920, 1080 };
	int windowPos[2] = { 100, 100 };
	bool disableBindlessSupport = false;
	
	/// glfw recommendation 
	/// 16x16, 32x32, 48x48
	int iconWidth = 32;
	int iconHeight = 32;
	unsigned char* iconPixel = nullptr;

	bool centerWindow = true;
	bool maximise = false;
};

namespace vx {
	class Vec2;
}

//change name later
struct GLFWwindow;
class ApplicationWindow
{
public:
	ApplicationWindow() = default;
	bool Init(const char* base_name, const WindowSpecification& win_spec, bool full_screen = false, const char* name_detail = "");

	GLFWwindow* BackendWindow_Ptr() { return mWindow; }
	unsigned int Width() { return  mWindowWidth; }
	unsigned int Height() { return mWindowHeight; }
	float AspectRatio() {return (float)mWindowWidth / (float)mWindowHeight;}

	void ToggleLockCursor();
	inline bool const GetLockCursor() const { return mLockCursor; }

	void ChangeWindowTitle(const char* name);
	const char* FullTitle() const;
	inline const char* BaseTitle() const { return mBaseTitle.c_str();}

	inline bool GetVSync() const { return mVSync; }
	void const SetVSync(bool value);

	static bool SupportsBindless() { return sSupportsBindless; }

	vx::Vec2 MouseCursorPosition() const;
	vx::Vec2 ComputePosition() const;

	void FlushAndSwapBuffer();
	void SwapBuffer() const;
	void PollEvents() const;
	bool ProgramActive() const;
	bool Minimised() const { return mMinimised; }
	bool Maximise();
	bool Maximised() const;
	void Close();
	void Destroy() const;

	int ComputeWindowCurrentMonitorIdx() const;

	void FullScreen(bool value);
	bool IsFullScreen() const;

	using WindowResizeListener = std::function<void(uint32_t, uint32_t)>;
	void SetWindowResizeListener(const WindowResizeListener& resize_func) { mWindowResizeListener = resize_func; }
private:
	GLFWwindow* mWindow = nullptr;
	std::string mBaseTitle = "";

	static unsigned int mWindowWidth;
	static unsigned int mWindowHeight;

	//unsigned int mWindowPos[2] = { 100, 100 };

	bool mLockCursor = false;
	bool mVSync = true;
	bool mMinimised = false;

	bool CreateDisplayWindow(const char* name, bool full_screen = false);
	bool InitGraphicsInterface();


	static void OnWindowResizeCallback(GLFWwindow* window, int width, int height);
	static void OnWindowPosCallback(GLFWwindow* window, int x, int y);
	static void OnWindowMinimisedCallback(GLFWwindow* window, int iconified);

	WindowResizeListener mWindowResizeListener;

	static bool sSupportsBindless;
};
