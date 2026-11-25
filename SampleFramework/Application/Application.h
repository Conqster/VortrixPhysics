#pragma once

#include "SampleFramework/Display/ApplicationWindow.h"
#include "SampleFramework/Renderer/Renderer.h"
#include "SampleFramework/Camera.h"


#include "SampleFramework/EditorUI.h"

#include "Vortrix/Collision/Shapes/Shape.h"

#include "SampleFramework/ApplicationUtil.h"

#include "Vortrix/Visuals/RenderSettings.h"

#include "Vortrix/PhysicsWorldSettings.h"

class PhysicsAppSetting
{
public:
	struct State
	{
		float frameTime = 0.0f;
		int subSteps = 0;
		float accumulator = 0.0f;
		int maxAttainedSubStep = 0;
		float maxAttainedTime = 0.0f;

		bool paused = false;
		int multipleStep = 0;//nSteps;

		int nStep = 5;

		float frameTimeEmaAvgMs = 0.0f;
	};

	void UpdateStepTimeEmaAvg(float physics_step_time)
	{
		mState.frameTimeEmaAvgMs = 
			mState.frameTimeEmaAvgMs * 0.9f + physics_step_time * 0.1f;
	}

	int Rate() const { return mFrequency; }
	float FixedTimeStep() const { return mFixedTimeStep; }

	const State& StateStats() const { return mState; }
	State& StateStats() { return mState; }

	void Pause(bool v) { mState.paused = v; }

	void SetFrequency(int hz)
	{
		mFrequency = hz;
		mFixedTimeStep = 1.0f / static_cast<float>(hz);
	}

	bool AllowStep()
	{
		bool single_step = mState.multipleStep > 0; //(bool)(mState.nSteps);
		bool allow = !(mState.paused && !single_step);
		mState.paused |= single_step;
		mState.multipleStep -= (mState.multipleStep > 0) ? 1 : 0;
		return allow;
	}

	int mMaxSubStep = 3;
private:
	int mFrequency = 60.0f;
	float mFixedTimeStep = 1.0f / 60.0f;

	State mState;
};

struct DirectionDebugProperties
{
	vx::Vec3 worldRefOrigin = vx::Vec3(0, 1.0f, 0);

	vx::Float3 viewPosition{ 1.0f, 3.5f, 3.3f };
	float halfSize = 2.0f;
	vx::Float3 viewTarget{ 0.0f, 1.1f, 0.0f };
	float arrowWidth = 0.1f;

	float arrowHeight = 0.2f;
	float lightGizmosBaseOffset = 0.35f;

	bool enableDraw = true;

	bool dirty = false;
};



//forward declares
namespace InputSystem {
	class EventHandler;
}
namespace vx {
	namespace Particles {
		class ParticleWorld;
		class Particle;
	}
	class Body;
	class PhysicsWorldSettings;
	class PhysicsWorld;
	//class DebugGizmosRenderer;
}


enum class EScenarioObjectType
{
	Ragdoll,
	Jenga,
	BoxPyramid
};
 
class Scenario;

class Application
{
private:
	ApplicationWindow mWindow;


	char mApplicationLaunchTime[16];

	struct ProfilingCaptureSettings
	{
		vx::uint32 captureEveryPhysicsStep = 120;

		vx::StackString<64> name;

		//vx::uint32 threadIdxSequence = 0;
		vx::uint32 threadCountSequence = 0;

		vx::uint32 iterationCountPerSequence = 4;

		bool reloadPhysicsEveryThreadSequence = true;

		bool closeAppOnComplete = false;
		bool pauseOnComplete = true;

		bool triggerNewCapture = false;

	//private:
		//optimise later
		//vx::StackString<128> filename = vx::StackString<128>("Logs/");
	};

	ProfilingCaptureSettings mProfilingCaptureSettings;
	vx::Scope<class ProfilingSession> mProfilingSession;

	//Remove later
	RendererImpl mRenderer;
	vx::DebugGizmosRenderer* mDebugGizmos = nullptr;

	Camera mCamera;

	//Pointer to global static event handler
	InputSystem::EventHandler* mPtrInputEventHandle;


	//class VPHX::ParticleWorld* mParticleWorld = nullptr;
	vx::Particles::ParticleWorld* mParticleWorld = nullptr;
	vx::PhysicsWorldSettings* mPhysicsWorldSettings = nullptr;
	vx::DrawSettings mPhysicsDrawSettings;
	vx::PhysicsWorld* mPhysicsWorld = nullptr;

	class ScenarioCatergory// : public vx::NonCopyable
	{
	public:
		const char* name;
		std::vector<ScenarioCatergory> catergories;
		std::vector<vx::Scope<Scenario>> scenarios;
	};
	ScenarioCatergory mScenarioCatergoies;
	Scenario* mCurrScenario = nullptr;
	class LoadedFromDiskScenario* mLoadedFromDiskScenario = nullptr;

	float mRunningScenarioDuration = 5.0f;
	float mCurrentScenarioDuration = 0.0f;
	std::vector<Scenario*> mPendingRunScenarios;
	void LoadScenarioWindow();
	void ScenariosInspectionPanel_Details();
	void ProgramWindowStateInfoWindow();

	void DebugDrawWindow(bool* p_open);

	void ProfilingCaptureWindow_WIP();

	void TempPhysicsSettings(bool* p_open);

	vx::RenderSettings mPhysicsRenderSettings{};

	EditorUI mUI;

public:
	Application() = delete;
	Application(const ApplicationSpecification& app_spec);

	virtual ~Application();

	//main loop
	void Run();
	void ResetWorld(bool& reset_flag);
	[[nodiscard]] vx::PhysicsWorld* CreatePhysicsWorld(vx::PhysicsWorldSettings* setting);
	void SetupScenario(Scenario& scenario);
	void ResetParticleWorld(bool& reset_flag);

private:

	void PhysicsStep(double frame_dt);

	/// 
	void CapturePhysicsSubstepVelocitySolverProfile();

	bool bFailLaunch = false;

	struct PhysicsDebugState
	{
		bool keepSettingsOnReset = true;
		bool triggerReset = false;

		bool triggerParticleReset = false;

		vx::Vec3 spawnObjOrigin;
		vx::Quat spawnOrientation;
	}mPhysicsDebugState;
	//bool bIsPhysicsStepPaused = false;

	double mFrameDeltaTime = 0.0f;
	double mLastFrameTime = 0.0f;
	double mMaxAttainedFrameDeltaTime = 0.0f;
	float mTimeScale = 1.0f;

	void UpdateCamera(float dt);
	void ResetCamera();

	void CheckInputs();
	bool CheckInputsBlocked();


	void Quit();

private:
	void OnRenderer();
	void OnApplicationDebugDraw();
	void SubmitRenderObjects();

	void OnExpandDrawScenarioCatergory(const ScenarioCatergory& catergory);
	
	
	void OnDrawImGuiOverlays();

	void SimulationDetailOverlay();
	void ApplicationDetailPanel(bool* p_open);

	void OnDrawBodiesOverlays();

	void DrawProfilePanel();
	void CreateNewPhysicsBodyWindow();
	void DrawUIRendererResourcesPanel(bool* p_open);

	void PhysicsIslandCoordImGuiWindow();
	/// window tab of PhysicsIslandCoordImGuiWindow
	void PhysicsIslandCoordBuilderTab();
	/// window tab of PhysicsIslandCoordImGuiWindow
	void PhysicsIslandCoordSplitterTab();

	void SolverBodyPhysicsBodyWindow();

	void CreateConstraintsWindow();

	void CreateNewCustomPhysicsObject();

	void DrawUI_LightingPanel();
	void PhysicsInteraction();

	void DrawHelpWindow(bool& p_open);

	void DrawBVHNodesOverlay(bool& open);

	ExternalEffectDynamicBodyInfo mExternalEffectDynamicBodyInfo;
	void DrawApplyExternalEffectDynamicBody(bool* p_open);

	AppCameraConfig mCameraConfig;

	//Test Util
	//float mMaxRenderTime = 0.0f;



private:

	struct
	{
		bool enable = true;
		bool appHelpWindow = false;
		bool activeBodiesList = false;
		bool islandCoord = false;
		bool applyExternalEffectOnBodies = false;
		bool scenariosInspection = false;
		bool loadScenarioWindow= false;
		bool createNewCustomPhysicsObject = false;
		bool createConstraints = false;
		bool showSolverBodyPhysicsBodyWindow = false;
		bool profilingCaptureWindow = false;
		bool programWindowStateInfoWindow = false;
	}mPhysicsUIPanels;


	double mPhysicsStepDuration;

	struct CustomPhysicsObject
	{
		vx::Ref<Texture> previewTexture;
		EScenarioObjectType mObjectType;
	};

	std::vector<CustomPhysicsObject> mCustomPhysicsObjs;

	//vx::BodyID mHighlightingBody = vx::BodyID();

	PhysicsAppSetting mPhysicsAppSetting;

	CreatePhysicsObjectSettings mNewPhyObjectSettings{};
	vx::Particles::Particle* mLastAnchor = nullptr; 

	DebugAngularImpulse mDebugAngularImpulse;
	bool bShowDebugRotation = false;


	struct {
		std::vector<vx::Vec3> positions;
		vx::Float3 euler{ 0 };
		vx::Vec3 halfExtent;
	}mSampleStructure;


	AppCreateConstraint mAppCreateConstraint;


	struct SimSelectedBody {
		vx::BodyID mBody;
		bool mEditingForce = false;
		vx::Vec3 mForceDir;
		void Reset() {
			mBody = vx::BodyID();
			mEditingForce = false;
			mForceDir.ToZero();
		}
	}mSimSelectedBody;

	std::vector<vx::RefConst<vx::Shape>> mCreatedShape;
	vx::RefConst<vx::Shape> TryGetCreatedShape(const vx::Float3& he, float density, const vx::EShapeType shape_type) const;

	std::string mAppName = "";
	DirectionDebugProperties mLightDebugProps;
};

Application* CreateApplication(const ApplicationSpecification& app_spec);