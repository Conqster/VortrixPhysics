#pragma once

#include <Vortrix/Vortrix.h>

#include "Vortrix/Dynamics/ConstraintSolver/CombineFrictionRestitution.h"


#include "Vortrix/Core/Colours.h"


namespace vx
{

	//TODO(Jay): Later have flag 
	//enum class DebugDrawFlag : uint32
	//{
	//	None	= 0,
	//	GetShape	= Bit32(0),
	//	Corners = Bit32(1),
	//	Normals = Bit32(2),
	//	AABB	= Bit32(3),
	//	Inertia = Bit32(4)
	//};


	struct PhysicsStepContext
	{
		Vec3 mGravity = Vec3(0.0f, -9.81f, 0.0f);

		class BodyManager* bodyManager = nullptr;

		const struct PhysicsWorldSettings* mSettings = nullptr;

		uint64 mStepIndex = 0;

		float mDeltaTime = 1.0f / 60.0f;

		class ScratchAllocator* mScratchAllocator = nullptr;
		class IslandCoordinator* mIslandCoordinator = nullptr;

		/// rebuild BVH, when imbalance ration grows
		/// above treshold
		//float rebuildBVH_ImbalanceRatioTreshold = 0.6f;
		//uint32 maxBroadphasePair = 10240;

		uint32 contactConstraintCount = 0;
		uint32 nonContactConstraintCount = 0;
		uint32 nonContactJacobianRowCount = 0;

		//bool forceBVHRebuild = true;

		//bool BVH_rebuild_SAH = false;

		class ConstraintSolver* constraintSolver = nullptr;

		class DebugGizmosRenderer* mDebugRenderer = nullptr;

		bool enableContact = true;
		uint32 velocityIterations = 10;
		///hack for now
		class PhysicsWorld* mPhysicsWorld = nullptr;
		struct DrawSettings* mDrawSettings = nullptr;
	};


	enum class EBodyColourMode : int8
	{
		Instances,		/// random per instance
		MotionState,	/// Dynamic -> yellow/oragne ; Sleeping -> dull red ; Static -> grey
		MotionType,		/// Dynamic -> yellow ; Static -> grey ; Kinematic blue
		ShapeType,		/// for now - complexity 0 --> x; plane grey (not low but rarely used), Sphere Cyan, Capsule Light green, Box yellow, {Cylinder Orange}
		Collision,		/// Colliding(dyn-dyn/dyn-static) / not
		Phase,			/// Broad / Narrow / Colliding convert this to heat based
		MotionMaxLinearVelocity,
		MotionMaxAngularVelocity,
		IslandIdx,
		IslandConstraintGroup,
	};

	static constexpr const char* BodyColourModeLabels = 
		"Instance\0"
		"MotionState\0"
		"MotionType\0"
		"ShapeType\0"
		"Collision\0"
		"Phase\0"
		"MotionMaxLinearVelocity\0"
		"MotionMaxAngularVelocity\0"
		"IslandIdx\0"
		"IslandConstraintGroup\0"
		"\0";

	/// BodyColour 
	/// Motion type
	/// GetShape Type
	/// Sleep -> red & activate yellow
	/// Island ->? different island have different colour
	/// Material 
	/// Instance: random colour


	/// later split as it getting huge
	/// and must info is not used but subsystems
	/// 
	/// split option
	/// seperate settings (physics sub systems and draw setting) as draw is called at end of frame
	/// 2 PhysicsWorldSettign still contains every info but in blobs of structs 
	/// physicsworld info
	/// draw setting
	/// 
	/// or 
	/// collision info 
	/// dynamic info  
	/// draw info
	/// 
	/// 
	/// Later group the booleans 
	/// like Motion_properties, bodies etc and make use of enums
	/// 
	struct CollisionSettings
	{
		float boundsMargin = 0.1f; //10cm 

		uint32 maxPairs = 10240;

		bool forceBVHRebuildEveryStep = false;
		bool rebuildSAH = true;

		/// rebuild BVH, when imbalance ration grows, above treshold
		float rebuildBVHimbalanceRatioTreshold = 0.6f;
	};

	struct SleepingSettings
	{
		bool enable = true;
		//to raise above gravity per step (0.163)
		float velocityThreshold = 0.10f;// 0.05f;// 0.17f;// 0.5f;		//Below this, velocity = resting
		float angularThreshold = 0.20f;// 0.05f;// 0.08f; //0.5f;
		float timeThreshold = 0.5f;// 0.25f;				// Time in seconds before sleep
	};

	struct SolverSettings
	{
		int velocityIterations = 9;

		bool enable = true;
		bool enableContact = true;
		bool warmstart = true;
		float restitutionThreshold = 1.0f; //0.2f - 0.5f
		float frictionThreshold = 0.02f;
		//6 - 10
		//2 - 4
		int positionIterations = 3; //2
		//position correction
		//allow bodies to sink into each other
		float positionCorrectionSlop = 0.02f;// 0.01f;
		float baumgarte = 0.2f;// 0.2f;
		Vec2 positionCorrectionGlobalLimits = Vec2(0.01, 4.0f);
		float positionCorrectionBodyLimitScale = 0.8f;

		ECombineMode frictionCombineMode = ECombineMode::SquareRoot;
		ECombineMode restitutionCombineMode = ECombineMode::Maximum;
	};


	struct NonContactConstraintDrawSettings
	{
		bool drawConstraints = true;
		bool drawConstraintBounds = false;
		bool drawActiveBounds = false;
		bool drawVelocitySolveBounds = false;
		bool drawPositionSolveBounds = false;
		float anchorSize = 0.085f;
	};


	struct WorldQuerySettings
	{
		bool mDebugTreeWalk = false;
	};



	enum EBroadphaseDrawFlag : uint8
	{
		None = 0,
		InternalNodes = Bit8(0),
		LeafNodes = Bit8(1),
		LeafBounds = Bit8(2),

		All = InternalNodes | LeafNodes
	};

	inline EBroadphaseDrawFlag operator|(EBroadphaseDrawFlag lhs, EBroadphaseDrawFlag rhs)
	{
		using underlying_t = std::underlying_type_t<EBroadphaseDrawFlag>;
		return static_cast<EBroadphaseDrawFlag>(static_cast<underlying_t>(lhs) | static_cast<underlying_t>(rhs));
	}

	inline EBroadphaseDrawFlag operator|=(EBroadphaseDrawFlag& lhs, EBroadphaseDrawFlag rhs)
	{
		lhs = lhs | rhs;
		return lhs;
	}

	//to remove a flag NOT
	constexpr EBroadphaseDrawFlag operator ~(EBroadphaseDrawFlag flag) { return EBroadphaseDrawFlag(~vx::uint32(flag)); }
	constexpr EBroadphaseDrawFlag operator ^(EBroadphaseDrawFlag lhs, EBroadphaseDrawFlag rhs)
	{
		return EBroadphaseDrawFlag(uint32(lhs) ^ (rhs));
	}

	constexpr EBroadphaseDrawFlag& operator ^= (EBroadphaseDrawFlag& io_lhs, EBroadphaseDrawFlag rhs)
	{
		io_lhs = io_lhs ^ rhs;
		return io_lhs;
	}


	VX_INLINE bool Contains(EBroadphaseDrawFlag dof, EBroadphaseDrawFlag flag)
	{
		return (uint8(dof) & uint8(flag)) != 0;
	}

	struct DrawSettings
	{
		/// broadphase
		//bool drawBVHNodes = false;
		//bool drawOnlyBVHLeafNodeBounds = false;
		EBroadphaseDrawFlag braodphaseFlags = EBroadphaseDrawFlag::None;

		/// narrowphase
		bool drawContacts = false;
		bool drawContactsNormals = true;
		bool drawPenetrationText = false;
		Colour drawContactPointColour = Colour::sGreen;
		Colour drawContactNormalsColour = Colour::sRed;// Colour::sYellow;
		float drawContactPointSize = 0.15f;
		float drawContactNormalSize = 1.0f;
		bool drawContactsNormalsWithPeneration = true;
		bool drawContactsScaleNormalPeneration = false;
		bool drawCollidingPairRefAndInc = false;
		Colour collidingPairRefColour = Colour::sOrange;
		Colour collidingPairIncColour = Colour::sYellow;

		/// SIMULATION
		/// 
		/// solver
		bool drawContactConstraintSolverTBNs = false;
		bool scaleAxesWithDepth = true;
		float drawContactConstraintSolverTBNsScale = 1.0f;
		/// island 
		bool drawPerIslandConstraintGroup = false;
		bool drawPerIslandConstraintGroupPoint = true;
		bool drawPerIslandConstraintGroupPlane = false;
		float drawPerIslandConstraintGroupPointSize = 0.15f;
		bool drawPerIslandConstraintGroupPointAsWireframe = true;
		float drawPerIslandConstraintGroupColourAlpha = 1.0f;
		
		/// bodies
		EBodyColourMode bodyColourMode = EBodyColourMode::Instances;
		//bool drawBounds = false; //for drawAABB, drawOBB
		bool drawAABB = false;
		bool drawOBB = false;
		bool drawBodiesAsSolid = true;
		bool drawBodiesPrincipalAxes = false;
		bool drawShapeOrientedBoundCorners = false;
		bool drawDebugInertia = false;
		Colour drawShapeCornersColour = Colour::sBlue;
		Colour drawDebugInertiaColour = Colour::sOrange;
		bool drawBodiesVelocities = false;
		bool drawBodiesMassText = false;

		float bodiesDrawColourAlpha = 1.0f;

		bool drawAABBContactManifoldInFrame = false;
		bool drawAABBContactManifoldInFrameWcPlane = true;



		///world query 
		bool drawWalkedTreeQuery = false;

		///joint constraint 
		NonContactConstraintDrawSettings nonContactConstraintDrawSettings{};

		/// Later add option for drawing axis orientation or not
		/// also with new axis draw

		// === COLOUR SCHEME ===
		/// Dynamic -> yellow / Sleeping -> dull red / Static -> grey
		Colour dynamicColour = Colour::sOrange; //yellow
		Colour staticColour = Colour(0.5f);//Colour(0.2f, 0.8f, 0.2f, 1.0f); //red
		Colour sleepingColour = Colour(0.8f, 0.2f, 0.2f);//dull red
		//Colour kinematicColour = Colour(0.5f, 0.6f, 1.0f);//cyan blue

		Colour groundColour = Colour((uint8)42, (uint8)175, (uint8)206);

		Colour contactColour = Colour::sWhite; //<-- by default contact does not generate colour 
		Colour contactWireColour = Colour(1.0f, 0.2f, 0.2f); //light red
		Colour shapeColliderWireColour = Colour(0.0f, 1.0f, 1.0f);//cyan wireframe

		Colour aabbColour = Colour(0.0f, 1.0f, 1.0f, 0.4f);//transparent cyn
		Colour bvhNodeColour = Colour(0.7f, 0.3f, 1.0f, 0.5f);//soft magenta

		//body sim phase colours
		///idle / passed broad phase 
		/// narrowphase check 
		/// collidinf 
		/// deep penetration
		Colour dynamicStaticCollisionColour = Colour(Vec3(0.2f, 0.8f, 0.2f) * 0.5f);
		Colour narrowPhaseColour = Colour(1.0f, 0.9f, 0.2f);
		Colour broadPhaseColour = Colour(0.1f, 0.2f, 0.5f);
		Colour neuralPhaseColour = Colour(0.5, 0.5, 0.5, 1.0f);

		Colour bodyLinearVelocityCol = Colour::sTurquoise;
		Colour bodyAngularVelocityCol = Colour::sRed;
	};


	struct PhysicsWorldSettings
	{
		// === GRAVITY ===
		Vec3 gravity = Vec3(0.0f, -9.81f, 0.0f);
		float gravityScale = 1.0f;

		//bool forceBVHRebuild = false;

		// === SOLVER ===
		SolverSettings solver;
		// === SLEEPING ===
		SleepingSettings sleeping;
		CollisionSettings collision;


		/// Later group the booleans 
		/// like Motion_properties, bodies etc and make use of enums
		// === DEBUG DRAW ===
		DrawSettings* drawSettings;

		//for debugging 
		Float3 frameGravityVelocity;


		int maxConcurrency = -1;
		bool splitLargeIsland = true;

		float scratchAllocationMiB = 1.0f;
	};

}