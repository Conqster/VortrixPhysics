#pragma once

#include "Vortrix/Vortrix.h"

#include "Vortrix/Geometry/AABB.h"
#include "Vortrix/PhysicsWorldSettings.h"

#include "BodyID.h"

#include "EBodyDebugFlags.h"

#include "EDynamicsDofs.h"

class EditorUI;
namespace vx
{
	class Shape;

	enum class EMotionType : uint8
	{
		Static = 0,		//never moves
		//Kinematic = 1,	//infinte mass to dynamic object, move via external forces
		Dynamic = 2,	// full physics dynamics
	};
	//constexpr const char* MotionTypeNames = 
	//	"Static\0"
	//	"Kinematic\0"
	//	"Dynamic\0"
	//	"\0";


	inline const char* MotionTypeNames(EMotionType Type)
	{
		if (Type == EMotionType::Static) return "Static";
		else if (Type == EMotionType::Dynamic) return "Dynamic";
		else return "Unknown";
	}


	/// use data to create unique key with data over compare even element, 
	struct BodySettings
	{
		static BodySettings DefaultDynamicConstruct()
		{
			BodySettings t;
			t.intialVelocity = Vec3::Zero();
			t.linearDamping = 0.1f; // 0.05f;
			t.angularDamping = 0.2f;// 0.1f;

			t.mass = 1.0f;

			t.motionType = EMotionType::Dynamic;

			t.maxLinearVelocity = 500.f;//100.0f;
			t.maxAngularVelocity = 47.0f;//35.0f;
			return t;
		}

		static BodySettings DefaultStaticConstruct()
		{
			BodySettings t;
			t.motionType = EMotionType::Static;
			t.density = 0.0f;
			t.mass = 0.0f; //infinte
			return t;
		}

		static Float3 UnitBoxinteriatensor()
		{
			return Float3{ 1.0f / 6.0f };
		}
		static Float3 UnitSphereinteriatensor()
		{
			return Float3{ 2.0f / 5.0f };
		}
		static Float3 UnitCapsuleinteriatensor()
		{
			return Float3{ 0.324f, 0.324f, 0.115f};
		}


		Vec3 position = Vec3::Zero();
		float mass;
		Float3 inertia{0.0f};
		Vec3 impluse = Vec3(0.0f);
		const char* debug_name = nullptr;
		RefConst<Shape> shape = nullptr;

		Vec3 intialVelocity = Vec3(0.0f);
		float linearDamping = 0.0f;
		float angularDamping = 0.0f;

		/// shape density
		float density = 1000.0f;
		/// 
		bool overrideMasses = false;

		float friction = 0.4f; 
		float restitution = 0.2f;// 0.2f;

		Quat orientation = Quat::Identity();

		EMotionType motionType = EMotionType::Dynamic;


		float maxLinearVelocity = 0.0f;
		float maxAngularVelocity = 0.0f;

		/// if in broadphase body participates in collision
		bool inBroadphase = true;

		EDynamicsDofs degreeFreedom = EDynamicsDofs::All;
	};


	struct TransformState
	{

		uint32 lastUpdateStep = 0;

		bool MovedSince(uint32 step) const
		{
			return uint32(lastUpdateStep - step) < uint32(1u << 31);
		}
	};

			//return lastUpdateStep < step;

	//struct TransformState
	//{
	//	static constexpr uint32 kInvalidStep = 0xffffffff;

	//	uint32 lastUpdateStep = kInvalidStep;


	//	bool MovedSince(uint32 step) const
	//	{
	//		//return lastUpdateStep > step;
	//		return uint32(step - lastUpdateStep) < uint32(1u << 31);
	//	}
	//};

	class VX_API Body
	{
	public:

		explicit Body(Vec3 pos);

		//External influences

		//Motion dynamics


		/// External impulse/forces
		/// Apply instiatiatious velocity 
		void ApplyImpulse(const Vec3& impulse);
		void ApplyImpulse(const Vec3& impulse, const Vec3& pointA);
		void ApplyImpulseLocal(const Vec3& impulse, const Vec3& pointA);
		void ApplyAngularImpulse(const Vec3& impulse, const Vec3& pointA);

		void ApplyPositionCorrection(const Vec3& nudge, const Vec3& r);

		/// Accumulate forces
		/// Add force, apply's force to the body
		/// as behave like a point masss
		/// force is applied to the center of mass
		/// without rotations
		void AddForce(const Vec3& force);
		/// add force to given point on the body
		// force and point are in world space.
		void AddForce(const Vec3& force, const Vec3& pointA);
		/// add force to given point on the body
		/// force direction in world coord, 
		/// point in bogy space
		/// useful for spring force, or other force fixed to the body
		//void AddForceAtBodyPoint(const Vec3& force, const Vec3& point);

		/// Solvers/States influences
		Vec3 Position() const { return mPosition; }
		void SetPosition(const Vec3& pos);

		Quat Orientation() const { return mOrientation; }
		void SetOrientation(const Quat& quat);

		void SetLinearVelocity(const Vec3& velocity);
		Vec3 LinearVelocity() const { return mLinearVelocity; }
		void AddLinearVelocityStep(const Vec3& delta) { mLinearVelocity += delta; }

		void SetAngularVelocity(const Vec3& velocity);
		Vec3 AngularVelocity() const { return mAngularVelocity; }
		void AddAngularVelocityStep(const Vec3& delta) { mAngularVelocity += delta; }

		/// GetPointVelocityRelCOM
		/// Total velocity of a point, given its offset from the COM
		/// point relative to body COM
		/// @returns total (absoute) velocity at point
		Vec3 GetPointVelocityRelCOM(const Vec3& offset) const { return IsDynamic() ? mLinearVelocity + mAngularVelocity.Cross(offset) : Vec3::Zero(); }
		/// GetPointVelocityWS
		/// point in world space
		/// @returns total (absoute) velocity at point
		Vec3 GetPointVelocity(const Vec3& world_point) const { return GetPointVelocityRelCOM(world_point - mPosition); }

		/// total world-space velocity of a point in body's local space
		/// Use: For points fixed to the object
		/// use GetPointVelocityRelCOM, if total world space of point in world space as offset from COM 
		/// 
		Vec3 GetPointVelocityLocal(const Vec3& local_point) const
		{
			Vec3 world_offset = mOrientation.Rotate(local_point);
			return GetPointVelocityRelCOM(world_offset);
		}

		/// Beware this is good to be used during position correction 
		/// to apply little nudge over iteration to bodies, and would not 
		/// compute internal bounds and mark position changes. but sure to update 
		/// Internal after correction.
		void ApplyAngularDisplacement(const Vec3& disp);
		/// Beware this is good to be used during position correction 
		/// to apply little nudge over iteration to bodies, and would not 
		/// compute internal bounds and mark position changes. but sure to update 
		/// Internal after correction.
		VX_INLINE void ApplyLinearDisplacement(const Vec3& disp) { mPosition += disp; }



		void UpdateTransformState();



		/// Static bodies is not allowed to go to sleep 
		/// for optimisation 
		/// Broad refits dynamic bodies node when aawake
		VX_INLINE bool IsAwake() const 
		{
			/// static is always awkae
			///return (mMotionType != EMotionType::Dynamic) || mAwake;
			/// 
			/// for now let just returen mAwake as static bodies are forced to not awake 
			/// on creation 
			return mAwake;
		}
		/// Static bodies is not allowed to go to sleep 
		/// for optimisation 
		/// Broad refits dynamic bodies node when aawake
		VX_INLINE bool IsSleeping() const 
		{ 
			//return (mMotionType == EMotionType::Dynamic) && !mAwake;
			return !mAwake;
		}

		///to participate in simulation id needs to be valid
		bool IsIDValid() const { return mID.IsValid(); }
		float InverseMass() const { return mInverseMass; }
		constexpr float Mass() const { return (mInverseMass == 0.0) ? 0.0 : (1.0 / mInverseMass); }

		bool IsDynamic() const { return mMotionType == EMotionType::Dynamic; }
		bool IsStatic() const { return mMotionType == EMotionType::Static; }

		EMotionType MotionType() const { return mMotionType; }

		Float3 AccumulatedForce() const { return mForceAccumulated; }

		

		//f(N->kgms^-2) = ma = mg 
		void ApplyGravity(const Vec3& gravity);


		/// Step Intergrators
		void IntegrateAcceleration(float dt, const Vec3& gravity);
		void IntegrateVelocity(float dt);

		void DampVelocities(float dt);
		void ClampVelocities();



		static bool CanBodiesCollide(const Body& b0, const Body& b1);

		Mat44 ComputeWorldTransform() const { return Mat44::RotationTranslation(mOrientation, mPosition);	}
		Mat44 ComputeWorldInverseTransform() const { return Mat44::InverseRotationTranslation(mOrientation, mPosition);	}

		/// for rendering
		/// hack for capsule as most shapes like sphere as a unified size on xyz similar to unit box
		/// but capsule is more close to a compound shape were y is usually twice xz (0.5, 1.0, 0.5)
	
		/// to wake up body use body manager to wake body, in order
		/// to activate body list
		void WakeUp(bool reset_timer = true)
		{
			mAwake = (mMotionType != EMotionType::Static) ? true : false;
			(reset_timer) ? mSleepTimer = 0.0f : (void)0;
		}
		void WakeUp(const Vec3& jolt)
		{
			WakeUp();
			ApplyImpulse(jolt);
		}

		void UpdateSleepState(float dt, const SleepingSettings& settings);

		//for debugging
		void SetWorldTransform(const Mat44& mat);

		
		const Shape* GetShape() const { return mShape.get(); }

		Mat44 TransformDiagonalInertiaTensor(const Vec3& inv_inertia_diagonal, const Quat& rot) const;
		Mat44 ComputeInvInertiaTensorWorld();
		/// ComputeInvInertiaAxesWorld
		/// compute tehe principal axes basis 
		/// of inertia tensor
		void ComputeInvInertiaAxesWorld(Vec3& axis_x, Vec3& axis_y, Vec3& axis_z);
		Vec3 ApplyInvInertiaTensorWorld_Basis(const Vec3& vector);
		Vec3 ApplyInvInertiaTensorWorld(const Vec3& vector);

		/// GetAABBWorld
		/// return precomputed world bound 
		/// 
		VX_INLINE AABB GetAABBWorld() const { return mBounds; }
		VX_INLINE AABB ComputeAABBWorld() 
		{ 
			ComputeWorldSpaceBoundsInternal();
			return mBounds;
		}
		
		///this is the inv interia with world rotation
		Mat44 ComputeInvInteriaWorld() const;
		Mat44 GetInvInteriaTensor() const { return Mat44::Scale(mInvInertiaTensorDiagonal); }

		Vec3 GetLocalInvInertiaDiagonal() const { return mInvInertiaTensorDiagonal; }

		void ClearAccumulatedForces()
		{
			mForceAccumulated = { 0.0f, 0.0f, 0.0f };
			mTorqueAccumulated = { 0.0f, 0.0f, 0.0f };
		}
		void ClearVelocities()
		{
			mLinearVelocity.ToZero();
			mAngularVelocity.ToZero();
		}




		BodyID ID() const { return mID; }
		TransformState GetTransformedState() const { return mMotionState; }
		EDynamicsDofs GetAllowedDynamicsDof() const { return mAllowedDynamicsDof; }
		void SetAllowedDynamicsDof(EDynamicsDofs dof) { mAllowedDynamicsDof = dof; }

		/// Co-efficents
		float FrictionCoeff() const { return mFriction; }
		float RestitutionCoeff() const { return mRestitution; }

		void SetFriction(float friction) { mFriction = friction; }
		void SetRestitution(float restitution) { mRestitution = restitution; }


		static constexpr uint32 kInvalidIslandIdx = 0xffffffff;
		uint32 GetIslandIndex() const { return mIslandIndex; }
		void SetIslandIndex(uint32 island_idx) { mIslandIndex = island_idx; }

		static constexpr uint32 kInvalidActiveIdx = 0xffffffff;
		uint32 GetIndexInActiveBodies() const { return mActiveIndex; }
		void SetIndexInActiveBodies(uint32 active_idx) { mActiveIndex = active_idx; }

		float MaxLinearVelocity() const { return mMaxLinearVelocity; }
		float MaxAngularVelocity() const { return mMaxAngularVelocity; }

		float LinearDamping() const { return mLinearDamping; }
		float AngularDamping() const { return mAngularDamping; }


		float GetKineticEnergy() const
		{
			if (IsStatic())return 0.0f;

			/// KE = linear energy + rot energy
			/// linear -> 1/2(m * v^2)
			/// rot -> 1/2(Iw^2) -> 1/2(w^T* I * w)
			float mass = 1.0f / mInverseMass;
			float lin_KE = 0.5f * mass * mLinearVelocity.LengthSq();
			
			Vec3 w_ls = mOrientation.InverseRotate(mAngularVelocity);//local space ang vel
			Vec3 Iw_ls = Vec3(
				w_ls.X() / mInvInertiaTensorDiagonal.X(),
				w_ls.Y() / mInvInertiaTensorDiagonal.Y(),
				w_ls.Z() / mInvInertiaTensorDiagonal.Z());

			float rot_KE = 0.5f * w_ls.Dot(Iw_ls);
			return lin_KE + rot_KE;
		}

	private:

		/// set the motion type and reset motion dynamics
		/// if update mass inertia, would use shape mass property
		/// if not ensure to set mass and inertia if motion type is dynamics
		void SetMotionType(EMotionType type);
		/// set shape
		/// if update mass inertia, would use shape mass property
		/// if not ensure to set mass and inertia if motion type is dynamics
		/// move to manager/interface 
		void SetShape(const RefConst<Shape>& shape, bool update_mass_inertia);

		VX_INLINE void SetMass(float mass)
		{
			mInverseMass = (mass > kEpsilon) ? (1.0f / mass) : 0.0f;
		}
		VX_INLINE void SetInertiaTensor(const Vec3& inertia_tensor_diagonal)
		{
			mInvInertiaTensorDiagonal = (!inertia_tensor_diagonal.IsZero()) ? inertia_tensor_diagonal.Reciprocal() : Vec3(0.0f);
		}

		void ComputeWorldSpaceBoundsInternal();


		///some of the values are not set intentionally
		///check body constructor for set values


		///MOTION STATE
		/// 1st cache line
		/// 16 bytes
		/// World space position
		Vec3 mPosition = Vec3(0.0f);								//16 bytes
		/// World space orientation (rotation)
		Quat mOrientation = Quat::Identity();							//16 bytes
		AABB mBounds = {};								//32 bytes (16 bytes aligned) //avoid bound strading 2 cache line
		
		
		/// collision crtical data on 2nd cache as all body has collision 
		/// properties then next motion (exculding static bodies)
		/// as well as infos like friction & resition for static - dynamic constraint praticpation

		//2nd cache line
		RefConst<Shape> mShape = nullptr;						//8 bytes
		/// for now the state is used by broadphase, to notice is body as moved before 
		/// updating BVH bound node
		TransformState mMotionState{};						//8 bytes	[16 bytes]<-- fix 
		BodyID mID{};										//4 bytes	[20 bytes]
		float mFriction = 0.4f;							//4 bytes	[24 bytes]
		float mRestitution = 0.6f;						//4 bytes	[28 bytes]
		/// Static bodies is not allowed to go to sleep 
		/// for optimisation 
		/// Broad refits dynamic bodies node when aawake
		/// later body should not have awake flag
		bool mAwake = false;							//1 bytes	[29 bytes]
		EMotionType mMotionType = EMotionType::Dynamic;//1 bytes	[30 bytes]
		/////////////////////////////////////////////////////////////////////////
		////////////////////////////////hack padding/////////////////////////////
		EDynamicsDofs mAllowedDynamicsDof; //1 bytes	[31 bytes] 
		//char padding[1 + 4*3];								//1 bytes	[32 bytes] //later sort this out when sleep time is removed padding will reduce

		char padding[1 + 4*0];								//1 bytes	[32 bytes] //later sort this out when sleep time is removed padding will reduce

		uint32 mIslandIndex = kInvalidIslandIdx;
		uint32 mActiveIndex = kInvalidActiveIdx;
	public:
		uint32 mIslandConstraintGroupMask{ 0 };
	private:
		 
		///World space linear velocity (m/s)
		Vec3 mLinearVelocity = Vec3(0.0f);							//16 bytes	[48 bytes]
		///World space angular velocity (rad/s)
		Vec3 mAngularVelocity = Vec3(0.0f);							//16 bytes	[64 bytes]


		/// motion is next on 3rd as not all body are dynamic, later ensure that 
		/// static bodies does not try get masses, not a hard requirement
		
		//3rd cache line 
		//The inertia tensor of this body, defined as a diagonal matrix in a reference
			//     frame positioned at this body's center of mass and rotated by Rigidbody.inertiaTensorRotation.
		Vec3 mInvInertiaTensorDiagonal = Vec3(0.0f);					//16 bytes	[16 bytes] 
		//4 bytes alignment, float3 is 4 bytes aligned compared to Vec3 which is 16 bytes align
		float mInverseMass = 0.0f;								//4 bytes	[20 bytes]
		Float3 mForceAccumulated{ 0 };						//12 bytes	[32 bytes]
		Float3 mTorqueAccumulated{ 0 };						//12 bytes	[44 bytes]
		float mLinearDamping = 1.0f;							//4 bytes	[48 bytes]
		float mAngularDamping = 1.0f;							//4 bytes	[52 bytes]
		float mMaxLinearVelocity;						//4 bytes	[56 bytes] // 100.0f;// 500.0f;too high at 60hz and no CCD 
		float mMaxAngularVelocity;						//4 bytes	[60 bytes] //30 - 50
		float mSleepTimer = 0.0f;								//4 bytes	[64 bytes]
		
		
		/// some detail for motion is overflowing into 4th cache line,
		/// later ensure that exterme least frequent data is in 4th cache line
		/// 


		
		friend class BodyManager;
		friend EditorUI;
	};
} //namespace vx