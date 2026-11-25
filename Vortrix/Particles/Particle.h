#pragma once
#include <Vortrix/Vortrix.h>


//#include "EditorUI.h"

namespace vx::Particles
{
	class VX_API Particle
	{
	public:
		Particle() = default;
		Particle(Vec3 pos, float mass = 1.0f, float gravity_scale = 1.0f, float damping = 0.95) : mPosition(pos), mMass(mass),
			mDamping(damping) {
			mInverseMass = static_cast<float>((mass > 0.0) ? 1.0 / mass : 0.0);
		}
		friend class EditorUI;
	private:
		Vec3 mVelocity = Vec3(0.0f);

		/*
		* mDamping --> For Implicit Numerical damping value [0, 1]
		* @ according to Newcastle university (Game Engineering - Game Physics Material)
		* Physics - Linear Motion a damping of 0.95f --> given a factor (1.0 - 0.95)
		* i.e defines the kinetic energy loss in velocity after intergrating acceleration
		* tending towards 1.0f strenghtens the energy loss.
		* 0.0f rep no energy loss which potentially serve as closed energy system,
		* but energy could be loss via Explicit Damping
		*											  1) @(void)ApplyVelocityDampling
		*											  2) Special @(void)ClampVelocity <-- limit the max gained velocity attainable
		*												 which prevent unrealistic high-speed motion.
		*/
		float mDamping = 0.95f;
		float mInverseMass = 1.0f;
		float mMass = 1.0f;

		/// <summary>
		/// Gravity scale (ratio [0,1] which define the ratio/scale of global gravity influence
		/// on particle. 
		/// </summary>
		float mGravityScale = 1.0f;


		float mRadius = 0.1f;
	public:
		Vec3 mPosition = Vec3(0.0f);
		Vec3 mAcceleration = Vec3(0.0f);

		Vec3 mAccumlatedForce = Vec3();

		void SemiImplicitIntegrate(float dt);
		void EulerIntegrate(float dt);
		void AddForce(const Vec3& force);
		void AddImpluse(const Vec3& impluse);

		//for inplicit force application
		//f(N->kgms^-2) = ma = mg 
		void ApplyGravity(const Vec3& gravity);

		void SetGravityScale(float scale);

		void SetMass(float mass) {

			mInverseMass = static_cast<float>((mass > 0.0) ? 1.0 / mass : 0.0);
			mMass = mass;
		}

		void SetDamping(float damping) {
			mDamping = damping;
		}

		const bool HasFiniteMass() const { return mInverseMass > 0.0; }
		float Mass() { return mMass; }
		float InverseMass() { return mInverseMass; }
		Vec3 GetVelocity() const { return mVelocity; }
		Vec3 GetPosition() const { return mPosition; }

		float GetRadius() const { return mRadius; }



		//Soft limiting Particle velocity 
		void ApplyVelocityDampling(float damping_factor = 0.4f, float max_speed = 0.0) {
			float speed = mVelocity.Length();
			//when max_speed is not set i.e == 0, explicitly damping (loss energy)
			max_speed = (max_speed == 0.0) ? speed + static_cast<float>(1.0) : max_speed;
			if (speed > max_speed)
				mVelocity *= damping_factor;
		}

		void ClampVelocity(float max_speed) {
			float speed = mVelocity.Length();
			if (speed > max_speed)
				mVelocity = (mVelocity / speed) * max_speed;
		}
	private:
		void ClearAccumlator();

	};
} //VPHX::Particles namespace