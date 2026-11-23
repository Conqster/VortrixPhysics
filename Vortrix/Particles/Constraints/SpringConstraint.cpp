#include "SpringConstraint.h"
#include "Vortrix/Particles/Particle.h"
#include "Vortrix/Visuals/Renderers.h"

#include "Vortrix/Core/Colours.h"

namespace vx::Particles
{
	SpringConstraint::SpringConstraint(Particle* particle_a, Particle* particle_b, float rest_length, float stiffness, float critical_damp_ratio) :
		mParticleA(particle_a), mParticleB(particle_b), mRestLength(rest_length), mStiffness(stiffness), mDampingRatio(critical_damp_ratio)
	{
		//if (critical_damp_ratio < -1.0)
		//	mDampingRatio = VxSqrt((mParticleA->Mass() + mParticleB->Mass()) * mStiffness);

		mDampingRatio = VxClamp01(critical_damp_ratio);
	}

	void SpringConstraint::UpdateSolver(float time_step)
	{
		if (!mParticleA || !mParticleB)
			return;

		auto& bodyA = *mParticleA;
		auto& bodyB = *mParticleB;

		//opt out if both particle has infinte masses
		if (!bodyA.HasFiniteMass() && !bodyB.HasFiniteMass())
			return;

		const Vec3 displacement = bodyB.GetPosition() - bodyA.GetPosition();
		const float length = displacement.Length();

		//opt out is distance/length is too small
		if (length < kEpsilon)
			return;

		const Vec3 dir = displacement / length;


		//Hookes Law F = -k * (X - X_rest)
		//where k = stiffness, X = curr position, X_rest = rest displacement
		// according:https://en.wikipedia.org/wiki/Hooke%27s_law
		//F & X are vector discribing a displacement in space
		//in this case f = -k * (x - x_rest)
		// f = Fmag => spring force magnitude, x = current length, x_rest = @rest length
		const float spring_mag = -mStiffness * (length - mRestLength);


		//effective mass along axis 
		// massA x massB / massA + mass B
		const float mA = bodyA.Mass();
		const float mB = bodyB.Mass();
		const float mass_eff = (mA * mB) / (mA + mB);
		//critical dampling 
		const float crit_damping = 4.0f * VxSqrt(mStiffness * mass_eff);
		const float damping_coeff = mDampingRatio * crit_damping;
		//damping dampingforce = -dampling constant * relative velocity
		const Vec3 relative_velocity = bodyB.GetVelocity() - bodyA.GetVelocity();
		const float damping_mag = -damping_coeff * relative_velocity.Dot(dir);


		//accomodating the displacement vector properties as both particle could be in motion
		//from F = -k * (X - X_rest)
		//displacement dir * total force i.e F = f(Fmag) * displacement
		const Vec3 force = dir * (spring_mag + damping_mag);



		if (!bodyA.HasFiniteMass())
			bodyB.AddForce(force);
		else if (!bodyB.HasFiniteMass())
			bodyA.AddForce(-force);
		else
		{
			bodyA.AddForce(-force);
			bodyB.AddForce(force);
		}
	}

	void SpringConstraint::DebugGizmos(DebugGizmosRenderer* debug_renderer)
	{
		if (debug_renderer && mParticleA && mParticleB)
		{
			debug_renderer->DrawLine(mParticleA->GetPosition(), mParticleB->GetPosition(), Colour(0.3f, 0.3f, 0.3f));

			//split rest length based on mass contribution 
			float total_mass = mParticleA->Mass() + mParticleB->Mass();
			//distribute force if total mass is not too small else split
			float ratio = (total_mass > kEpsilon) ? (mParticleB->Mass() / total_mass) : 0.5;

			Vec3 dir = (mParticleB->GetPosition() - mParticleA->GetPosition()).Normalised();
			debug_renderer->DrawLine(mParticleA->GetPosition(), (mParticleA->GetPosition() + (dir * static_cast<float>(mRestLength * ratio))), Colour(1.0f, 1.0f, 0.0f));
			debug_renderer->DrawLine(mParticleB->GetPosition(), (mParticleB->GetPosition() + (-dir * static_cast<float>(mRestLength * (1 - ratio)))), Colour(0.0f, 1.0f, 0.0f));
		}
	}
}