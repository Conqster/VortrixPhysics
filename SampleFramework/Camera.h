#pragma once

#include "Vortrix/Maths/VortrixMaths.h"
#include "Vortrix/Core/Assertion.h"

class Camera
{
public:
	Camera() = default;
	Camera(const vx::Vec3& pos, float pitch, float yaw, float move_speed, float rot_speed);
	Camera(const vx::Vec3& pos, const vx::Vec3& forward, const vx::Vec3& up, float move_speed, float rot_speed);

	vx::Mat44 ViewMat();
	vx::Mat44 ProjMat(float aspect_ratio);

	void Translate(vx::Vec3 dir, float dt);
	void Rotate(float dx, float dy);

	vx::Vec3 Position() const { return mState.position; }
	vx::Vec3 Forward() const { return mState.forward; }
	vx::Vec3 GetRight() const
	{ 
		return vx::Vec3::Cross(mState.forward, mState.up);
		//return -mState.orientation.RotateAxisX().Normalised();
	}
	vx::Vec3 Up() const { return mState.up; }

	vx::Vec3 ScreenToWorld(const vx::Vec3 screen_pos, vx::uint32 width, vx::uint32 height)
	{
		vx::Vec3 m2x2y = (screen_pos * vx::Vec3(2.0f, 2.0f, 1.0f)) / 
						vx::Vec3(width, height, 1.0f);

		//NDC 
		float x = m2x2y.X() - 1.0f;
		float y = 1.0f - m2x2y.Y();

		vx::Vec4 clip(x, y, screen_pos.Z(), 1.0f);

		vx::Vec4 view = ProjMat(float(width)/float(height)).Inverse().Multiply(clip);
		view /= view.W();

		return ViewMat().TransformInverse(view);
	}

	vx::Vec3 WorldToScreen(vx::Vec3 screen_pos)
	{
		return ViewMat().Transform(screen_pos);
	}

	struct State
	{
		State(vx::Vec3 pos = vx::Vec3::Zero(),
			vx::Vec3 fwd = vx::Vec3(0.0f, 0.0f, -1.0f), vx::Vec3 up = vx::Vec3(0.0f, 1.0f, 0.0f)) :
			position(pos), forward(fwd), up(up) 
		{
			orientation = vx::Quat::LookRotation(fwd, up);
			VX_ASSERT_WARN(orientation.IsUnitQuat(), "Issue Camera State Orientation is not unit.");
			forward = orientation.RotateAxisZ();
			up = orientation.RotateAxisY();
		}

		State(vx::Vec3 pos, float _pitch, float yaw) :
			position(pos), pitch(_pitch)
		{
			orientation = vx::Quat::FromEulerAngle(vx::Vec3(vx::DegToRad(_pitch), vx::DegToRad(yaw), 0.0f));
			forward = orientation.RotateAxisZ();
			up = orientation.RotateAxisY();
		}

		vx::Vec3 position = vx::Vec3::Zero();
		vx::Vec3 forward = vx::Vec3(0.0f, 0.0f, -1.0f);
		vx::Vec3 up = vx::Vec3(0.0f, 1.0f, 0.0f);

		float pitch = 0.0f;
		vx::Quat orientation = vx::Quat::Identity();

	};

	struct Properties
	{
		Properties() = default;
		Properties(float move, float rot_sensitivity, float fovy = 60.0f,
			float _near = 0.2f, float _far = 500.0f) :
			moveSpeed(move), rotSensitivity(rot_sensitivity),
			fovY(fovy), zNear(_near), zFar(_far) {
		}

		/// 20 - 100
		float moveSpeed = 25.0f;
		float rotSensitivity = 0.05f;

		float fovY = 60.0f;
		/// 0.1 - 1
		float zNear = 0.2f;
		/// 100 - 2000
		float zFar = 500.0f;
	};
private:
	State mState;
	Properties mProperties;

public:
	State& GetState() { return mState; }
	const State& GetState() const { return mState; }

	void SetState(const State& state) { mState = state; }
	void SetProperties(const Properties& props) { mProperties = props; }

	Properties& GetProperties() { return mProperties; }
	const Properties& GetProperties() const { return mProperties; }

private:
	void Update();
};