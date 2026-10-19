#pragma once

#include <external/json/single_include/nlohmann/json_fwd.hpp>
#include <external/json/single_include/nlohmann/json.hpp>

#include <fstream>

#include "Vortrix/Collision/Shapes/Shape.h"
#include "Vortrix/Dynamics/Body/Body.h"

#include "Vortrix/Maths/VortrixMaths.h"

using Json = nlohmann::json;


struct SerialisedShape
{
	vx::EShapeType type = vx::EShapeType::Sphere;
	vx::Float3 halfExtext;
	vx::Float3 planeNormal;
	float density = 1000.0f;
};



struct SerialisedBody
{
	//const char* debugName;
	vx::uint32 serialisedShapeIndex = vx::uint32(-1);
	vx::Float3 position;
	vx::Quat orientation;
	float friction = 0.4f;
	float restitution = 0.2f;
	vx::EMotionType motionType = vx::EMotionType::Dynamic;
	float maxLinearVelocity = 0.0f;
	float maxAngularVelocity = 0.0f;
	/// if in broadphase body participates in collision
	bool inBroadphase = true;

	float linearDamping = 0.0f;
	float angularDamping = 0.0f;
};


struct SerialisedRagdollSettings
{
	vx::Vec3 position = vx::Vec3(0.0f);
	float limbsOffset = 0.15f;

	bool splitTorso = true;
	vx::EShapeType shapesType = vx::EShapeType::Box;

	/// used to approximate ragdoll position to save	
	int firstBodyOffsetInScene = -1;
	vx::uint32 limbsCount = vx::uint32(-1);
};


struct SerialisedSceneDesc
{
	vx::StackString<32> mName;
	std::string mInfo;
};

struct SerialisedSceneCameraState
{
	vx::Float3 position;
	vx::Float3 forward;
	vx::Float3 up;
};

struct SerialisedDistanceConstraint
{
	vx::uint32 serialisedBodyA;
	vx::uint32 serialisedBodyB;
	vx::Vec3 localAnchorA;
	vx::Vec3 localAnchorB;
	float minDist;
	float maxDist;
	float frequency;
	float dampingRatio;
};

struct SerialisedPointConstraint
{
	vx::uint32 serialisedBodyA;
	vx::uint32 serialisedBodyB;

	vx::Vec3 localAnchorA;
	vx::Vec3 localAnchorB;

	bool enableVelocityBias = true;
	float errorTreshold = vx::kEpsilon;
};


class Camera;
class Scenario;
namespace vx {
	class BodyManager;
	class PhysicsWorld;
}

namespace serialiser
{
	template<typename T>
	Json ToJson(const T& t) { return Json(t); }
	template<typename T>
	T FromJson(Json& j) { return j.get<T>(); }
	template<typename T>
	T& FromJson(Json& j, T& to) { return j.get_to(to); }



	static vx::StackString<128> directory("scenarios/");
	static constexpr const char* defaulfDirectory = "assets/scenarios/";
	bool Serialise(const vx::BodyManager* body_manager, const Camera& app_cam, const char* name, const char* directory, const char* info, Scenario* scenario);
	bool Deserialise(vx::PhysicsWorld* io_physicsworld, Scenario* scenario, std::string& scene_name, std::string& scene_info, Camera& app_cam, const char* file_path);

	VX_INLINE bool Deserialise(vx::PhysicsWorld* io_physicsworld, Scenario* scenario, std::string& scene_name, std::string& scene_info, Camera& app_cam, const char* name, const char* directory)
	{ return Deserialise(io_physicsworld, scenario, scene_name, scene_info, app_cam, (std::string(directory) + name).c_str()); }
};


namespace nlohmann {
	/////////////////////////////////////////////////////////////
	/////////////////////////Vec2////////////////////////////
	/////////////////////////////////////////////////////////////
	template<>
	struct adl_serializer<vx::Vec2>
	{
		static void to_json(Json& j, const vx::Vec2& v)
		{
			j = Json{ v.X(), v.Y() };
		}
		static void from_json(const Json& j, vx::Vec2& v)
		{
			for (int i = 0; i < 2; i++)
				v[i] = j.at(i).get<float>();
		}
	};

	/////////////////////////////////////////////////////////////
	/////////////////////////Vec3////////////////////////////
	/////////////////////////////////////////////////////////////
	template<>
	struct adl_serializer<vx::Vec3>
	{
		static void to_json(Json& j, const vx::Vec3& v)
		{
			j = Json{ v.X(), v.Y(), v.Z()};
		}
		static void from_json(const Json& j, vx::Vec3& v)
		{
			for (int i = 0; i < 3; i++)
				v[i] = j.at(i).get<float>();
		}
	};

	/////////////////////////////////////////////////////////////
	/////////////////////////Float3////////////////////////////
	/////////////////////////////////////////////////////////////
	template<>
	struct adl_serializer<vx::Float3>
	{
		static void to_json(Json& j, const vx::Float3& v)
		{
			j = Json{ v.x, v.y, v.z };
		}
		static void from_json(const Json& j, vx::Float3& v)
		{
			for (int i = 0; i < 3; i++)
				v[i] = j.at(i).get<float>();
		}
	};


	/////////////////////////////////////////////////////////////
	/////////////////////////Vec4////////////////////////////
	/////////////////////////////////////////////////////////////
	template<>
	struct adl_serializer<vx::Vec4>
	{
		static void to_json(Json& j, const vx::Vec4& v)
		{
			j = Json{ v.X(), v.Y(), v.Z(), v.W()};
		}
		static void from_json(const Json& j, vx::Vec4& v)
		{
			for (int i = 0; i < 4; i++)
				v[i] = j.at(i).get<float>();
		}
	};

	/////////////////////////////////////////////////////////////
	/////////////////////////Quat////////////////////////////
	/////////////////////////////////////////////////////////////
	template<>
	struct adl_serializer<vx::Quat>
	{
		static void to_json(Json& j, const vx::Quat& q)
		{
			j = Json{ q.X(), q.Y(), q.Z(), q.W()};
		}
		static void from_json(const Json& j, vx::Quat& q)
		{
			vx::Vec4 v;
			for (int i = 0; i < 4; i++)
				v[i] = j.at(i).get<float>();
			q = v;
		}
	};


	template<>
	struct adl_serializer<SerialisedShape>
	{
		static void to_json(Json& j, const SerialisedShape& s)
		{
			j = Json
			{
				{"density", s.density},
				{"type", s.type}, 
				{"halfExtext", s.halfExtext}, 
				{"planeNormal", s.planeNormal},
			};
		}
		static void from_json(const Json& j, SerialisedShape& s)
		{
			j.at("density").get_to(s.density);
			j.at("type").get_to(s.type);
			j.at("halfExtext").get_to(s.halfExtext);
			j.at("planeNormal").get_to(s.planeNormal);
		}
	};


	template<>
	struct adl_serializer<SerialisedBody>
	{
		static void to_json(Json& j, const SerialisedBody& s)
		{
			j = Json
			{
				//{"debugName", s.debugName},
				{"serialisedShapeIndex", s.serialisedShapeIndex},
				{"position", s.position},
				{"orientation", s.orientation},
				{"friction", s.friction},
				{"restitution", s.restitution},
				{"motionType", s.motionType},
				{"maxLinearVelocity", s.maxLinearVelocity},
				{"maxAngularVelocity", s.maxAngularVelocity},
				{"inBroadphase", s.inBroadphase},
				{"linearDamping", s.linearDamping},
				{"angularDamping",s.angularDamping},
			};
		}
		static void from_json(const Json& j, SerialisedBody& s)
		{
			//j.at("debugName").get_to(s.debugName);
			j.at("serialisedShapeIndex").get_to(s.serialisedShapeIndex);
			j.at("position").get_to(s.position);
			j.at("orientation").get_to(s.orientation);
			j.at("friction").get_to(s.friction);
			j.at("restitution").get_to(s.restitution);
			j.at("motionType").get_to(s.motionType);
			j.at("maxLinearVelocity").get_to(s.maxLinearVelocity);
			j.at("maxAngularVelocity").get_to(s.maxAngularVelocity);
			j.at("inBroadphase").get_to(s.inBroadphase);
			j.at("linearDamping").get_to(s.linearDamping);
			j.at("angularDamping").get_to(s.angularDamping);
		}
	};


	template<>
	struct adl_serializer<SerialisedSceneCameraState>
	{
		static void to_json(Json& j, const SerialisedSceneCameraState& s)
		{
			j = Json
			{
				{"position", s.position},
				{"forward", s.forward},
				{"up", s.up},
			};
		}
		static void from_json(const Json& j, SerialisedSceneCameraState& s)
		{
			j.at("position").get_to(s.position);
			j.at("forward").get_to(s.forward);
			j.at("up").get_to(s.up);
		}
	};

	
	template<>
	struct adl_serializer<SerialisedRagdollSettings>
	{
		static void to_json(Json& j, const SerialisedRagdollSettings& s)
		{
			j = Json
			{
				{"position", s.position},
				{"limbsOffset", s.limbsOffset},
				{"splitTorso", s.splitTorso},
				{"shapesType", s.shapesType},
			};
		}

		static void from_json(const Json& j, SerialisedRagdollSettings& s)
		{
			j.at("position").get_to(s.position);
			j.at("limbsOffset").get_to(s.limbsOffset);
			j.at("splitTorso").get_to(s.splitTorso);
			j.at("shapesType").get_to(s.shapesType);
		}
	};


	template<>
	struct adl_serializer<SerialisedDistanceConstraint>
	{
		static void to_json(Json& j, const SerialisedDistanceConstraint& s)
		{
			j = Json
			{
				{"serialisedBodyA", s.serialisedBodyA},
				{"serialisedBodyB", s.serialisedBodyB},
				{"localAnchorA", s.localAnchorA},
				{"localAnchorB", s.localAnchorB},
				{"minDist", s.minDist},
				{"maxDist", s.maxDist},
				{"frequency", s.frequency},
				{"dampingRatio", s.dampingRatio},
			};
		}

		static void from_json(const Json& j, SerialisedDistanceConstraint& s)
		{
			j.at("serialisedBodyA").get_to(s.serialisedBodyA);
			j.at("serialisedBodyB").get_to(s.serialisedBodyB);
			j.at("localAnchorA").get_to(s.localAnchorA);
			j.at("localAnchorB").get_to(s.localAnchorB);
			j.at("minDist").get_to(s.minDist);
			j.at("maxDist").get_to(s.maxDist);
			j.at("frequency").get_to(s.frequency);
			j.at("dampingRatio").get_to(s.dampingRatio);
		}
	};

	template<>
	struct adl_serializer<SerialisedPointConstraint>
	{
		vx::uint32 serialisedBodyA;
		vx::uint32 serialisedBodyB;

		vx::Vec3 localAnchorA;
		vx::Vec3 localAnchorB;

		bool enableVelocityBias = true;
		float errorTreshold = vx::kEpsilon;


		static void to_json(Json& j, const SerialisedPointConstraint& s)
		{
			j = Json
			{
				{"serialisedBodyA", s.serialisedBodyA},
				{"serialisedBodyB", s.serialisedBodyB},
				{"localAnchorA", s.localAnchorA},
				{"localAnchorB", s.localAnchorB},
				{"enableVelocityBias", s.enableVelocityBias},
				{"errorTreshold", s.errorTreshold},
			};
		}

		static void from_json(const Json& j, SerialisedPointConstraint& s)
		{
			j.at("serialisedBodyA").get_to(s.serialisedBodyA);
			j.at("serialisedBodyB").get_to(s.serialisedBodyB);
			j.at("localAnchorA").get_to(s.localAnchorA);
			j.at("localAnchorB").get_to(s.localAnchorB);
			j.at("enableVelocityBias").get_to(s.enableVelocityBias);
			j.at("errorTreshold").get_to(s.errorTreshold);
		}
	};


}