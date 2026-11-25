#pragma once
#include "Scenario.h"

namespace vx {
	class Constraint;
}

class PointConstraintScenario : public Scenario
{
public:

	void Init(vx::PhysicsWorld* i_world) override;
	const char* Name() override { return "Point Constraint Scenario"; }
	const char* Info() override
	{
		return "Scenario to test dynamic Point Constraint Scenario";
	}

	void OnUI() override;
	void PostPhysicsStep(float dt) override;

	void OnClose() override;
	~PointConstraintScenario();

private:
	std::vector<vx::Constraint*> mConstraints;
};