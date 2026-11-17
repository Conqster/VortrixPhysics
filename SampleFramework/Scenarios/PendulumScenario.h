#pragma once
#include "Scenario.h"


class PendulumScenario : public Scenario
{
public:
	void Init(vx::PhysicsWorld* i_world) override;
	const char* Name() override { return "Simple Pendulum Scenario"; }
	const char* Info() override
	{
		return "A Simple Pendulum Scenario";
	}

	
	void OnUI() override;

private:
};