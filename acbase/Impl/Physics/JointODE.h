#pragma once

class _JointODE : public IJoint
{
public:
	inline _JointODE(PhysicsCore* inCore, dJointID inId) : core(inCore), id(inId) {}

	PhysicsCore* core;
	dJointID id;
};
