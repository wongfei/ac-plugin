#pragma once

#if 0

class _JointODE : public IJoint
{
public:
	_JointODE(PhysicsCore* inCore, dJointID inId) : core(inCore), id(inId) {}

	virtual void release_vf0() {}
	virtual void setERPCFM_vf1(float  _arg0, float  _arg1) {}
	virtual ~_JointODE() {}

	PhysicsCore* core;
	dJointID id;
};

#endif
