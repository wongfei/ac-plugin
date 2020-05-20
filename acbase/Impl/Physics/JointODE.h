#pragma once

//UDT: class IJoint @len=8 @vfcount=3
	//_VTable: 
	//_Func: public void release(); @intro @pure @virtual vtpo=0 vfid=0 @loc=optimized @len=0 @rva=0
	//_Func: public void setERPCFM(float _arg0, float _arg1); @intro @pure @virtual vtpo=0 vfid=1 @loc=optimized @len=0 @rva=0
	//_Func: protected void ~IJoint(); @intro @virtual vtpo=0 vfid=2 @loc=optimized @len=0 @rva=0
	//_Func: public void IJoint(const IJoint & _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void IJoint(); @loc=optimized @len=0 @rva=0
	//_Func: public IJoint & operator=(const IJoint & _arg0); @loc=optimized @len=0 @rva=0
	//_Func: public void __local_vftable_ctor_closure(); @loc=optimized @len=0 @rva=0
	//_Func: protected void * __vecDelDtor(unsigned int _arg0); @intro @virtual vtpo=0 vfid=2 @loc=optimized @len=0 @rva=0
//UDT;

class _IJoint
{
public:
	virtual void release() {}
	virtual void setERPCFM(float erp, float cfm) {}

protected:
	virtual ~_IJoint() {}
};

class _JointODE : public _IJoint
{
public:
	inline _JointODE(PhysicsCore* inCore, dJointID inId) : 
		id(inId)
	{}

	dJointID id;
};

class _SliderJointODE : public _JointODE
{
public:
	inline _SliderJointODE(PhysicsCore* inCore, dJointID inId) : 
		_JointODE(inCore, inId) 
	{}

	virtual void setERPCFM(float erp, float cfm)
	{
		if (erp > 0.0f)
			ODE_CALL(dJointSetSliderParam)(this->id, 13, erp);
		if (cfm > 0.0f)
			ODE_CALL(dJointSetSliderParam)(this->id, 8, cfm);
	}
};

class _DistanceJointODE : public _JointODE
{
public:
	inline _DistanceJointODE(PhysicsCore* inCore, dJointID inId, float inDistance) : 
		_JointODE(inCore, inId),
		distance(inDistance)
	{}

	virtual void setERPCFM(float erp, float cfm)
	{
		if (erp > 0.0f)
			ODE_CALL(dJointSetDBallParam)(this->id, 13, erp);
		if (cfm > 0.0f)
			ODE_CALL(dJointSetDBallParam)(this->id, 8, cfm);
	}

	float distance;
};
