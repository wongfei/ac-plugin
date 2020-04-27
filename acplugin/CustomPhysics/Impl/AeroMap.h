#pragma once

BEGIN_HOOK_OBJ(AeroMap)

	#define RVA_AeroMap_step 2847056
	#define RVA_AeroMap_addDrag 2840672
	#define RVA_AeroMap_addLift 2841232
	#define RVA_AeroMap_getCurrentDragKG 2841440
	#define RVA_AeroMap_getCurrentLiftKG 2841488

	void _step(float dt);
	void _addDrag(const vec3f& lv);
	void _addLift(const vec3f& lv);
	float _getCurrentDragKG();
	float _getCurrentLiftKG();

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

void _AeroMap::_step(float dt)
{
	if (this->wings.empty())
	{
		vec3f lv = this->carBody->getLocalVelocity();
		this->addDrag(lv);
		this->addLift(lv);
	}

	for (auto& wing : this->wings)
	{
		wing.step(dt);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _AeroMap::_addDrag(const vec3f& lv)
{
	float fDot = lv.sqlen();
	if (fDot != 0.0f)
	{
		vec3f vNorm = lv / sqrtf(fDot);
		this->dynamicCD = (((fabsf(vNorm.x) * this->CD) * this->CDX) + this->CD) + ((fabsf(vNorm.y) * this->CD) * this->CDY);

		float fDrag = ((this->dynamicCD * fDot) * this->airDensity) * this->referenceArea;
		this->carBody->addLocalForce(vNorm * -(fDrag * 0.5f));

		vec3f vAngVel = this->carBody->getAngularVelocity();
		fDot = vAngVel.sqlen();
		if (fDot != 0.0f)
		{
			vNorm = vAngVel / sqrtf(fDot);
			this->carBody->addLocalTorque(vNorm * -(fDot * this->CDA));
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _AeroMap::_addLift(const vec3f& lv)
{
	float fZZ = lv.z * lv.z;
	if (fZZ != 0.0f)
	{
		float fLift = (((fZZ * this->CL) * this->airDensity) * this->referenceArea) * 0.5f;

		float fFrontLift = fLift * this->frontShare;
		this->carBody->addLocalForceAtLocalPos(vec3f(0, -fFrontLift, 0), this->frontApplicationPoint);

		float fRearLift = fLift * (1.0f - this->frontShare);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, -fRearLift, 0), this->rearApplicationPoint);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _AeroMap::_getCurrentDragKG()
{
	float fSum = 0;
	for (auto& wing : this->wings)
	{
		fSum += wing.status.dragKG;
	}
	return fSum;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _AeroMap::_getCurrentLiftKG()
{
	float fSum = 0;
	for (auto& wing : this->wings)
	{
		fSum += wing.status.liftKG;
	}
	return fSum;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
