#pragma once

BEGIN_HOOK_OBJ(SuspensionStrut)

	#define RVA_SuspensionStrut_step 2909696

	void _step(float dt);

END_HOOK_OBJ()

void _SuspensionStrut::_step(float dt)
{
	this->steerTorque = 0.0f;

	mat44f mxCarWorld = this->carBody->getWorldMatrix(0.0f);

	vec3f vCarStrut = this->carBody->localToWorld(this->dataRelToBody.carStrut);
	vec3f vTyreStrut = this->hub->localToWorld(this->dataRelToWheel.tyreStrut);

	vec3f vDelta = vTyreStrut - vCarStrut;
	float fDelta = vlen(vDelta);
	vDelta = vnorm(vDelta, fDelta);

	float fDefaultLength = this->strutBaseLength + this->rodLength;
	float fTravel = fDefaultLength - fDelta;
	this->status.travel = fTravel;

	float fForce = ((fTravel * this->progressiveK) + this->k) * fTravel;
	if (fForce < 0) 
		fForce = 0;

	if (this->packerRange != 0.0f && fTravel > this->packerRange)
		fForce += ((fTravel - this->packerRange) * this->bumpStopRate);

	if (fForce > 0)
	{
		vec3f vForce = vDelta * fForce;
		this->addForceAtPos(vForce, vTyreStrut, false, false);
		this->carBody->addForceAtPos(vForce * -1.0f, vCarStrut);
	}

	vec3f vHubWorld = this->hub->getPosition(0.0f);
	vec3f vHubLocal = this->carBody->worldToLocal(vHubWorld);
	float fHubDelta = vHubLocal.y - this->dataRelToWheel.refPoint.y;

	if (fHubDelta > this->bumpStopUp)
	{
		fForce = (fHubDelta - this->bumpStopUp) * 500000.0f;
		this->addForceAtPos(vec3f(&mxCarWorld.M21) * -fForce, this->hub->getPosition(0.0f), false, false);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fForce, 0), vHubLocal);
	}

	if (fHubDelta < this->bumpStopDn)
	{
		fForce = (fHubDelta - this->bumpStopDn) * 500000.0f;
		this->addForceAtPos(vec3f(&mxCarWorld.M21) * -fForce, this->hub->getPosition(0.0f), false, false);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fForce, 0), vHubLocal);
	}

	vec3f vTyreStrutVel = this->hub->getLocalPointVelocity(this->dataRelToWheel.tyreStrut);
	vec3f vCarStrutVel = this->carBody->getLocalPointVelocity(this->dataRelToBody.carStrut);
	vec3f vDamperDelta = vTyreStrutVel - vCarStrutVel;

	float fDamperSpeed = vdot(vDamperDelta, vDelta);
	this->status.damperSpeedMS = fDamperSpeed;

	float fDamperForce = this->damper.getForce(fDamperSpeed);
	vec3f vDamperForce = vDelta * fDamperForce;
	this->addForceAtPos(vDamperForce, vTyreStrut, false, false);
	this->carBody->addForceAtPos(vDamperForce * -1.0f, vCarStrut);
}
