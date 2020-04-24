#pragma once

BEGIN_HOOK_OBJ(SuspensionStrut)

	#define RVA_SuspensionStrut_step 2909696

	void _step(float dt);

END_HOOK_OBJ()

void _SuspensionStrut::_step(float dt)
{
	this->steerTorque = 0.0f;

	vec3f vCarStrut = this->carBody->localToWorld(this->dataRelToBody.carStrut);
	vec3f vTyreStrut = this->hub->localToWorld(this->dataRelToWheel.tyreStrut);
	vec3f vDelta = vsub(vTyreStrut, vCarStrut);
	float fDelta = vlen(vDelta);

	if (fDelta != 0.0f) {
		vDelta = vmul(vDelta, 1.0f / fDelta);
	}

	float fDefaultLength = this->strutBaseLength + this->rodLength;
	float fTravel = fDefaultLength - fDelta;
	this->status.travel = fTravel;

	float fForce = ((fTravel * this->progressiveK) + this->k) * fTravel;
	if (fForce < 0) fForce = 0;

	if (this->packerRange != 0.0f && fTravel > this->packerRange)
		fForce += ((fTravel - this->packerRange) * this->bumpStopRate);

	if (fForce > 0)
	{
		vec3f vForce = vmul(vDelta, fForce);
		this->addForceAtPos(vForce, vTyreStrut, false, false);

		vForce = vmul(vForce, -1.0f);
		this->carBody->addForceAtPos(vForce, vCarStrut);
	}

	vec3f vHubWorld = this->hub->getPosition(0.0f);
	vec3f vHubLocal = this->carBody->worldToLocal(vHubWorld);
	mat44f mxCarWorld = this->carBody->getWorldMatrix(0.0f);

	float fHubDelta = vHubLocal.y - this->dataRelToWheel.refPoint.y;

	if (fHubDelta > this->bumpStopUp)
	{
		fForce = (fHubDelta - this->bumpStopUp) * 500000.0f;

		vec3f vForce = vmul(vec3f(mxCarWorld.M21, mxCarWorld.M22, mxCarWorld.M23), -fForce);
		vec3f vPos = this->hub->getPosition(0.0f);
		this->addForceAtPos(vForce, vPos, false, false);

		vForce = vec3f(0, fForce, 0);
		this->carBody->addLocalForceAtLocalPos(vForce, vHubLocal);
	}

	if (fHubDelta < this->bumpStopDn)
	{
		fForce = (fHubDelta - this->bumpStopDn) * 500000.0f;

		vec3f vForce = vmul(vec3f(mxCarWorld.M21, mxCarWorld.M22, mxCarWorld.M23), -fForce);
		vec3f vPos = this->hub->getPosition(0.0f);
		this->addForceAtPos(vForce, vPos, false, false);

		vForce = vec3f(0, fForce, 0);
		this->carBody->addLocalForceAtLocalPos(vForce, vHubLocal);
	}

	vec3f vTyreStrutVel = this->hub->getLocalPointVelocity(this->dataRelToWheel.tyreStrut);
	vec3f vCarStrutVel = this->carBody->getLocalPointVelocity(this->dataRelToBody.carStrut);
	vec3f vDamperDelta = vsub(vTyreStrutVel, vCarStrutVel);

	float fDamperSpeed = vdot(vDamperDelta, vDelta);
	float fDamperForce = this->damper.getForce(fDamperSpeed);
	this->status.damperSpeedMS = fDamperSpeed;

	vec3f vDamperForce = vmul(vDelta, fDamperForce);
	this->addForceAtPos(vDamperForce, vTyreStrut, false, false);

	vDamperForce = vmul(vDamperForce, -1.0f);
	this->carBody->addForceAtPos(vDamperForce, vCarStrut);
}
