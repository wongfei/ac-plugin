#pragma once

BEGIN_HOOK_OBJ(SuspensionAxle)

	#define RVA_SuspensionAxle_step 2918256

	void _step(float dt);

END_HOOK_OBJ()

void _SuspensionAxle::_step(float dt)
{
	mat44f mxAxle = this->axle->getWorldMatrix(0.0f);
	vec3f vAxleM1(mxAxle.M11, mxAxle.M12, mxAxle.M13);
	vec3f vAxleM4(mxAxle.M41, mxAxle.M42, mxAxle.M43);

	float fSideSign = ((int)this->side ? -1.0f : 1.0f);
	vec3f vAxleWorld = vadd(vmul(vAxleM1, fSideSign * this->track * this->attachRelativePos), vAxleM4);
	vec3f vAxleLocal = this->car->body->worldToLocal(vAxleWorld);

	vec3f vBase = this->getBasePosition();
	vBase.x *= this->attachRelativePos;
	vBase.y += 0.2f;
	vec3f vBaseWorld = this->car->body->localToWorld(vBase);

	vec3f vDelta = vsub(vBaseWorld, vAxleWorld);
	float fDeltaLen = vlen(vDelta);
	if (fDeltaLen != 0.0f)
	{
		vDelta = vnorm(vDelta);
	}

	float fTravel = (0.2f - fDeltaLen) + this->rodLength;
	this->status.travel = fTravel;

	float fForce = -(((fTravel * this->progressiveK) + this->k) * fTravel);
	if (fForce < 0.0f)
	{
		vec3f vForce = vmul(vDelta, fForce);
		this->addForceAtPos(vForce, vAxleWorld, false, false);

		vForce = vmul(vDelta, -fForce);
		this->car->body->addForceAtPos(vForce, vBaseWorld);
	}

	if (this->leafSpringK.x != 0.0f)
	{
		vBase = this->getBasePosition();
		fForce = (vAxleLocal.x - (this->attachRelativePos * vBase.x)) * this->leafSpringK.x;
		mat44f mxCarWorld = this->car->body->getWorldMatrix(0.0f);
		vec3f vM1 = vec3f(mxCarWorld.M11, mxCarWorld.M12, mxCarWorld.M13);

		vec3f vForce = vmul(vM1, -fForce);
		this->addForceAtPos(vForce, vAxleWorld, false, false);

		vForce = vec3f(fForce, 0.0f, 0.0f);
		this->car->body->addLocalForceAtLocalPos(vForce, vAxleLocal);
	}

	float fBumpStopUp = this->bumpStopUp;
	float fRefY = vAxleLocal.y - this->referenceY;
	if (fBumpStopUp != 0.0f && fRefY > fBumpStopUp && 0.0f != this->k)
	{
		float fForce = (fRefY - fBumpStopUp) * 500000.0f;
		mat44f mxCarWorld = this->car->body->getWorldMatrix(0.0f);
		vec3f vM2 = vec3f(mxCarWorld.M21, mxCarWorld.M22, mxCarWorld.M23);

		vec3f vForce = vmul(vM2, -fForce);
		this->addForceAtPos(vForce, vAxleWorld, false, false);

		vForce = vec3f(0.0f, fForce, 0.0f);
		this->car->body->addLocalForceAtLocalPos(vForce, vAxleLocal);
	}

	float fBumpStopDn = this->bumpStopDn;
	if (fBumpStopDn != 0.0f && fRefY < fBumpStopDn && 0.0f != this->k)
	{
		float fForce = (fRefY - fBumpStopDn) * 500000.0f;
		mat44f mxCarWorld = this->car->body->getWorldMatrix(0.0f);
		vec3f vM2 = vec3f(mxCarWorld.M21, mxCarWorld.M22, mxCarWorld.M23);

		vec3f vForce = vmul(vM2, -fForce);
		this->addForceAtPos(vForce, vAxleWorld, false, false);

		vForce = vec3f(0.0f, fForce, 0.0f);
		this->car->body->addLocalForceAtLocalPos(vForce, vAxleLocal);
	}

	vec3f vVel = this->getVelocity();
	vec3f vPointVel = this->car->body->getPointVelocity(vBaseWorld);
	vec3f vDeltaVel = vsub(vVel, vPointVel);

	float fSpeed = vdot(vDeltaVel, vDelta);
	this->status.damperSpeedMS = fSpeed;

	float fDamperForce = this->damper.getForce(fSpeed);

	vec3f vForce = vmul(vDelta, fDamperForce);
	this->addForceAtPos(vForce, vAxleWorld, false, false);

	vForce = vmul(vForce, -1.0f);
	this->car->body->addForceAtPos(vForce, vBaseWorld);
}
