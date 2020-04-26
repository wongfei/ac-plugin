#pragma once

BEGIN_HOOK_OBJ(SuspensionAxle)

	#define RVA_SuspensionAxle_step 2918256

	void _step(float dt);

END_HOOK_OBJ()

void _SuspensionAxle::_step(float dt)
{
	mat44f mxCarWorld = this->car->body->getWorldMatrix(0.0f);

	mat44f mxAxle = this->axle->getWorldMatrix(0.0f);
	vec3f vAxleM1(&mxAxle.M11);
	vec3f vAxleM4(&mxAxle.M41);

	float fSideSign = ((int)this->side ? -1.0f : 1.0f);
	vec3f vAxleWorld = (vAxleM1 * (fSideSign * this->track * this->attachRelativePos)) + vAxleM4;
	vec3f vAxleLocal = this->car->body->worldToLocal(vAxleWorld);

	vec3f vBase = this->getBasePosition();
	vBase.x *= this->attachRelativePos;
	vBase.y += 0.2f;
	vec3f vBaseWorld = this->car->body->localToWorld(vBase);

	vec3f vDelta = vBaseWorld - vAxleWorld;
	float fDeltaLen = vlen(vDelta);
	vDelta = vnorm(vDelta, fDeltaLen);

	float fTravel = (0.2f - fDeltaLen) + this->rodLength;
	this->status.travel = fTravel;

	float fForce = -(((fTravel * this->progressiveK) + this->k) * fTravel);
	if (fForce < 0.0f)
	{
		this->addForceAtPos(vDelta * fForce, vAxleWorld, false, false);
		this->car->body->addForceAtPos(vDelta * -fForce, vBaseWorld);
	}

	if (this->leafSpringK.x != 0.0f)
	{
		fForce = (vAxleLocal.x - (this->attachRelativePos * this->getBasePosition().x)) * this->leafSpringK.x;
		this->addForceAtPos(vec3f(&mxCarWorld.M11) * -fForce, vAxleWorld, false, false);
		this->car->body->addLocalForceAtLocalPos(vec3f(fForce, 0.0f, 0.0f), vAxleLocal);
	}

	float fBumpStopUp = this->bumpStopUp;
	float fRefY = vAxleLocal.y - this->referenceY;
	if (fBumpStopUp != 0.0f && fRefY > fBumpStopUp && 0.0f != this->k)
	{
		fForce = (fRefY - fBumpStopUp) * 500000.0f;
		this->addForceAtPos(vec3f(&mxCarWorld.M21) * -fForce, vAxleWorld, false, false);
		this->car->body->addLocalForceAtLocalPos(vec3f(0.0f, fForce, 0.0f), vAxleLocal);
	}

	float fBumpStopDn = this->bumpStopDn;
	if (fBumpStopDn != 0.0f && fRefY < fBumpStopDn && 0.0f != this->k)
	{
		fForce = (fRefY - fBumpStopDn) * 500000.0f;
		this->addForceAtPos(vec3f(&mxCarWorld.M21) * -fForce, vAxleWorld, false, false);
		this->car->body->addLocalForceAtLocalPos(vec3f(0.0f, fForce, 0.0f), vAxleLocal);
	}

	vec3f vPointVel = this->car->body->getPointVelocity(vBaseWorld);
	vec3f vDeltaVel = this->getVelocity() - vPointVel;

	float fSpeed = vdot(vDeltaVel, vDelta);
	this->status.damperSpeedMS = fSpeed;

	float fDamperForce = this->damper.getForce(fSpeed);
	vec3f vForce = vDelta * fDamperForce;
	this->addForceAtPos(vForce, vAxleWorld, false, false);
	this->car->body->addForceAtPos(vForce * -1.0f, vBaseWorld);
}
