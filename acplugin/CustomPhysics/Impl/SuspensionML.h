#pragma once

BEGIN_HOOK_OBJ(SuspensionML)

	#define RVA_SuspensionML_step 2927360

	void _step(float dt);

END_HOOK_OBJ()

void _SuspensionML::_step(float dt)
{
	this->steerTorque = 0.0;

	mat44f mxCarWorld = this->car->body->getWorldMatrix(0.0f);
	vec3f vM2(&mxCarWorld.M21);

	vec3f vHubWorld = this->hub->getPosition(0.0f);
	vec3f vHubLocal = this->car->body->worldToLocal(vHubWorld);

	float fTravel = (vHubLocal.y - this->basePosition.y) + this->rodLength;
	this->status.travel = fTravel;

	float fForce = (fTravel * this->progressiveK + this->k) * fTravel;
	if (this->packerRange != 0.0f && fTravel > this->packerRange && this->k != 0.0f)
	{
		fForce += ((fTravel - this->packerRange) * this->bumpStopRate);
	}

	this->addForceAtPos(vM2 * -fForce, vHubWorld, false, false);
	this->car->body->addLocalForceAtLocalPos(vec3f(0.0f, fForce, 0.0f), this->basePosition);

	vec3f vPointVel = this->car->body->getLocalPointVelocity(this->basePosition);
	vec3f vDeltaVel = this->hub->getVelocity() - vPointVel;

	float fSpeed = vdot(vDeltaVel, vM2);
	this->status.damperSpeedMS = fSpeed;

	float fDamperForce = this->damper.getForce(fSpeed);
	vec3f vForce = vM2 * fDamperForce;
	this->addForceAtPos(vForce, vHubWorld, false, false);
	this->car->body->addForceAtLocalPos(vForce * -1.0f, this->basePosition);
}
