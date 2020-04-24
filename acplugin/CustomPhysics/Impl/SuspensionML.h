#pragma once

BEGIN_HOOK_OBJ(SuspensionML)

	#define RVA_SuspensionML_step 2927360

	void _step(float dt);

END_HOOK_OBJ()

void _SuspensionML::_step(float dt)
{
	this->steerTorque = 0.0;

	vec3f vHubWorld = this->hub->getPosition(0.0f);
	vec3f vHubLocal = this->car->body->worldToLocal(vHubWorld);

	mat44f mxCar = this->car->body->getWorldMatrix(0.0f);
	vec3f vM2 = vec3f(mxCar.M21, mxCar.M22, mxCar.M23);

	float fK = this->k;
	float fPackerRange = this->packerRange;
	float fTravel = (vHubLocal.y - this->basePosition.y) + this->rodLength;
	this->status.travel = fTravel;

	float fForce = (fTravel * this->progressiveK + fK) * fTravel;
	if (fPackerRange != 0.0f && fTravel > fPackerRange && fK != 0.0f)
	{
		fForce = fForce + ((fTravel - fPackerRange) * this->bumpStopRate);
	}

	vec3f vForce = vmul(vM2, -fForce);
	this->addForceAtPos(vForce, vHubWorld, false, false);

	vForce = vec3f(0.0f, fForce, 0.0f);
	this->car->body->addLocalForceAtLocalPos(vForce, this->basePosition);

	vec3f vVel = this->hub->getVelocity();
	vec3f vPointVel = this->car->body->getLocalPointVelocity(this->basePosition);
	vec3f vDeltaVel = vsub(vVel, vPointVel);

	float fSpeed = vdot(vDeltaVel, vM2);
	this->status.damperSpeedMS = fSpeed;

	float fDamperForce = this->damper.getForce(fSpeed);

	vForce = vmul(vM2, fDamperForce);
	this->addForceAtPos(vForce, vHubWorld, false, false);

	vForce = vmul(vForce, -1.0f);
	this->car->body->addForceAtLocalPos(vForce, this->basePosition);
}
