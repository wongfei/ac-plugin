#pragma once

BEGIN_HOOK_OBJ(Suspension)

	#define RVA_Suspension_step 2896784

	void _step(float dt);

END_HOOK_OBJ()

void _Suspension::_step(float dt)
{
	this->steerTorque = 0.0f;

	mat44f mxBody = this->carBody->getWorldMatrix(0.0f);
	vec3f vBodyM2(&mxBody.M21);

	vec3f vHubWorldPos = this->hub->getPosition(0.0f);
	vec3f vHubLocalPos = this->carBody->worldToLocal(vHubWorldPos);

	float fHubDeltaY = vHubLocalPos.y - this->dataRelToWheel.refPoint.y;
	float fTravel = fHubDeltaY + this->rodLength;
	this->status.travel = fTravel;

	if (this->useActiveActuator)
	{
		this->activeActuator.targetTravel = 0.0f;
		float fActiveActuator = this->activeActuator.eval(dt, fHubDeltaY);

		this->car->antirollBars[0].k = 0.0f;
		this->car->antirollBars[1].k = 0.0f;

		vec3f vForce = vBodyM2 * fActiveActuator;
		this->addForceAtPos(vForce, vHubWorldPos, false, false);
		this->carBody->addForceAtLocalPos(vForce * -1.0f, this->dataRelToWheel.refPoint);

		this->status.travel = fHubDeltaY;
	}
	else
	{
		float fDelta1 = ((fTravel * this->progressiveK) + this->k) * fTravel;
		if (this->packerRange != 0.0f && fTravel > this->packerRange && this->k != 0.0f)
		{
			fDelta1 += (((fTravel - this->packerRange) * this->bumpStopProgressive) + this->bumpStopRate) * (fTravel - this->packerRange);
		}

		if (fDelta1 > 0.0f)
		{
			this->addForceAtPos(vBodyM2 * -fDelta1, vHubWorldPos, false, false);
			this->carBody->addLocalForceAtLocalPos(vec3f(0, fDelta1, 0), this->dataRelToWheel.refPoint);
		}

		vec3f vHubVel = this->hub->getVelocity();
		vec3f vPointVel = this->carBody->getLocalPointVelocity(this->dataRelToWheel.refPoint);
		vec3f vDeltaVel = vHubVel - vPointVel;

		float fDamperSpeed = vBodyM2 * vDeltaVel;
		this->status.damperSpeedMS = fDamperSpeed;

		float fDamperForce = this->damper.getForce(fDamperSpeed);
		vec3f vForce = vBodyM2 * fDamperForce;
		this->addForceAtPos(vForce, vHubWorldPos, false, false);
		this->carBody->addForceAtLocalPos(vForce * -1.0f, this->dataRelToWheel.refPoint);
	}

	if (this->bumpStopUp != 0.0f && fHubDeltaY > this->bumpStopUp && 0.0f != this->k)
	{
		float fDelta2 = (((fHubDeltaY - this->bumpStopUp) * this->bumpStopProgressive) + this->bumpStopRate) * (fHubDeltaY - this->bumpStopUp);
		this->addForceAtPos(vBodyM2 * -fDelta2, vHubWorldPos, false, false);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fDelta2, 0), vHubLocalPos);
	}

	if (this->bumpStopDn != 0.0f && fHubDeltaY < this->bumpStopDn && 0.0f != this->k)
	{
		float fDelta3 = (((fHubDeltaY - this->bumpStopDn) * this->bumpStopProgressive) + this->bumpStopRate) * (fHubDeltaY - this->bumpStopDn);
		this->addForceAtPos(vBodyM2 * -fDelta3, vHubWorldPos, false, false);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fDelta3, 0), vHubLocalPos);
	}
}
