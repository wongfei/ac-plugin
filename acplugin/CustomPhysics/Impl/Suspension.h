#pragma once

BEGIN_HOOK_OBJ(Suspension)

	#define RVA_Suspension_step 2896784

	void _step(float dt);

END_HOOK_OBJ()

void _Suspension::_step(float dt)
{
	this->steerTorque = 0.0f;

	vec3f vHubWorldPos = this->hub->getPosition(0.0f);
	vec3f vHubLocalPos = this->carBody->worldToLocal(vHubWorldPos);

	mat44f mxBody = this->carBody->getWorldMatrix(0.0f);
	vec3f vBodyM2(mxBody.M21, mxBody.M22, mxBody.M23);

	float fHubDeltaY = vHubLocalPos.y - this->dataRelToWheel.refPoint.y;
	float fTravel = fHubDeltaY + this->rodLength;
	this->status.travel = fTravel;

	if (this->useActiveActuator)
	{
		this->activeActuator.targetTravel = 0.0f;
		float fActiveActuator = this->activeActuator.eval(dt, fHubDeltaY);

		this->car->antirollBars[0].k = 0.0f;
		this->car->antirollBars[1].k = 0.0f;

		vec3f vForce(vBodyM2 * fActiveActuator);
		this->addForceAtPos(vForce, vHubWorldPos, false, false);
		this->carBody->addForceAtLocalPos(vForce * -1.0f, this->dataRelToWheel.refPoint);

		this->status.travel = fHubDeltaY;
	}
	else
	{
		float fK = this->k;
		float fPackerRange = this->packerRange;
		float fDelta1 = ((fTravel * this->progressiveK) + fK) * fTravel;

		if (fPackerRange != 0.0f && fTravel > fPackerRange && fK != 0.0f)
		{
			fDelta1 += (((fTravel - fPackerRange) * this->bumpStopProgressive) + this->bumpStopRate) * (fTravel - fPackerRange);
		}

		if (fDelta1 > 0.0)
		{
			this->addForceAtPos(vBodyM2 * -fDelta1, vHubWorldPos, false, false);
			this->carBody->addLocalForceAtLocalPos(vec3f(0, fDelta1, 0), this->dataRelToWheel.refPoint);
		}

		vec3f vHubVel = this->hub->getVelocity();
		vec3f vPointVel = this->carBody->getLocalPointVelocity(this->dataRelToWheel.refPoint);
		vec3f vDeltaVel(vHubVel - vPointVel);

		float fDamperSpeed = vDeltaVel * vBodyM2;
		this->status.damperSpeedMS = fDamperSpeed;

		float fDamperForce = this->damper.getForce(fDamperSpeed);
		vec3f vForce(vBodyM2 * fDamperForce);

		this->addForceAtPos(vForce, vHubWorldPos, false, false);
		this->carBody->addForceAtLocalPos(vForce * -1.0f, this->dataRelToWheel.refPoint);
	}

	float fBumpStopUp = this->bumpStopUp;
	if (fBumpStopUp != 0.0f && fHubDeltaY > fBumpStopUp && 0.0f != this->k)
	{
		float fDelta2 = (((fHubDeltaY - fBumpStopUp) * this->bumpStopProgressive) + this->bumpStopRate) * (fHubDeltaY - fBumpStopUp);
		this->addForceAtPos(vBodyM2 * -fDelta2, vHubWorldPos, false, false);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fDelta2, 0), vHubLocalPos);
	}

	float fBumpStopDown = this->bumpStopDn;
	if (fBumpStopDown != 0.0f && fHubDeltaY < fBumpStopDown && 0.0f != this->k)
	{
		float fDelta3 = (((fHubDeltaY - fBumpStopDown) * this->bumpStopProgressive) + this->bumpStopRate) * (fHubDeltaY - fBumpStopDown);
		this->addForceAtPos(vBodyM2 * -fDelta3, vHubWorldPos, false, false);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fDelta3, 0), vHubLocalPos);
	}
}
