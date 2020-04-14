#pragma once

#define RVA_Suspension_step 2896784
#define RVA_SuspensionAxle_step 2918256
#define RVA_SuspensionML_step 2927360
#define RVA_SuspensionStrut_step 2909696

void Suspension_step(Suspension* pThis, float dt)
{
	pThis->steerTorque = 0.0f;

	vec3f vHubWorldPos = pThis->hub->getPosition(0.0f);
	vec3f vHubLocalPos = pThis->carBody->worldToLocal(vHubWorldPos);

	mat44f mxBody = pThis->carBody->getWorldMatrix(0.0f);
	vec3f vBodyM2(mxBody.M21, mxBody.M22, mxBody.M23);

	float fHubDeltaY = vHubLocalPos.y - pThis->dataRelToWheel.refPoint.y;
	float fTravel = fHubDeltaY + pThis->rodLength;
	pThis->status.travel = fTravel;

	if (pThis->useActiveActuator)
	{
		pThis->activeActuator.targetTravel = 0.0f;
		float fActiveActuator = pThis->activeActuator.eval(dt, fHubDeltaY);

		pThis->car->antirollBars[0].k = 0.0f;
		pThis->car->antirollBars[1].k = 0.0f;

		vec3f vForce(vBodyM2 * fActiveActuator);
		pThis->addForceAtPos(vForce, vHubWorldPos, false, false);
		pThis->carBody->addForceAtLocalPos(vForce * -1.0f, pThis->dataRelToWheel.refPoint);

		pThis->status.travel = fHubDeltaY;
	}
	else
	{
		float fK = pThis->k;
		float fPackerRange = pThis->packerRange;
		float fDelta1 = ((fTravel * pThis->progressiveK) + fK) * fTravel;

		if (fPackerRange != 0.0f && fTravel > fPackerRange && fK != 0.0f)
		{
			fDelta1 += (((fTravel - fPackerRange) * pThis->bumpStopProgressive) + pThis->bumpStopRate) * (fTravel - fPackerRange);
		}

		if (fDelta1 > 0.0)
		{
			pThis->addForceAtPos(vBodyM2 * -fDelta1, vHubWorldPos, false, false);
			pThis->carBody->addLocalForceAtLocalPos(vec3f(0, fDelta1, 0), pThis->dataRelToWheel.refPoint);
		}

		vec3f vHubVel = pThis->hub->getVelocity();
		vec3f vPointVel = pThis->carBody->getLocalPointVelocity(pThis->dataRelToWheel.refPoint);
		vec3f vDeltaVel(vHubVel - vPointVel);

		float fDamperSpeed = vDeltaVel * vBodyM2;
		pThis->status.damperSpeedMS = fDamperSpeed;

		float fDamperForce = pThis->damper.getForce(fDamperSpeed);
		vec3f vForce(vBodyM2 * fDamperForce);

		pThis->addForceAtPos(vForce, vHubWorldPos, false, false);
		pThis->carBody->addForceAtLocalPos(vForce * -1.0f, pThis->dataRelToWheel.refPoint);
	}

	float fBumpStopUp = pThis->bumpStopUp;
	if (fBumpStopUp != 0.0f && fHubDeltaY > fBumpStopUp && 0.0f != pThis->k)
	{
		float fDelta2 = (((fHubDeltaY - fBumpStopUp) * pThis->bumpStopProgressive) + pThis->bumpStopRate) * (fHubDeltaY - fBumpStopUp);
		pThis->addForceAtPos(vBodyM2 * -fDelta2, vHubWorldPos, false, false);
		pThis->carBody->addLocalForceAtLocalPos(vec3f(0, fDelta2, 0), vHubLocalPos);
	}

	float fBumpStopDown = pThis->bumpStopDn;
	if (fBumpStopDown != 0.0f && fHubDeltaY < fBumpStopDown && 0.0f != pThis->k)
	{
		float fDelta3 = (((fHubDeltaY - fBumpStopDown) * pThis->bumpStopProgressive) + pThis->bumpStopRate) * (fHubDeltaY - fBumpStopDown);
		pThis->addForceAtPos(vBodyM2 * -fDelta3, vHubWorldPos, false, false);
		pThis->carBody->addLocalForceAtLocalPos(vec3f(0, fDelta3, 0), vHubLocalPos);
	}
}
