#include "precompiled.h"
#include "GameHooks.h"

void Suspension_step(Suspension* pThis, float dt)
{
	pThis->steerTorque = 0.0f;

	vec3f worldHubPos = pThis->hub->getPosition(0.0f);
	vec3f localHubPos = pThis->carBody->worldToLocal(worldHubPos);
	mat44f carBodyM = pThis->carBody->getWorldMatrix(0.0f);

	float M4 = carBodyM.M21;
	float M5 = carBodyM.M22;
	float M6 = carBodyM.M23;

	float fHubDeltaY = localHubPos.y - pThis->dataRelToWheel.refPoint.y;
	float fTravel = fHubDeltaY + pThis->rodLength;
	pThis->status.travel = fTravel;

	if (pThis->useActiveActuator)
	{
		pThis->activeActuator.targetTravel = 0.0f;
		float fActiveActuator = pThis->activeActuator.eval(dt, fHubDeltaY);

		pThis->car->antirollBars[0].k = 0.0f;
		pThis->car->antirollBars[1].k = 0.0f;

		vec3f force;
		force.x = M4 * fActiveActuator;
		force.y = M5 * fActiveActuator;
		force.z = M6 * fActiveActuator;
		pThis->addForceAtPos(force, worldHubPos, false, false);

		vec3f floc;
		floc.x = -force.x;
		floc.y = -force.y;
		floc.z = -force.z;
		pThis->carBody->addForceAtLocalPos(floc, pThis->dataRelToWheel.refPoint);

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
			vec3f force;
			force.x = M4 * -fDelta1;
			force.y = M5 * -fDelta1;
			force.z = M6 * -fDelta1;
			pThis->addForceAtPos(force, worldHubPos, false, false);

			vec3f floc;
			floc.x = 0.0f;
			floc.y = fDelta1;
			floc.z = 0.0f;
			pThis->carBody->addLocalForceAtLocalPos(floc, pThis->dataRelToWheel.refPoint);
		}

		vec3f hubVel = pThis->hub->getVelocity();
		vec3f localBodyVel = pThis->carBody->getLocalPointVelocity(pThis->dataRelToWheel.refPoint);
		float fDamperSpeed = (((hubVel.y - localBodyVel.y) * M5) + ((hubVel.x - localBodyVel.x) * M4)) + ((hubVel.z - localBodyVel.z) * M6);
		float fDamperForce = pThis->damper.getForce(fDamperSpeed);
		pThis->status.damperSpeedMS = fDamperSpeed;

		vec3f force;
		force.x = M4 * fDamperForce;
		force.y = M5 * fDamperForce;
		force.z = M6 * fDamperForce;
		pThis->addForceAtPos(force, worldHubPos, false, false);

		vec3f floc;
		floc.x = -force.x;
		floc.y = -force.y;
		floc.z = -force.z;
		pThis->carBody->addForceAtLocalPos(floc, pThis->dataRelToWheel.refPoint);
	}

	float fBumpStopUp = pThis->bumpStopUp;
	if (fBumpStopUp != 0.0f && fHubDeltaY > fBumpStopUp && 0.0f != pThis->k)
	{
		float fDelta2 = (((fHubDeltaY - fBumpStopUp) * pThis->bumpStopProgressive) + pThis->bumpStopRate) * (fHubDeltaY - fBumpStopUp);

		vec3f force;
		force.x = M4 * -fDelta2;
		force.y = M5 * -fDelta2;
		force.z = M6 * -fDelta2;
		pThis->addForceAtPos(force, worldHubPos, false, false);

		vec3f floc;
		floc.x = 0.0f;
		floc.y = fDelta2;
		floc.z = 0.0f;
		pThis->carBody->addLocalForceAtLocalPos(floc, localHubPos);
	}

	float fBumpStopDown = pThis->bumpStopDn;
	if (fBumpStopDown != 0.0f && fHubDeltaY < fBumpStopDown && 0.0f != pThis->k)
	{
		float fDelta3 = (((fHubDeltaY - fBumpStopDown) * pThis->bumpStopProgressive) + pThis->bumpStopRate) * (fHubDeltaY - fBumpStopDown);

		vec3f force;
		force.x = M4 * -fDelta3;
		force.y = M5 * -fDelta3;
		force.z = M6 * -fDelta3;
		pThis->addForceAtPos(force, worldHubPos, false, false);

		vec3f floc;
		floc.x = 0.0f;
		floc.y = fDelta3;
		floc.z = 0.0f;
		pThis->carBody->addLocalForceAtLocalPos(floc, localHubPos);
	}
}
