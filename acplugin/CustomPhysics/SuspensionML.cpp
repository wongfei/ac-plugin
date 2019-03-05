#include "precompiled.h"
#include "GameHooks.h"

void SuspensionML_step(SuspensionML* pThis, float dt)
{
	pThis->steerTorque = 0.0;

	vec3f vHubWorld = pThis->hub->getPosition(0.0f);
	vec3f vHubLocal = pThis->car->body->worldToLocal(vHubWorld);

	mat44f mxCar = pThis->car->body->getWorldMatrix(0.0f);
	vec3f vM2 = makev(mxCar.M21, mxCar.M22, mxCar.M23);

	float fK = pThis->k;
	float fPackerRange = pThis->packerRange;
	float fTravel = (vHubLocal.y - pThis->basePosition.y) + pThis->rodLength;
	pThis->status.travel = fTravel;

	float fForce = (fTravel * pThis->progressiveK + fK) * fTravel;
	if (fPackerRange != 0.0f && fTravel > fPackerRange && fK != 0.0f)
	{
		fForce = fForce + ((fTravel - fPackerRange) * pThis->bumpStopRate);
	}

	vec3f vForce = vmul(vM2, -fForce);
	pThis->addForceAtPos(vForce, vHubWorld, false, false);

	vForce = makev(0.0f, fForce, 0.0f);
	pThis->car->body->addLocalForceAtLocalPos(vForce, pThis->basePosition);

	vec3f vVel = pThis->hub->getVelocity();
	vec3f vPointVel = pThis->car->body->getLocalPointVelocity(pThis->basePosition);
	vec3f vDeltaVel = vsub(vVel, vPointVel);

	vDeltaVel.x *= vM2.x;
	vDeltaVel.y *= vM2.y;
	vDeltaVel.z *= vM2.z;

	float fSpeed = vDeltaVel.x + vDeltaVel.y + vDeltaVel.z;
	pThis->status.damperSpeedMS = fSpeed;

	float fDamperForce = pThis->damper.getForce(fSpeed);

	vForce = vmul(vM2, fDamperForce);
	pThis->addForceAtPos(vForce, vHubWorld, false, false);

	vForce = vmul(vForce, -1.0f);
	pThis->car->body->addForceAtLocalPos(vForce, pThis->basePosition);
}
