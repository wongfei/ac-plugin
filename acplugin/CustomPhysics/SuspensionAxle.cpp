#include "precompiled.h"
#include "GameHooks.h"

void SuspensionAxle_step(SuspensionAxle* pThis, float dt)
{
	mat44f mxAxle = pThis->axle->getWorldMatrix(0.0f);
	vec3f vAxleM1 = makev(mxAxle.M11, mxAxle.M12, mxAxle.M13);
	vec3f vAxleM4 = makev(mxAxle.M41, mxAxle.M42, mxAxle.M43);

	float fSideSign = ((int)pThis->side ? -1.0f : 1.0f);
	vec3f vAxleWorld = vadd(vmul(vAxleM1, pThis->track * fSideSign * pThis->attachRelativePos), vAxleM4);
	vec3f vAxleLocal = pThis->car->body->worldToLocal(vAxleWorld);

	vec3f vBase = pThis->getBasePosition();
	vBase.x *= pThis->attachRelativePos;
	vBase.y += 0.2f;
	vec3f vBaseWorld = pThis->car->body->localToWorld(vBase);

	vec3f vDelta = vsub(vBaseWorld, vAxleWorld);
	float fDeltaLen = vlen(vDelta);
	if (fDeltaLen != 0.0f)
	{
		vDelta = vnorm(vDelta);
	}

	float fTravel = (0.2f - fDeltaLen) + pThis->rodLength;
	pThis->status.travel = fTravel;

	float fForce = -(((fTravel * pThis->progressiveK) + pThis->k) * fTravel);
	if (fForce < 0.0f)
	{
		vec3f vForce = vmul(vDelta, fForce);
		pThis->addForceAtPos(vForce, vAxleWorld, false, false);

		vForce = vmul(vDelta, -fForce);
		pThis->car->body->addForceAtPos(vForce, vBaseWorld);
	}

	if (pThis->leafSpringK.x != 0.0f)
	{
		vBase = pThis->getBasePosition();
		fForce = (vAxleLocal.x - (pThis->attachRelativePos * vBase.x)) * pThis->leafSpringK.x;
		mat44f mxCarWorld = pThis->car->body->getWorldMatrix(0.0f);
		vec3f vM1 = makev(mxCarWorld.M11, mxCarWorld.M12, mxCarWorld.M13);

		vec3f vForce = vmul(vM1, -fForce);
		pThis->addForceAtPos(vForce, vAxleWorld, false, false);

		vForce = makev(fForce, 0.0f, 0.0f);
		pThis->car->body->addLocalForceAtLocalPos(vForce, vAxleLocal);
	}

	float fBumpStopUp = pThis->bumpStopUp;
	float fRefY = vAxleLocal.y - pThis->referenceY;
	if (fBumpStopUp != 0.0f && fRefY > fBumpStopUp && 0.0f != pThis->k)
	{
		float fForce = (fRefY - fBumpStopUp) * 500000.0f;
		mat44f mxCarWorld = pThis->car->body->getWorldMatrix(0.0f);
		vec3f vM2 = makev(mxCarWorld.M21, mxCarWorld.M22, mxCarWorld.M23);

		vec3f vForce = vmul(vM2, -fForce);
		pThis->addForceAtPos(vForce, vAxleWorld, false, false);

		vForce = makev(0.0f, fForce, 0.0f);
		pThis->car->body->addLocalForceAtLocalPos(vForce, vAxleLocal);
	}

	float fBumpStopDn = pThis->bumpStopDn;
	if (fBumpStopDn != 0.0f && fRefY < fBumpStopDn && 0.0f != pThis->k)
	{
		float fForce = (fRefY - fBumpStopDn) * 500000.0f;
		mat44f mxCarWorld = pThis->car->body->getWorldMatrix(0.0f);
		vec3f vM2 = makev(mxCarWorld.M21, mxCarWorld.M22, mxCarWorld.M23);

		vec3f vForce = vmul(vM2, -fForce);
		pThis->addForceAtPos(vForce, vAxleWorld, false, false);

		vForce = makev(0.0f, fForce, 0.0f);
		pThis->car->body->addLocalForceAtLocalPos(vForce, vAxleLocal);
	}

	vec3f vVel = pThis->getVelocity();
	vec3f vPointVel = pThis->car->body->getPointVelocity(vBaseWorld);
	vec3f vDeltaVel = vsub(vVel, vPointVel);

	vDeltaVel.x *= vDelta.x;
	vDeltaVel.y *= vDelta.y;
	vDeltaVel.z *= vDelta.z;

	float fSpeed = vDeltaVel.x + vDeltaVel.y + vDeltaVel.z;
	pThis->status.damperSpeedMS = fSpeed;

	float fDamperForce = pThis->damper.getForce(fSpeed);

	vec3f vForce = vmul(vDelta, fDamperForce);
	pThis->addForceAtPos(vForce, vAxleWorld, false, false);

	vForce = vmul(vForce, -1.0f);
	pThis->car->body->addForceAtPos(vForce, vBaseWorld);
}
