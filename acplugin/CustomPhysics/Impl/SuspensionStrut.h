#pragma once

void SuspensionStrut_step(SuspensionStrut* pThis, float dt)
{
	pThis->steerTorque = 0.0f;

	vec3f vCarStrut = pThis->carBody->localToWorld(pThis->dataRelToBody.carStrut);
	vec3f vTyreStrut = pThis->hub->localToWorld(pThis->dataRelToWheel.tyreStrut);
	vec3f vDelta = vsub(vTyreStrut, vCarStrut);
	float fDelta = vlen(vDelta);

	if (fDelta != 0.0f) {
		vDelta = vmul(vDelta, 1.0f / fDelta);
	}

	float fDefaultLength = pThis->strutBaseLength + pThis->rodLength;
	float fTravel = fDefaultLength - fDelta;
	pThis->status.travel = fTravel;

	float fForce = ((fTravel * pThis->progressiveK) + pThis->k) * fTravel;
	if (fForce < 0) fForce = 0;

	if (pThis->packerRange != 0.0f && fTravel > pThis->packerRange)
		fForce += ((fTravel - pThis->packerRange) * pThis->bumpStopRate);

	if (fForce > 0)
	{
		vec3f vForce = vmul(vDelta, fForce);
		pThis->addForceAtPos(vForce, vTyreStrut, false, false);

		vForce = vmul(vForce, -1.0f);
		pThis->carBody->addForceAtPos(vForce, vCarStrut);
	}

	vec3f vHubWorld = pThis->hub->getPosition(0.0f);
	vec3f vHubLocal = pThis->carBody->worldToLocal(vHubWorld);
	mat44f mxCarWorld = pThis->carBody->getWorldMatrix(0.0f);

	float fHubDelta = vHubLocal.y - pThis->dataRelToWheel.refPoint.y;

	if (fHubDelta > pThis->bumpStopUp)
	{
		fForce = (fHubDelta - pThis->bumpStopUp) * 500000.0f;

		vec3f vForce = vmul(vec3f(mxCarWorld.M21, mxCarWorld.M22, mxCarWorld.M23), -fForce);
		vec3f vPos = pThis->hub->getPosition(0.0f);
		pThis->addForceAtPos(vForce, vPos, false, false);

		vForce = vec3f(0, fForce, 0);
		pThis->carBody->addLocalForceAtLocalPos(vForce, vHubLocal);
	}

	if (fHubDelta < pThis->bumpStopDn)
	{
		fForce = (fHubDelta - pThis->bumpStopDn) * 500000.0f;

		vec3f vForce = vmul(vec3f(mxCarWorld.M21, mxCarWorld.M22, mxCarWorld.M23), -fForce);
		vec3f vPos = pThis->hub->getPosition(0.0f);
		pThis->addForceAtPos(vForce, vPos, false, false);

		vForce = vec3f(0, fForce, 0);
		pThis->carBody->addLocalForceAtLocalPos(vForce, vHubLocal);
	}

	vec3f vTyreStrutVel = pThis->hub->getLocalPointVelocity(pThis->dataRelToWheel.tyreStrut);
	vec3f vCarStrutVel = pThis->carBody->getLocalPointVelocity(pThis->dataRelToBody.carStrut);
	vec3f vDamperDelta = vsub(vTyreStrutVel, vCarStrutVel);

	float fDamperSpeed = vdot(vDamperDelta, vDelta);
	float fDamperForce = pThis->damper.getForce(fDamperSpeed);
	pThis->status.damperSpeedMS = fDamperSpeed;

	vec3f vDamperForce = vmul(vDelta, fDamperForce);
	pThis->addForceAtPos(vDamperForce, vTyreStrut, false, false);

	vDamperForce = vmul(vDamperForce, -1.0f);
	pThis->carBody->addForceAtPos(vDamperForce, vCarStrut);
}
