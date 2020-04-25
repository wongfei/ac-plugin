#pragma once

BEGIN_HOOK_OBJ(HeaveSpring)

	#define RVA_HeaveSpring_step 2832736

	void _step(float dt);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

void _HeaveSpring::_step(float dt)
{
	mat44f mxBodyWorld = this->car->body->getWorldMatrix(0.0f);
	vec3f vM2 = vec3f(mxBodyWorld.M21, mxBodyWorld.M22, mxBodyWorld.M23);

	auto* pSusp0 = this->suspensions[0];
	auto* pSusp1 = this->suspensions[1];

	vec3f vRefPoint0 = pSusp0->dataRelToWheel.refPoint;
	vec3f vRefPoint1 = pSusp1->dataRelToWheel.refPoint;

	vec3f vHubPos0 = pSusp0->hub->getPosition(0.0f);
	vec3f vHubPos1 = pSusp1->hub->getPosition(0.0f);
	vec3f vHubLoc0 = this->car->body->worldToLocal(vHubPos0);
	vec3f vHubLoc1 = this->car->body->worldToLocal(vHubPos1);

	//

	if (pSusp0->k != 0.0f || pSusp1->k != 0.0f)
		this->rodLength = (pSusp1->rodLength + pSusp0->rodLength) * 0.5f;

	float fAvgY = (vHubLoc0.y + vHubLoc1.y) * 0.5f;
	float fTravel = (fAvgY - pSusp0->dataRelToWheel.refPoint.y) + this->rodLength;
	this->status.travel = fTravel;

	//

	float v12 = ((fTravel * this->progressiveK) + this->k) * fTravel;

	if (this->packerRange != 0.0f && fTravel > this->packerRange)
		v12 += ((fTravel - this->packerRange) * this->bumpStopRate);

	vec3f vForce = vM2 * -v12;
	pSusp0->addForceAtPos(vForce, pSusp0->hub->getPosition(0.0f), false, false);
	pSusp1->addForceAtPos(vForce, pSusp1->hub->getPosition(0.0f), false, false);

	this->car->body->addLocalForceAtLocalPos(vec3f(0, v12, 0), vRefPoint0);
	this->car->body->addLocalForceAtLocalPos(vec3f(0, v12, 0), vRefPoint1);

	//

	float fBumpStopUp = this->bumpStopUp;
	float fBumpStopDn = this->bumpStopDn;
	float fDeltaY0 = fAvgY - pSusp0->dataRelToWheel.refPoint.y;

	if (fBumpStopUp != 0.0f && fDeltaY0 > fBumpStopUp)
	{
		float fForce = (fDeltaY0 - fBumpStopUp) * 500000.0f;

		vForce = vM2 * -fForce;
		pSusp0->addForceAtPos(vForce, pSusp0->hub->getPosition(0.0f), false, false);
		pSusp1->addForceAtPos(vForce, pSusp1->hub->getPosition(0.0f), false, false);

		vForce = vec3f(0, fForce, 0);
		this->car->body->addLocalForceAtLocalPos(vForce, vRefPoint0);
		this->car->body->addLocalForceAtLocalPos(vForce, vRefPoint1);
	}

	//

	if (fBumpStopDn != 0.0f && fDeltaY0 < fBumpStopDn)
	{
		float fForce = (fDeltaY0 - fBumpStopDn) * 500000.0f;

		vForce = vM2 * -fForce;
		pSusp0->addForceAtPos(vForce, pSusp0->hub->getPosition(0.0f), false, false);
		pSusp1->addForceAtPos(vForce, pSusp1->hub->getPosition(0.0f), false, false);

		vForce = vec3f(0, fForce, 0);
		this->car->body->addLocalForceAtLocalPos(vForce, vRefPoint0);
		this->car->body->addLocalForceAtLocalPos(vForce, vRefPoint1);
	}

	//

	vec3f vHubVel0 = pSusp0->hub->getVelocity();
	vec3f vHubVel1 = pSusp1->hub->getVelocity();
	vec3f vHubVel = (vHubVel0 + vHubVel1) * 0.5f;

	vec3f vLpv0 = this->car->body->getLocalPointVelocity(vRefPoint0);
	vec3f vLpv1 = this->car->body->getLocalPointVelocity(vRefPoint1);
	vec3f vLpv = (vLpv0 + vLpv1) * 0.5f;

	float v = 
		(vHubVel.x - vLpv.x) * vM2.x + 
		(vHubVel.y - vLpv.y) * vM2.y + 
		(vHubVel.z - vLpv.z) * vM2.z;

	float fDamperForce = this->damper.getForce(v);

	vForce = vM2 * fDamperForce;
	pSusp0->addForceAtPos(vForce, pSusp0->hub->getPosition(0.0f), false, false);
	pSusp1->addForceAtPos(vForce, pSusp1->hub->getPosition(0.0f), false, false);

	vForce *= -1.0f;
	this->car->body->addLocalForceAtLocalPos(vForce, vRefPoint0);
	this->car->body->addLocalForceAtLocalPos(vForce, vRefPoint1);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
