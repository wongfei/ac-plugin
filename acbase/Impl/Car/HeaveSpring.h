#pragma once

BEGIN_HOOK_OBJ(HeaveSpring)

	#define RVA_HeaveSpring_ctor 2546416
	#define RVA_HeaveSpring_init 2831120
	#define RVA_HeaveSpring_initData 2831168
	#define RVA_HeaveSpring_step 2832736

	static void _hook()
	{
		HOOK_METHOD_RVA(HeaveSpring, ctor);
		HOOK_METHOD_RVA(HeaveSpring, init);
		HOOK_METHOD_RVA(HeaveSpring, initData);
		HOOK_METHOD_RVA(HeaveSpring, step); // (test on F138)
	}

	HeaveSpring* _ctor();
	void _init(Car* car, Suspension* s0, Suspension* s1, bool isFront);
	void _initData();
	void _step(float dt);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

HeaveSpring* _HeaveSpring::_ctor()
{
	AC_CTOR_THIS_POD(HeaveSpring);
	AC_CTOR_UDT(this->damper)();
	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _HeaveSpring::_init(Car* car, Suspension* s0, Suspension* s1, bool isFront)
{
	this->car = car;
	this->isFront = isFront;
	this->suspensions[0] = s0;
	this->suspensions[1] = s1;

	this->rodLength = 0;
	this->k = 0;
	this->progressiveK = 0;

	this->initData();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _HeaveSpring::_initData()
{
	auto ini(new_udt_unique<INIReader>(this->car->carDataPath + L"suspensions.ini"));
	if (!ini->ready)
	{
		SHOULD_NOT_REACH_FATAL;
		return;
	}

	std::wstring strId = this->isFront ? L"HEAVE_FRONT" : L"HEAVE_REAR";

	if (!ini->hasSection(strId))
	{
		this->isPresent = false;
		return;
	}

	this->isPresent = true;
	this->bumpStopUp = ini->getFloat(strId, L"BUMPSTOP_UP");
	this->bumpStopDn = -ini->getFloat(strId, L"BUMPSTOP_DN");
	this->rodLength = ini->getFloat(strId, L"ROD_LENGTH");
	this->k = ini->getFloat(strId, L"SPRING_RATE");
	this->progressiveK = ini->getFloat(strId, L"PROGRESSIVE_SPRING_RATE");

	this->damper.bumpSlow = ini->getFloat(strId, L"DAMP_BUMP");
	this->damper.reboundSlow = ini->getFloat(strId, L"DAMP_REBOUND");
	this->damper.bumpFast = ini->getFloat(strId, L"DAMP_FAST_BUMP");
	this->damper.reboundFast = ini->getFloat(strId, L"DAMP_FAST_REBOUND");
	this->damper.fastThresholdBump = ini->getFloat(strId, L"DAMP_FAST_BUMPTHRESHOLD");
	this->damper.fastThresholdRebound = ini->getFloat(strId, L"DAMP_FAST_REBOUNDTHRESHOLD");

	if (this->damper.fastThresholdBump == 0.0f)
		this->damper.fastThresholdBump = 0.2f;
	if (this->damper.fastThresholdRebound == 0.0f)
		this->damper.fastThresholdRebound = 0.2f;
	if (this->damper.bumpFast == 0.0f)
		this->damper.bumpFast = this->damper.bumpSlow;
	if (this->damper.reboundFast == 0.0f)
		this->damper.reboundFast = this->damper.reboundSlow;

	this->bumpStopRate = ini->getFloat(strId, L"BUMP_STOP_RATE");
	if (this->bumpStopRate == 0.0f)
		this->bumpStopRate = 500000.0f;

	this->packerRange = ini->getFloat(strId, L"PACKER_RANGE");
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _HeaveSpring::_step(float dt)
{
	mat44f mxBodyWorld = this->car->body->getWorldMatrix(0.0f);
	vec3f vM2 = vec3f(&mxBodyWorld.M21);

	auto* pSusp0 = this->suspensions[0];
	vec3f vRefPoint0 = pSusp0->dataRelToWheel.refPoint;
	vec3f vHubPos0 = pSusp0->hub->getPosition(0.0f);
	vec3f vHubLoc0 = this->car->body->worldToLocal(vHubPos0);

	auto* pSusp1 = this->suspensions[1];
	vec3f vRefPoint1 = pSusp1->dataRelToWheel.refPoint;
	vec3f vHubPos1 = pSusp1->hub->getPosition(0.0f);
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

	vec3f vDeltaVel = vHubVel - vLpv;
	float fDamperSpeed = vDeltaVel * vM2;
	float fDamperForce = this->damper.getForce(fDamperSpeed);

	vForce = vM2 * fDamperForce;
	pSusp0->addForceAtPos(vForce, pSusp0->hub->getPosition(0.0f), false, false);
	pSusp1->addForceAtPos(vForce, pSusp1->hub->getPosition(0.0f), false, false);

	vForce *= -1.0f;
	this->car->body->addLocalForceAtLocalPos(vForce, vRefPoint0);
	this->car->body->addLocalForceAtLocalPos(vForce, vRefPoint1);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
