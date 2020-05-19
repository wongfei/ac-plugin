#pragma once

BEGIN_HOOK_OBJ(SuspensionAxle)

	#define RVA_SuspensionAxle_vtable 0x4FFE90
	#define RVA_SuspensionAxle_ctor 2911120
	#define RVA_SuspensionAxle_attach 2916272
	#define RVA_SuspensionAxle_setPositions 2918112
	#define RVA_SuspensionAxle_step 2918256
	#define RVA_SuspensionAxle_addForceAtPos 2916096
	#define RVA_SuspensionAxle_addLocalForceAndTorque 2916112
	#define RVA_SuspensionAxle_addTorque 2916256

	static void _hook()
	{
		HOOK_METHOD_RVA(SuspensionAxle, ctor);
		// dont have loadINI
		HOOK_METHOD_RVA(SuspensionAxle, attach);
		HOOK_METHOD_RVA(SuspensionAxle, setPositions);
		HOOK_METHOD_RVA(SuspensionAxle, step);
		HOOK_METHOD_RVA(SuspensionAxle, addForceAtPos);
		HOOK_METHOD_RVA(SuspensionAxle, addLocalForceAndTorque);
		HOOK_METHOD_RVA(SuspensionAxle, addTorque);
	}

	SuspensionAxle* _ctor(Car* pCar, RigidAxleSide side, const std::wstring& carDataPath);
	void _loadINI(RigidAxleSide side);
	void _attach();
	void _setPositions();
	void _step(float dt);
	void _addForceAtPos(const vec3f& force, const vec3f& pos, bool driven, bool addToSteerTorque);
	void _addLocalForceAndTorque(const vec3f& force, const vec3f& torque, const vec3f& driveTorque);
	void _addTorque(const vec3f& torque);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

SuspensionAxle* _SuspensionAxle::_ctor(Car* pCar, RigidAxleSide side, const std::wstring& carDataPath)
{
	AC_CTOR_THIS_VT(SuspensionAxle);

	this->damper.ctor();

	this->car = pCar;
	this->axle = pCar->rigidAxle;
	this->side = side;

	this->baseCFM = 0.0000001f;
	this->attachRelativePos = 1.0;

	_loadINI(side);

	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionAxle::_attach()
{
	this->setPositions();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionAxle::_setPositions()
{
	this->axle->setRotation(this->car->body->getWorldMatrix(0));
	this->axle->setPosition(this->car->body->localToWorld(this->axleBasePos));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionAxle::_loadINI(RigidAxleSide side)
{
	auto ini(new_udt_unique<INIReader>(this->car->carDataPath + L"suspensions.ini"));
	if (!ini->ready)
	{
		SHOULD_NOT_REACH_FATAL;
		return;
	}

	auto* pCore = this->car->ksPhysics->getCore();
	auto* pCarBody = this->car->body;

	int iVer = ini->getInt(L"HEADER", L"VERSION");

	float fWheelBase = ini->getFloat(L"BASIC", L"WHEELBASE");
	float fCgLocation = ini->getFloat(L"BASIC", L"CG_LOCATION");
	float fRearBaseY = ini->getFloat(L"REAR", L"BASEY");
	float fRearTrack = ini->getFloat(L"REAR", L"TRACK") * 0.5f;

	this->track = fRearTrack;
	this->referenceY = fRearBaseY;
	this->axleBasePos = vec3f(0.0f, fRearBaseY, -(fCgLocation * fWheelBase));

	if (iVer >= 4)
	{
		this->attachRelativePos = ini->getFloat(L"AXLE", L"ATTACH_REL_POS");
	}

	if (side == RigidAxleSide::Left)
	{
		float fMass = ini->getFloat(L"REAR", L"HUB_MASS");
		this->axle->setMassBox(fMass, this->track * 2.0f, 0.2f, 0.5f); // TODO: check
		this->setPositions();

		int iLinkCount = ini->getInt(L"AXLE", L"LINK_COUNT");
		for (int i = 0; i < iLinkCount; ++i)
		{
			auto strId(strf(L"J%d", i));

			auto vBallCar = ini->getFloat3(L"AXLE", strId + L"_CAR");
			auto vBallAxle = ini->getFloat3(L"AXLE", strId + L"_AXLE");

			AxleJoint j;
			memset(&j, 0, sizeof(j));

			j.ballCar.relToAxle = vBallCar;
			j.ballCar.relToCar = pCarBody->worldToLocal(this->axle->localToWorld(vBallCar));
			j.ballAxle.relToAxle = vBallAxle;
			j.ballAxle.relToCar = pCarBody->worldToLocal(this->axle->localToWorld(vBallAxle));

			auto vJ0 = pCarBody->localToWorld(j.ballCar.relToCar);
			auto vJ1 = pCarBody->localToWorld(j.ballAxle.relToCar);

			j.ballAxle.joint = pCore->createDistanceJoint(pCarBody, this->axle, vJ0, vJ1);
			this->joints.push_back(j);
		}
	}

	this->bumpStopUp = ini->getFloat(L"REAR", L"BUMPSTOP_UP");
	this->bumpStopDn = -ini->getFloat(L"REAR", L"BUMPSTOP_DN");
	this->rodLength = ini->getFloat(L"REAR", L"ROD_LENGTH");
	this->toeOUT_Linear = ini->getFloat(L"REAR", L"TOE_OUT");
	this->k = ini->getFloat(L"REAR", L"SPRING_RATE");
	this->progressiveK = ini->getFloat(L"REAR", L"PROGRESSIVE_SPRING_RATE");

	this->damper.bumpSlow = ini->getFloat(L"REAR", L"DAMP_BUMP");
	this->damper.reboundSlow = ini->getFloat(L"REAR", L"DAMP_REBOUND");
	this->damper.bumpFast = ini->getFloat(L"REAR", L"DAMP_FAST_BUMP");
	this->damper.reboundFast = ini->getFloat(L"REAR", L"DAMP_FAST_REBOUND");
	this->damper.fastThresholdBump = ini->getFloat(L"REAR", L"DAMP_FAST_BUMPTHRESHOLD");
	this->damper.fastThresholdRebound = ini->getFloat(L"REAR", L"DAMP_FAST_REBOUNDTHRESHOLD");

	if (this->damper.fastThresholdBump == 0.0f)
		this->damper.fastThresholdBump = 0.2f;
	if (this->damper.fastThresholdRebound == 0.0f)
		this->damper.fastThresholdRebound = 0.2f;
	if (this->damper.bumpFast == 0.0f)
		this->damper.bumpFast = this->damper.bumpSlow;
	if (this->damper.reboundFast == 0.0f)
		this->damper.reboundFast = this->damper.reboundSlow;

	this->bumpStopRate = ini->getFloat(L"REAR", L"BUMP_STOP_RATE");
	if (this->bumpStopRate == 0.0f)
		this->bumpStopRate = 500000.0f;

	if (iVer >= 3)
	{
		this->leafSpringK.x = ini->getFloat(L"AXLE", L"LEAF_SPRING_LAT_K");
	}

	this->setERPCFM(0.3f, this->baseCFM);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionAxle::_step(float dt)
{
	mat44f mxBodyWorld = this->car->body->getWorldMatrix(0.0f);

	mat44f mxAxle = this->axle->getWorldMatrix(0.0f);
	vec3f vAxleM1(&mxAxle.M11);
	vec3f vAxleM4(&mxAxle.M41);

	float fSideSign = ((int)this->side ? -1.0f : 1.0f);
	vec3f vAxleWorld = (vAxleM1 * (fSideSign * this->track * this->attachRelativePos)) + vAxleM4;
	vec3f vAxleLocal = this->car->body->worldToLocal(vAxleWorld);

	vec3f vBase = this->getBasePosition();
	vBase.x *= this->attachRelativePos;
	vBase.y += 0.2f;
	vec3f vBaseWorld = this->car->body->localToWorld(vBase);

	vec3f vDelta = vBaseWorld - vAxleWorld;
	float fDeltaLen = vDelta.len();
	vDelta.norm(fDeltaLen);

	float fTravel = (0.2f - fDeltaLen) + this->rodLength;
	this->status.travel = fTravel;

	float fForce = -(((fTravel * this->progressiveK) + this->k) * fTravel);
	if (fForce < 0.0f)
	{
		this->addForceAtPos(vDelta * fForce, vAxleWorld, false, false);
		this->car->body->addForceAtPos(vDelta * -fForce, vBaseWorld);
	}

	if (this->leafSpringK.x != 0.0f)
	{
		fForce = (vAxleLocal.x - (this->attachRelativePos * this->getBasePosition().x)) * this->leafSpringK.x;
		this->addForceAtPos(vec3f(&mxBodyWorld.M11) * -fForce, vAxleWorld, false, false);
		this->car->body->addLocalForceAtLocalPos(vec3f(fForce, 0.0f, 0.0f), vAxleLocal);
	}

	float fBumpStopUp = this->bumpStopUp;
	float fRefY = vAxleLocal.y - this->referenceY;
	if (fBumpStopUp != 0.0f && fRefY > fBumpStopUp && 0.0f != this->k)
	{
		fForce = (fRefY - fBumpStopUp) * 500000.0f;
		this->addForceAtPos(vec3f(&mxBodyWorld.M21) * -fForce, vAxleWorld, false, false);
		this->car->body->addLocalForceAtLocalPos(vec3f(0.0f, fForce, 0.0f), vAxleLocal);
	}

	float fBumpStopDn = this->bumpStopDn;
	if (fBumpStopDn != 0.0f && fRefY < fBumpStopDn && 0.0f != this->k)
	{
		fForce = (fRefY - fBumpStopDn) * 500000.0f;
		this->addForceAtPos(vec3f(&mxBodyWorld.M21) * -fForce, vAxleWorld, false, false);
		this->car->body->addLocalForceAtLocalPos(vec3f(0.0f, fForce, 0.0f), vAxleLocal);
	}

	vec3f vPointVel = this->car->body->getPointVelocity(vBaseWorld);
	vec3f vDeltaVel = this->getVelocity() - vPointVel;

	float fDamperSpeed = vDeltaVel * vDelta;
	this->status.damperSpeedMS = fDamperSpeed;

	float fDamperForce = this->damper.getForce(fDamperSpeed);
	vec3f vForce = vDelta * fDamperForce;
	this->addForceAtPos(vForce, vAxleWorld, false, false);
	this->car->body->addForceAtPos(vForce * -1.0f, vBaseWorld);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionAxle::_addForceAtPos(const vec3f& force, const vec3f& pos, bool driven, bool addToSteerTorque)
{
	this->axle->addForceAtPos(force, pos);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionAxle::_addLocalForceAndTorque(const vec3f& force, const vec3f& torque, const vec3f& driveTorque)
{
	this->axle->addForceAtLocalPos(force, vec3f(0, 0, 0));
	this->axle->addTorque(torque);

	if (driveTorque.x != 0.0f || driveTorque.y != 0.0f || driveTorque.z != 0.0f)
		this->car->body->addTorque(driveTorque);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionAxle::_addTorque(const vec3f& torque)
{
	this->axle->addTorque(torque);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
