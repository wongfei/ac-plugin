#pragma once

BEGIN_HOOK_OBJ(SuspensionML)

	#define RVA_SuspensionML_vtable 0x5001A0
	#define RVA_SuspensionML_ctor 2919968
	#define RVA_SuspensionML_loadINI 2922864
	#define RVA_SuspensionML_attach 2921504
	#define RVA_SuspensionML_setPositions 2927056
	#define RVA_SuspensionML_step 2927360
	#define RVA_SuspensionML_addForceAtPos 2920912
	#define RVA_SuspensionML_addLocalForceAndTorque 2921200
	#define RVA_SuspensionML_addTorque 2921344

	static void _hook()
	{
		HOOK_METHOD_RVA(SuspensionML, ctor);
		HOOK_METHOD_RVA(SuspensionML, loadINI);
		HOOK_METHOD_RVA(SuspensionML, attach);
		HOOK_METHOD_RVA(SuspensionML, setPositions);
		HOOK_METHOD_RVA(SuspensionML, step);
		HOOK_METHOD_RVA(SuspensionML, addForceAtPos);
		HOOK_METHOD_RVA(SuspensionML, addLocalForceAndTorque);
		HOOK_METHOD_RVA(SuspensionML, addTorque);
	}

	SuspensionML* _ctor(Car* pCar, int index);
	void _loadINI(int index);
	void _attach();
	void _setPositions();
	void _step(float dt);
	void _addForceAtPos(const vec3f& force, const vec3f& pos, bool driven, bool addToSteerTorque);
	void _addLocalForceAndTorque(const vec3f& force, const vec3f& torque, const vec3f& driveTorque);
	void _addTorque(const vec3f& torque);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

SuspensionML* _SuspensionML::_ctor(Car* pCar, int index)
{
	AC_CTOR_THIS_VT(SuspensionML);

	this->damper.ctor();

	this->car = pCar;
	this->index = index;

	this->baseCFM = 0.0000001f;
	this->damageData.minVelocity = 15.0f;
	this->damageData.damageDirection = 1.0; // TODO: random

	this->loadINI(index);

	this->baseCarSteerPosition = this->joints[4].ballCar.relToCar; // TODO: check

	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionML::_attach()
{
	this->setPositions();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionML::_setPositions()
{
	this->hub->setRotation(this->car->body->getWorldMatrix(0));
	this->hub->setPosition(this->car->body->localToWorld(this->basePosition));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionML::_loadINI(int index)
{
	auto ini(new_udt_unique<INIReader>(this->car->carDataPath + L"suspensions.ini"));
	if (!ini->ready)
	{
		SHOULD_NOT_REACH_FATAL;
		return;
	}

	auto* pCore = this->car->ksPhysics->getCore();
	auto* pCarBody = this->car->body;

	std::wstring strIndex[4] = {L"FRONT", L"FRONT", L"REAR", L"REAR"};
	const auto& strSection = strIndex[index];

	float fWheelBase = ini->getFloat(L"BASIC", L"WHEELBASE");
	float fCgLocation = ini->getFloat(L"BASIC", L"CG_LOCATION");
	float fFrontBaseY = ini->getFloat(L"FRONT", L"BASEY");
	float fFrontTrack = ini->getFloat(L"FRONT", L"TRACK") * 0.5f;
	float fRearBaseY = ini->getFloat(L"REAR", L"BASEY");
	float fRearTrack = ini->getFloat(L"REAR", L"TRACK") * 0.5f;

	vec3f vRefPoints[4];
	vRefPoints[0] = vec3f(fFrontTrack, fFrontBaseY, (1.0f - fCgLocation) * fWheelBase);
	vRefPoints[1] = vec3f(-fFrontTrack, fFrontBaseY, (1.0f - fCgLocation) * fWheelBase);
	vRefPoints[2] = vec3f(fRearTrack, fRearBaseY, -(fCgLocation * fWheelBase));
	vRefPoints[3] = vec3f(-fRearTrack, fRearBaseY, -(fCgLocation * fWheelBase));

	this->basePosition = vRefPoints[index];

	this->hubMass = ini->getFloat(strSection, L"HUB_MASS");
	this->hub = pCore->createRigidBody();
	this->hub->setMassBox(this->hubMass, 0.2f, 0.6f, 0.6f); // TODO: check

	this->setPositions();

	for (int i = 0; i < 5; ++i)
	{
		MLJoint j;
		memset(&j, 0, sizeof(j));

		auto strJoint(strf(L"JOINT%d", i));
		j.ballCar.relToTyre = ini->getFloat3(strSection, strJoint + L"_CAR");
		j.ballTyre.relToTyre = ini->getFloat3(strSection, strJoint + L"_TYRE");

		if (this->basePosition.x > 0.0f)
		{
			j.ballCar.relToTyre.x *= -1.0f;
			j.ballTyre.relToTyre.x *= -1.0f;
		}

		j.ballCar.relToCar = pCarBody->worldToLocal(this->hub->localToWorld(j.ballCar.relToTyre));
		j.ballTyre.relToCar = pCarBody->worldToLocal(this->hub->localToWorld(j.ballTyre.relToTyre));

		auto vBallCar = pCarBody->localToWorld(j.ballCar.relToCar);
		auto vBallTyre = pCarBody->localToWorld(j.ballTyre.relToCar);

		j.joint = pCore->createDistanceJoint(pCarBody, this->hub, vBallCar, vBallTyre);
		this->joints.push_back(j);
	}

	this->rodLength = ini->getFloat(strSection, L"ROD_LENGTH");
	this->toeOUT_Linear = ini->getFloat(strSection, L"TOE_OUT");
	this->k = ini->getFloat(strSection, L"SPRING_RATE");
	this->progressiveK = ini->getFloat(strSection, L"PROGRESSIVE_SPRING_RATE");

	this->damper.bumpSlow = ini->getFloat(strSection, L"DAMP_BUMP");
	this->damper.reboundSlow = ini->getFloat(strSection, L"DAMP_REBOUND");
	this->damper.bumpFast = ini->getFloat(strSection, L"DAMP_FAST_BUMP");
	this->damper.reboundFast = ini->getFloat(strSection, L"DAMP_FAST_REBOUND");
	this->damper.fastThresholdBump = ini->getFloat(strSection, L"DAMP_FAST_BUMPTHRESHOLD");
	this->damper.fastThresholdRebound = ini->getFloat(strSection, L"DAMP_FAST_REBOUNDTHRESHOLD");

	this->staticCamber = -ini->getFloat(strSection, L"STATIC_CAMBER") * 0.017453f;
	if (index % 2)
		this->staticCamber *= -1.0f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionML::_step(float dt)
{
	this->steerTorque = 0.0;

	mat44f mxBodyWorld = this->car->body->getWorldMatrix(0.0f);
	vec3f vM2(&mxBodyWorld.M21);

	vec3f vHubWorld = this->hub->getPosition(0.0f);
	vec3f vHubLocal = this->car->body->worldToLocal(vHubWorld);

	float fTravel = (vHubLocal.y - this->basePosition.y) + this->rodLength;
	this->status.travel = fTravel;

	float fForce = (fTravel * this->progressiveK + this->k) * fTravel;
	if (this->packerRange != 0.0f && fTravel > this->packerRange && this->k != 0.0f)
	{
		fForce += ((fTravel - this->packerRange) * this->bumpStopRate);
	}

	this->addForceAtPos(vM2 * -fForce, vHubWorld, false, false);
	this->car->body->addLocalForceAtLocalPos(vec3f(0.0f, fForce, 0.0f), this->basePosition);

	vec3f vPointVel = this->car->body->getLocalPointVelocity(this->basePosition);
	vec3f vDeltaVel = this->hub->getVelocity() - vPointVel;

	float fDamperSpeed = vDeltaVel * vM2;
	this->status.damperSpeedMS = fDamperSpeed;

	float fDamperForce = this->damper.getForce(fDamperSpeed);
	vec3f vForce = vM2 * fDamperForce;
	this->addForceAtPos(vForce, vHubWorld, false, false);
	this->car->body->addForceAtLocalPos(vForce * -1.0f, this->basePosition);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionML::_addForceAtPos(const vec3f& force, const vec3f& pos, bool driven, bool addToSteerTorque)
{
	this->hub->addForceAtPos(force, pos);

	if (addToSteerTorque)
	{
		vec3f vCenter, vAxis;
		this->getSteerBasis(vCenter, vAxis);

		this->steerTorque += (pos - vCenter).cross(force) * vAxis;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionML::_addLocalForceAndTorque(const vec3f& force, const vec3f& torque, const vec3f& driveTorque)
{
	this->hub->addForceAtLocalPos(force, vec3f(0, 0, 0));
	this->hub->addTorque(torque);

	if (driveTorque.x != 0.0f || driveTorque.y != 0.0f || driveTorque.z != 0.0f)
		this->car->body->addTorque(driveTorque);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionML::_addTorque(const vec3f& torque)
{
	this->hub->addTorque(torque);

	vec3f vCenter, vAxis;
	this->getSteerBasis(vCenter, vAxis);

	this->steerTorque += torque * vAxis;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
