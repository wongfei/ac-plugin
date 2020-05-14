#pragma once

BEGIN_HOOK_OBJ(SuspensionStrut)

	#define RVA_SuspensionStrut_vtable 0x4FFC80
	#define RVA_SuspensionStrut_ctor 2898128
	#define RVA_SuspensionStrut_loadINI 2903776
	#define RVA_SuspensionStrut_attach 2900224
	#define RVA_SuspensionStrut_setPositions 2908688
	#define RVA_SuspensionStrut_step 2909696
	#define RVA_SuspensionStrut_addForceAtPos 2899296
	#define RVA_SuspensionStrut_addLocalForceAndTorque 2899584
	#define RVA_SuspensionStrut_addTorque 2900064

	static void _hook()
	{
		HOOK_METHOD_RVA(SuspensionStrut, ctor);
		HOOK_METHOD_RVA(SuspensionStrut, loadINI);
		HOOK_METHOD_RVA(SuspensionStrut, attach);
		HOOK_METHOD_RVA(SuspensionStrut, setPositions);
		HOOK_METHOD_RVA(SuspensionStrut, step);
		HOOK_METHOD_RVA(SuspensionStrut, addForceAtPos);
		HOOK_METHOD_RVA(SuspensionStrut, addLocalForceAndTorque);
		HOOK_METHOD_RVA(SuspensionStrut, addTorque);
	}

	SuspensionStrut* _ctor(Car* pCar, int index);
	void _loadINI(int index);
	void _attach();
	void _setPositions();
	void _step(float dt);
	void _addForceAtPos(const vec3f& force, const vec3f& pos, bool driven, bool addToSteerTorque);
	void _addLocalForceAndTorque(const vec3f& force, const vec3f& torque, const vec3f& driveTorque);
	void _addTorque(const vec3f& torque);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

SuspensionStrut* _SuspensionStrut::_ctor(Car* pCar, int index)
{
	AC_CTOR_VCLASS(SuspensionStrut);

	this->damper.ctor();

	this->physicsEngine = pCar->ksPhysics;
	this->car = pCar;
	this->carBody = pCar->body;
	this->index = index;

	this->baseCFM = 0.0000001f;
	this->damageData.damageDirection = 1.0; // TODO: random

	this->loadINI(index);

	if (this->dataRelToWheel.refPoint.x > 0.0f)
	{
		this->dataRelToWheel.carBottomWB_F.x *= -1.0f;
		this->dataRelToWheel.carBottomWB_R.x *= -1.0f;
		this->dataRelToWheel.carSteer.x *= -1.0f;
		this->dataRelToWheel.carStrut.x *= -1.0f;
		this->dataRelToWheel.tyreBottomWB.x *= -1.0f;
		this->dataRelToWheel.tyreSteer.x *= -1.0f;
		this->dataRelToWheel.tyreStrut.x *= -1.0f;
	}

	float fMass = this->dataRelToWheel.hubMass;
	if (fMass <= 0.0f)
		fMass = 20.0f;

	vec3f vInertia = this->dataRelToWheel.hubInertiaBox;
	if (vInertia.len() == 0.0f)
		vInertia = vec3f(0.2f, 0.6f, 0.6f); // TODO: check

	this->hub = this->physicsEngine->getCore()->createRigidBody();
	this->hub->setMassBox(fMass * 0.8f, vInertia.x, vInertia.y, vInertia.z);

	this->strutBody = this->physicsEngine->getCore()->createRigidBody();
	this->strutBody->setMassBox(fMass * 0.2f, 0.05f, 0.5f, 0.2f); // TODO: check
	this->strutBodyLength = 0.2f;

	this->basePosition = this->dataRelToWheel.refPoint;

	this->attach();

	this->baseCarSteerPosition = this->dataRelToBody.carSteer;
	this->setSteerLengthOffset(0);

	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionStrut::_attach()
{
	this->setPositions();

	if (this->joints[0])
		return;

	auto* pCore = this->physicsEngine->getCore();

	#define CALC_REL_TO_BODY(name)\
		this->dataRelToBody.name = this->carBody->localToWorld(\
			this->dataRelToWheel.name + this->dataRelToWheel.refPoint)

	#define CREATE_DIST_JOINT(id, a, b)\
		this->joints[id] = pCore->createDistanceJoint(this->carBody, this->hub, \
			this->dataRelToBody.a, this->dataRelToBody.b)

	CALC_REL_TO_BODY(carBottomWB_F);
	CALC_REL_TO_BODY(carBottomWB_R);
	CALC_REL_TO_BODY(carStrut);
	CALC_REL_TO_BODY(tyreBottomWB);
	CALC_REL_TO_BODY(tyreStrut);
	CALC_REL_TO_BODY(carSteer);
	CALC_REL_TO_BODY(tyreSteer);

	this->setPositions();

	auto vCarStrut = this->carBody->localToWorld(this->dataRelToBody.carStrut);
    auto vTyreStrut = this->hub->localToWorld(this->dataRelToWheel.tyreStrut);

	CREATE_DIST_JOINT(0, carBottomWB_R, tyreBottomWB);
	CREATE_DIST_JOINT(1, carBottomWB_F, tyreBottomWB);
	CREATE_DIST_JOINT(2, carSteer, tyreSteer);

	this->joints[3] = pCore->createSliderJoint(this->strutBody, this->hub, (vTyreStrut - vCarStrut));
	this->joints[4] = pCore->createBallJoint(this->carBody, this->strutBody, vCarStrut);

	this->steerLinkBaseLength = (this->dataRelToBody.carSteer - this->dataRelToBody.tyreSteer).len();
	this->strutBaseLength = (vTyreStrut - vCarStrut).len();

	this->bumpStopJoint = pCore->createBumpJoint(this->carBody, this->hub, 
		this->dataRelToWheel.refPoint, this->bumpStopUp, this->bumpStopDn);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionStrut::_setPositions()
{
	mat44f mxBody(this->car->body->getWorldMatrix(0));

	this->hub->setRotation(mxBody);
	this->hub->setPosition(this->car->body->localToWorld(this->basePosition));

	auto vCarStrut = this->carBody->localToWorld(this->dataRelToBody.carStrut);
	auto vTyreStrut = this->hub->localToWorld(this->dataRelToWheel.tyreStrut);
	auto vNorm = (vTyreStrut - vCarStrut).norm();

	// TODO: WHY THE FUCK THIS SHIT WORKS???
	auto vM3 = vec3f(&mxBody.M31) * -1.0f;
	auto vM3N = vM3.cross(vNorm);
	auto vM3NN = vM3N.cross(vNorm).norm();

	mat44f mxStrut;
	mxStrut.M11 = vM3NN.x;
	mxStrut.M12 = vM3NN.y;
	mxStrut.M13 = vM3NN.z;
	mxStrut.M14 = 0;
	mxStrut.M21 = -vM3N.x;
	mxStrut.M22 = -vM3N.y;
	mxStrut.M23 = -vM3N.z;
	mxStrut.M24 = 0;
	mxStrut.M31 = -vNorm.x;
	mxStrut.M32 = -vNorm.y;
	mxStrut.M33 = -vNorm.z;
	mxStrut.M34 = 0;

	this->strutBody->setRotation(mxStrut);
	this->strutBody->setPosition((vNorm * this->strutBodyLength) * 0.5f + vCarStrut);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionStrut::_loadINI(int index)
{
	auto ini(new_udt_unique<INIReader>(this->car->carDataPath + L"suspensions.ini"));
	if (!ini->ready)
	{
		SHOULD_NOT_REACH_FATAL;
		return;
	}

	int iVer = ini->getInt(L"HEADER", L"VERSION");

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

	this->dataRelToWheel.refPoint = vRefPoints[index];

	this->dataRelToWheel.carStrut = ini->getFloat3(strSection, L"STRUT_CAR");
	this->dataRelToWheel.tyreStrut = ini->getFloat3(strSection, L"STRUT_TYRE");
	this->dataRelToWheel.carBottomWB_F = ini->getFloat3(strSection, L"WBCAR_BOTTOM_FRONT");
	this->dataRelToWheel.carBottomWB_R = ini->getFloat3(strSection, L"WBCAR_BOTTOM_REAR");
	this->dataRelToWheel.tyreBottomWB = ini->getFloat3(strSection, L"WBTYRE_BOTTOM");
	this->dataRelToWheel.tyreSteer = ini->getFloat3(strSection, L"WBTYRE_STEER");
	this->dataRelToWheel.carSteer = ini->getFloat3(strSection, L"WBCAR_STEER");

	if (iVer >= 2)
	{
		float fRimOffset = -ini->getFloat(strSection, L"RIM_OFFSET");
		if (fRimOffset != 0.0f)
		{
			this->dataRelToWheel.carStrut.x  += fRimOffset;
			this->dataRelToWheel.tyreStrut.x  += fRimOffset;
			this->dataRelToWheel.carBottomWB_F.x  += fRimOffset;
			this->dataRelToWheel.carBottomWB_R.x  += fRimOffset;
			this->dataRelToWheel.tyreBottomWB.x  += fRimOffset;
			this->dataRelToWheel.tyreSteer.x  += fRimOffset;
			this->dataRelToWheel.carSteer.x  += fRimOffset;
	  	}
	}

	this->dataRelToWheel.hubMass = ini->getFloat(strSection, L"HUB_MASS");

	this->bumpStopUp = ini->getFloat(strSection, L"BUMPSTOP_UP");
	this->bumpStopDn = -ini->getFloat(strSection, L"BUMPSTOP_DN");
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

	if (this->damper.fastThresholdBump == 0.0f)
		this->damper.fastThresholdBump = 0.2f;
	if (this->damper.fastThresholdRebound == 0.0f)
		this->damper.fastThresholdRebound = 0.2f;
	if (this->damper.bumpFast == 0.0f)
		this->damper.bumpFast = this->damper.bumpSlow;
	if (this->damper.reboundFast == 0.0f)
		this->damper.reboundFast = this->damper.reboundSlow;

	this->bumpStopRate = ini->getFloat(strSection, L"BUMP_STOP_RATE");
	if (this->bumpStopRate == 0.0f)
		this->bumpStopRate = 500000.0f;

	this->staticCamber = -ini->getFloat(strSection, L"STATIC_CAMBER") * 0.017453f;
	if (index % 2)
		this->staticCamber *= -1.0f;

	this->packerRange = ini->getFloat(strSection, L"PACKER_RANGE");

	if (ini->hasSection(L"DAMAGE"))
	{
		this->damageData.minVelocity = ini->getFloat(L"DAMAGE", L"MIN_VELOCITY");
		this->damageData.damageGain = ini->getFloat(L"DAMAGE", L"GAIN");
		this->damageData.maxDamage = ini->getFloat(L"DAMAGE", L"MAX_DAMAGE");
		this->damageData.isDebug = ini->getInt(L"DAMAGE", L"DEBUG_LOG") != 0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionStrut::_step(float dt)
{
	this->steerTorque = 0.0f;

	mat44f mxBodyWorld = this->carBody->getWorldMatrix(0.0f);

	vec3f vCarStrut = this->carBody->localToWorld(this->dataRelToBody.carStrut);
	vec3f vTyreStrut = this->hub->localToWorld(this->dataRelToWheel.tyreStrut);

	vec3f vDelta = vTyreStrut - vCarStrut;
	float fDeltaLen = vDelta.len();
	vDelta.norm(fDeltaLen);

	float fDefaultLength = this->strutBaseLength + this->rodLength;
	float fTravel = fDefaultLength - fDeltaLen;
	this->status.travel = fTravel;

	float fForce = ((fTravel * this->progressiveK) + this->k) * fTravel;
	if (fForce < 0) 
		fForce = 0;

	if (this->packerRange != 0.0f && fTravel > this->packerRange)
		fForce += ((fTravel - this->packerRange) * this->bumpStopRate);

	if (fForce > 0)
	{
		vec3f vForce = vDelta * fForce;
		this->addForceAtPos(vForce, vTyreStrut, false, false);
		this->carBody->addForceAtPos(vForce * -1.0f, vCarStrut);
	}

	vec3f vHubWorld = this->hub->getPosition(0.0f);
	vec3f vHubLocal = this->carBody->worldToLocal(vHubWorld);
	float fHubDelta = vHubLocal.y - this->dataRelToWheel.refPoint.y;

	if (fHubDelta > this->bumpStopUp)
	{
		fForce = (fHubDelta - this->bumpStopUp) * 500000.0f;
		this->addForceAtPos(vec3f(&mxBodyWorld.M21) * -fForce, this->hub->getPosition(0.0f), false, false);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fForce, 0), vHubLocal);
	}

	if (fHubDelta < this->bumpStopDn)
	{
		fForce = (fHubDelta - this->bumpStopDn) * 500000.0f;
		this->addForceAtPos(vec3f(&mxBodyWorld.M21) * -fForce, this->hub->getPosition(0.0f), false, false);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fForce, 0), vHubLocal);
	}

	vec3f vTyreStrutVel = this->hub->getLocalPointVelocity(this->dataRelToWheel.tyreStrut);
	vec3f vCarStrutVel = this->carBody->getLocalPointVelocity(this->dataRelToBody.carStrut);
	vec3f vDamperDelta = vTyreStrutVel - vCarStrutVel;

	float fDamperSpeed = vDamperDelta * vDelta;
	this->status.damperSpeedMS = fDamperSpeed;

	float fDamperForce = this->damper.getForce(fDamperSpeed);
	vec3f vDamperForce = vDelta * fDamperForce;
	this->addForceAtPos(vDamperForce, vTyreStrut, false, false);
	this->carBody->addForceAtPos(vDamperForce * -1.0f, vCarStrut);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionStrut::_addForceAtPos(const vec3f& force, const vec3f& pos, bool driven, bool addToSteerTorque)
{
	this->hub->addForceAtPos(force, pos);

	if (addToSteerTorque)
	{
		vec3f vCenter, vAxis;
		this->getSteerBasis(vCenter, vAxis);

		this->steerTorque -= force.cross(pos - vCenter) * vAxis;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionStrut::_addLocalForceAndTorque(const vec3f& force, const vec3f& torque, const vec3f& driveTorque)
{
	this->hub->addForceAtLocalPos(force, vec3f(0, 0, 0));
	this->hub->addTorque(torque);

	vec3f vCenter, vAxis;
	this->getSteerBasis(vCenter, vAxis);

	vec3f vHubPos = this->hub->getPosition(0);

	this->steerTorque += (vHubPos - vCenter).cross(force) * vAxis;
	this->steerTorque += torque * vAxis;

	if (driveTorque.x != 0.0f || driveTorque.y != 0.0f || driveTorque.z != 0.0f)
		this->car->body->addTorque(driveTorque);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _SuspensionStrut::_addTorque(const vec3f& torque)
{
	this->hub->addTorque(torque);

	vec3f vCenter, vAxis;
	this->getSteerBasis(vCenter, vAxis);

	this->steerTorque += torque * vAxis;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
