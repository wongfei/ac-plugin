#pragma once

// DoubleWishbone

BEGIN_HOOK_OBJ(Suspension)

	#define RVA_Suspension_vtable 0x4FF870
	#define RVA_Suspension_ctor 2885408
	#define RVA_Suspension_loadINI 2891152
	#define RVA_Suspension_attach 2887600
	#define RVA_Suspension_step 2896784
	#define RVA_Suspension_addForceAtPos 2886640
	#define RVA_Suspension_addLocalForceAndTorque 2886976
	#define RVA_Suspension_addTorque 2887440

	static void _hook()
	{
		HOOK_METHOD_RVA(Suspension, ctor);
		HOOK_METHOD_RVA(Suspension, loadINI);
		HOOK_METHOD_RVA(Suspension, attach);
		HOOK_METHOD_RVA(Suspension, step);
		HOOK_METHOD_RVA(Suspension, addForceAtPos);
		HOOK_METHOD_RVA(Suspension, addLocalForceAndTorque);
		HOOK_METHOD_RVA(Suspension, addTorque);
	}

	Suspension* _ctor(Car* pCar, int index);
	void _loadINI(int index);
	void _attach();
	void _step(float dt);
	void _addForceAtPos(const vec3f& force, const vec3f& pos, bool driven, bool addToSteerTorque);
	void _addLocalForceAndTorque(const vec3f& force, const vec3f& torque, const vec3f& driveTorque);
	void _addTorque(const vec3f& torque);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

Suspension* _Suspension::_ctor(Car* pCar, int index)
{
	AC_CTOR_VCLASS(Suspension);

	this->damper.ctor();
	this->activeActuator.ctor();

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
		this->dataRelToWheel.carTopWB_F.x *= -1.0f;
		this->dataRelToWheel.carTopWB_R.x *= -1.0f;
		this->dataRelToWheel.tyreBottomWB.x *= -1.0f;
		this->dataRelToWheel.tyreSteer.x *= -1.0f;
		this->dataRelToWheel.tyreTopWB.x *= -1.0f;
	}

	float fMass = this->dataRelToWheel.hubMass;
	if (fMass <= 0.0f)
		fMass = 20.0f;

	vec3f vInertia = this->dataRelToWheel.hubInertiaBox;
	if (vInertia.len() == 0.0f)
		vInertia = vec3f(0.2f, 0.6f, 0.6f); // TODO: check

	this->hub = this->physicsEngine->getCore()->createRigidBody();
	this->hub->setMassBox(fMass, vInertia.x, vInertia.y, vInertia.z);
	this->basePosition = this->dataRelToWheel.refPoint;

	this->attach();

	this->baseCarSteerPosition = this->dataRelToBody.carSteer;
	this->setSteerLengthOffset(0);

	for (auto* pJoint : this->joints)
	{
		pJoint->setERPCFM(0.3f, this->baseCFM);
	}

	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Suspension::_attach()
{
	this->hub->setRotation(this->carBody->getWorldMatrix(0));
	this->hub->setPosition(this->carBody->localToWorld(this->basePosition));

	if (this->joints[0])
		return;

	auto* pCore = this->physicsEngine->getCore();

	#define CALC_REL_TO_BODY(name)\
		this->dataRelToBody.name = this->carBody->localToWorld(\
			this->dataRelToWheel.name + this->dataRelToWheel.refPoint)

	#define CREATE_JOINT(id, a, b)\
		this->joints[id] = pCore->createDistanceJoint(this->carBody, this->hub, \
			this->dataRelToBody.a, this->dataRelToBody.b)

	CALC_REL_TO_BODY(carBottomWB_F);
	CALC_REL_TO_BODY(carBottomWB_R);
	CALC_REL_TO_BODY(carTopWB_F);
	CALC_REL_TO_BODY(carTopWB_R);
	CALC_REL_TO_BODY(tyreBottomWB);
	CALC_REL_TO_BODY(tyreTopWB);
	CALC_REL_TO_BODY(carSteer);
	CALC_REL_TO_BODY(tyreSteer);

	CREATE_JOINT(0, carTopWB_R, tyreTopWB);
	CREATE_JOINT(1, carTopWB_F, tyreTopWB);
	CREATE_JOINT(2, carBottomWB_R, tyreBottomWB);
	CREATE_JOINT(3, carBottomWB_F, tyreBottomWB);
	CREATE_JOINT(4, carSteer, tyreSteer);

	this->steerLinkBaseLength = (this->dataRelToBody.carSteer - this->dataRelToBody.tyreSteer).len();

	this->bumpStopJoint = pCore->createBumpJoint(this->carBody, this->hub, 
		this->dataRelToWheel.refPoint, this->bumpStopUp, this->bumpStopDn);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Suspension::_loadINI(int index)
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

	this->dataRelToWheel.carTopWB_F = ini->getFloat3(strSection, L"WBCAR_TOP_FRONT");
	this->dataRelToWheel.carTopWB_R = ini->getFloat3(strSection, L"WBCAR_TOP_REAR");
	this->dataRelToWheel.carBottomWB_F = ini->getFloat3(strSection, L"WBCAR_BOTTOM_FRONT");
	this->dataRelToWheel.carBottomWB_R = ini->getFloat3(strSection, L"WBCAR_BOTTOM_REAR");
	this->dataRelToWheel.tyreTopWB = ini->getFloat3(strSection, L"WBTYRE_TOP");
	this->dataRelToWheel.tyreBottomWB = ini->getFloat3(strSection, L"WBTYRE_BOTTOM");
	this->dataRelToWheel.tyreSteer = ini->getFloat3(strSection, L"WBTYRE_STEER");
	this->dataRelToWheel.carSteer = ini->getFloat3(strSection, L"WBCAR_STEER");

	if (iVer >= 2)
	{
		float fRimOffset = -ini->getFloat(strSection, L"RIM_OFFSET");
		if (fRimOffset != 0.0f)
		{
			this->dataRelToWheel.carTopWB_F.x += fRimOffset;
			this->dataRelToWheel.carTopWB_R.x += fRimOffset;
			this->dataRelToWheel.carBottomWB_F.x += fRimOffset;
			this->dataRelToWheel.carBottomWB_R.x += fRimOffset;
			this->dataRelToWheel.tyreTopWB.x += fRimOffset;
			this->dataRelToWheel.tyreBottomWB.x += fRimOffset;
			this->dataRelToWheel.tyreSteer.x += fRimOffset;
			this->dataRelToWheel.carSteer.x += fRimOffset;
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

	if (ini->hasKey(strSection, L"BUMP_STOP_PROGRESSIVE"))
	{
		this->bumpStopProgressive = ini->getFloat(strSection, L"BUMP_STOP_PROGRESSIVE");
	}

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

	if (ini->hasSection(L"RIGIDITY")) // TODO: check
	{
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Suspension::_step(float dt)
{
	this->steerTorque = 0.0f;

	mat44f mxBodyWorld = this->carBody->getWorldMatrix(0.0f);
	vec3f vBodyM2(&mxBodyWorld.M21);

	vec3f vHubWorldPos = this->hub->getPosition(0.0f);
	vec3f vHubLocalPos = this->carBody->worldToLocal(vHubWorldPos);

	float fHubDeltaY = vHubLocalPos.y - this->dataRelToWheel.refPoint.y;
	float fTravel = fHubDeltaY + this->rodLength;
	this->status.travel = fTravel;

	if (this->useActiveActuator)
	{
		this->activeActuator.targetTravel = 0.0f;
		float fActiveActuator = this->activeActuator.eval(dt, fHubDeltaY);

		this->car->antirollBars[0].k = 0.0f;
		this->car->antirollBars[1].k = 0.0f;

		vec3f vForce = vBodyM2 * fActiveActuator;
		this->addForceAtPos(vForce, vHubWorldPos, false, false);
		this->carBody->addForceAtLocalPos(vForce * -1.0f, this->dataRelToWheel.refPoint);

		this->status.travel = fHubDeltaY;
	}
	else
	{
		float fForce = ((fTravel * this->progressiveK) + this->k) * fTravel;
		if (this->packerRange != 0.0f && fTravel > this->packerRange && this->k != 0.0f)
		{
			fForce += (((fTravel - this->packerRange) * this->bumpStopProgressive) + this->bumpStopRate) * (fTravel - this->packerRange);
		}

		if (fForce > 0.0f)
		{
			this->addForceAtPos(vBodyM2 * -fForce, vHubWorldPos, false, false);
			this->carBody->addLocalForceAtLocalPos(vec3f(0, fForce, 0), this->dataRelToWheel.refPoint);
		}

		vec3f vHubVel = this->hub->getVelocity();
		vec3f vPointVel = this->carBody->getLocalPointVelocity(this->dataRelToWheel.refPoint);
		vec3f vDeltaVel = vHubVel - vPointVel;

		float fDamperSpeed = vDeltaVel * vBodyM2;
		this->status.damperSpeedMS = fDamperSpeed;

		float fDamperForce = this->damper.getForce(fDamperSpeed);
		vec3f vForce = vBodyM2 * fDamperForce;
		this->addForceAtPos(vForce, vHubWorldPos, false, false);
		this->carBody->addForceAtLocalPos(vForce * -1.0f, this->dataRelToWheel.refPoint);
	}

	if (this->bumpStopUp != 0.0f && fHubDeltaY > this->bumpStopUp && 0.0f != this->k)
	{
		float fForce = (((fHubDeltaY - this->bumpStopUp) * this->bumpStopProgressive) + this->bumpStopRate) * (fHubDeltaY - this->bumpStopUp);
		this->addForceAtPos(vBodyM2 * -fForce, vHubWorldPos, false, false);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fForce, 0), vHubLocalPos);
	}

	if (this->bumpStopDn != 0.0f && fHubDeltaY < this->bumpStopDn && 0.0f != this->k)
	{
		float fForce = (((fHubDeltaY - this->bumpStopDn) * this->bumpStopProgressive) + this->bumpStopRate) * (fHubDeltaY - this->bumpStopDn);
		this->addForceAtPos(vBodyM2 * -fForce, vHubWorldPos, false, false);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, fForce, 0), vHubLocalPos);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Suspension::_addForceAtPos(const vec3f& force, const vec3f& pos, bool driven, bool addToSteerTorque)
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

void _Suspension::_addLocalForceAndTorque(const vec3f& force, const vec3f& torque, const vec3f& driveTorque)
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

void _Suspension::_addTorque(const vec3f& torque)
{
	this->hub->addTorque(torque);

	vec3f vCenter, vAxis;
	this->getSteerBasis(vCenter, vAxis);

	this->steerTorque += torque * vAxis;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
