#pragma once

BEGIN_HOOK_OBJ(Car)

	#define RVA_Car_vtable 0x4F6EB0
	#define RVA_Car_ctor 2539264
	#define RVA_Car_initCarData 2566960
	#define RVA_Car_step 2579872
	#define RVA_Car_updateAirPressure 2583264
	#define RVA_Car_updateBodyMass 2583664
	#define RVA_Car_calcBodyMass 2554736
	#define RVA_Car_stepThermalObjects 2583024
	#define RVA_Car_stepComponents 2581712
	#define RVA_Car_onCollisionCallBack 2573904

	static void _hook()
	{
		HOOK_METHOD_RVA(Car, ctor);
		HOOK_METHOD_RVA(Car, initCarData);
		HOOK_METHOD_RVA_ORIG(Car, step);
		HOOK_METHOD_RVA(Car, updateAirPressure);
		HOOK_METHOD_RVA(Car, updateBodyMass);
		HOOK_METHOD_RVA(Car, calcBodyMass);
		HOOK_METHOD_RVA(Car, stepThermalObjects);
		HOOK_METHOD_RVA(Car, stepComponents);
		HOOK_METHOD_RVA(Car, onCollisionCallBack);
	}

	Car* _ctor(PhysicsEngine* iengine, const std::wstring& iunixName, const std::wstring& config);
	void _initCarData();
	void _step(float dt);
	void _updateAirPressure();
	void _updateBodyMass();
	float _calcBodyMass();
	void _stepThermalObjects(float dt);
	void _stepComponents(float dt);
	void _onCollisionCallBack(void* userData0, void* shape0, void* userData1, void* shape1, const vec3f& normal, const vec3f& pos, float depth);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

Car* _Car::_ctor(PhysicsEngine* iengine, const std::wstring& iunixName, const std::wstring& config)
{
	// TODO: experimental

	AC_CTOR_THIS_VT(Car);

	auto pThis = this;

	this->carHalfWidth = 1.0;
	this->userFFGain = 1.0;
	this->ksPhysics = iengine;
	this->controls.requestedGearIndex = -1;
	AC_CTOR_UDT(this->colliderManager)();
	AC_CTOR_UDT(this->drivetrain)();
	AC_CTOR_UDT(this->abs.valueCurve)();
	this->abs.channels = 4;
	this->abs.currentValue = 1.0;
	AC_CTOR_UDT(this->tractionControl.valueCurve)();
	for (auto& iter : this->tyres) { AC_CTOR_UDT(iter)(); }
	AC_CTOR_UDT(this->brakeSystem)();
	AC_CTOR_UDT(this->autoClutch)();
	AC_CTOR_UDT(this->telemetry)();
	AC_CTOR_UDT(this->autoBlip.blipProfile)();
	this->edl.rightTyreIndex = 1;
	for (auto& iter : this->antirollBars) { AC_CTOR_UDT(iter)(); }
	this->setupManager.minimumHeight_m = -1.0;
	this->setupManager.maxWaitTime = 1.0;
	this->setupManager.waitTime = 1.0;
	this->setupManager.setupState = CarSetupState::Legal;
	this->drs.isAvailable = true;
	this->kers._vtable = _drva(0x4F6E98);
	AC_CTOR_UDT(this->kers.torqueLUT)();
	AC_CTOR_UDT(this->kers.controller)();
	AC_CTOR_UDT(this->ers)();
	this->lapInvalidator.collisionSafeTime = 5000.0;
	this->lapInvalidator.currentTyresOut = -1;
	this->fuelLapEvaluator.startFuel = -100.0;
	for (auto& iter : this->heaveSprings) { AC_CTOR_UDT(iter)(); }
	AC_CTOR_UDT(this->steeringSystem.ctrl4ws)();
	this->axleTorqueReaction = 1.0;
	AC_CTOR_UDT(this->water)();
	this->aiLapsToComplete = -1;
	this->bounds.length = 4.0;
	this->bounds.width = 2.0;
	this->bounds.lengthFront = 2.0;
	this->bounds.lengthRear = 2.0;
	this->slipStream.triangle._vtable = _drva(0x4AD788);
	this->slipStream.speedFactorMult = 1.0;
	this->slipStream.effectGainMult = 1.0;
	this->slipStream.speedFactor = 0.25;
	this->slipStreamEffectGain = 1.0;
	this->framesToSleep = 50;
	this->pitTimings.tyreChangeTimeSec = 10.0;
	this->pitTimings.fuelChangeTimeSec = 0.1;
	this->pitTimings.bodyRepairTimeSec = 2.0;
	this->pitTimings.engineRepairTimeSec = 2.0;
	this->pitTimings.suspRepairTimeSec = 2.0;
	AC_CTOR_UDT(this->valueCache.speed)();
	this->fuelKG = 0.74;
	this->lastBodyMassUpdateTime = -100000000.0;
	this->physicsGUID = (unsigned int)this->ksPhysics->cars.size();
	this->steerAssist = 1.0;
	this->fuel = 30.0;
	this->maxFuel = 30.0;
	this->requestedFuel = 30.0;
	this->ffMult = 0.003;
	this->steerLock = 200.0;
	this->steerRatio = 12.0;

	auto* pCore = this->ksPhysics->getCore();
	this->body = pCore->createRigidBody();
	this->fuelTankBody = pCore->createRigidBody();

	this->unixName = iunixName;
	this->carDataPath = L"content/cars/" + iunixName + L"/data/"; // this->initCarDataPath();

	this->initCarData();
	this->brakeSystem.init(this);

	auto ini(new_udt_unique<INIReader>(this->carDataPath + L"suspensions.ini"));

	auto strRearType = ini->getString(L"REAR", L"TYPE");
	if (strRearType == L"AXLE")
	{
		this->rigidAxle = pCore->createRigidBody();
		this->axleTorqueReaction = ini->getFloat(L"AXLE", L"TORQUE_REACTION");
	}

	for (int index = 0; index < 4; ++index)
	{
		std::wstring strSuspType;
		SuspensionType eSuspType;
		ISuspension* pSusp = nullptr;

		if (index >= 2)
			strSuspType = ini->getString(L"REAR", L"TYPE");
		else
			strSuspType = ini->getString(L"FRONT", L"TYPE");

		if (strSuspType == L"STRUT")
		{
			pSusp = new_udt<SuspensionStrut>(this, index);
			eSuspType = SuspensionType::Strut;
		}
		else if (strSuspType == L"DWB")
		{
			pSusp = new_udt<Suspension>(this, index);
			eSuspType = SuspensionType::DoubleWishbone;
		}
		else if (strSuspType == L"ML")
		{
			pSusp = new_udt<SuspensionML>(this, index);
			eSuspType = SuspensionType::Multilink;
		}
		else if (strSuspType == L"AXLE" && index >= 2)
		{
			auto eSide = (index == 2) ? RigidAxleSide::Left : RigidAxleSide::Right;
			pSusp = new_udt<SuspensionAxle>(this, eSide, this->carDataPath);
			eSuspType = SuspensionType::Axle;
		}
		else
		{
			SHOULD_NOT_REACH_FATAL;
			return this;
		}

		if (index >= 2)
			this->suspensionTypeR = eSuspType;
		else
			this->suspensionTypeF = eSuspType;

		this->suspensions.push_back(pSusp);

		auto* pTyre = &this->tyres[index];
		pTyre->ctor();

		auto strDataPath(this->carDataPath);
		pTyre->init(pSusp, this->ksPhysics->track, &strDataPath, index, this->physicsGUID, this);
	}

	this->tyres[3].onStepCompleted = [pThis]() { pThis->onTyresStepCompleted(); };

	for (auto& tcd : this->tyres[0].compoundDefs)
	{
		this->tyreCompounds.push_back(tcd.name);
	}

	this->initHeaveSprings();
	this->ridePickupPoint[0].z = this->suspensions[0]->getBasePosition().z;
	this->ridePickupPoint[1].z = this->suspensions[2]->getBasePosition().z;
	this->initAeroMap();
	this->ers.init(this);
	if (!this->ers.present)
		this->kers.init(this);
	this->steeringSystem.init(this);
	this->steeringSystem.linearRatio = this->steerLinearRatio;
	this->drivetrain.init(this);
	this->controls.isShifterSupported = this->drivetrain.isShifterSupported;
	this->autoClutch.init(this);
	this->autoBlip.init(this);
	this->autoShift.init(this);
	this->gearChanger.init(this);
	this->edl.init(this);
	this->buildARBS();
	this->abs.init(this);
	this->tractionControl.init(this);
	this->speedLimiter.init(this);
	this->sleepingFrames = 0;
	this->ksPhysics->cars.push_back(this);
	this->updateBodyMass();
	this->colliderManager.init(this);
	this->setupManager.init(this);
	if (!this->physicsGUID)
		this->telemetry.init(this);
	this->splineLocator.init(this);
	this->stabilityControl.init(this);
	this->driftMode.init(this);
	this->performanceMeter.init(this);
	this->lapInvalidator.init(this);
	this->penaltyManager.init(this);
	this->transponder.init(this);
	this->slipStream.init(this->ksPhysics);
	this->fuelLapEvaluator.init(this);

	this->ksPhysics->evOnNewSessionPhysics.add(this, [pThis](const SessionInfo& si) {
		pThis->onNewSession(si);
	});

	this->ksPhysics->evOnStepCompleted.add(this, [pThis](double dt) {
		pThis->postStep((float)dt);
	});

	ini = new_udt_unique<INIReader>(this->carDataPath + L"fuel_cons.ini");
	if (ini->hasSection(L"FUEL_EVAL"))
	{
		auto* pSpline = this->ksPhysics->track->aiSplineRecorder->getBestLapSpline();
		if (pSpline)
		{
			float fEval = ini->getFloat(L"FUEL_EVAL", L"KM_PER_LITER");
			this->expectedFuelPerLap = pSpline->spline.length() / (fEval * 1000.0f);
		}
	}

	auto re(new_udt_unique<RaceEngineer>(this));

	if (this->expectedFuelPerLap == 0.0f)
		this->expectedFuelPerLap = re->evaluateFuelPerLapFromTrackSpline();

	float fMaxTrack = tmax(re->getFrontTrack(), re->getRearTrack());
	this->carHalfWidth = (fMaxTrack * 0.5f) + 0.3f;

	float fMass = this->getTotalMass(true);
	this->powerClassIndex = this->drivetrain.acEngine.getMaxPowerW() / fMass;

	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Car::_initCarData()
{
	auto ini(new_udt_unique<INIReader>(this->carDataPath + L"car.ini"));
	if (!ini->ready)
	{
		SHOULD_NOT_REACH_FATAL;
		return;
	}

	this->screenName = ini->getString(L"INFO", L"SCREEN_NAME");
	this->mass = ini->getFloat(L"BASIC", L"TOTALMASS");

	if (ini->hasSection(L"EXPLICIT_INERTIA"))
	{
		this->explicitInertia = ini->getFloat3(L"EXPLICIT_INERTIA", L"INERTIA");
		this->body->setMassExplicitInertia(this->mass, this->explicitInertia.x, this->explicitInertia.y, this->explicitInertia.z);
	}
	else
	{
		this->bodyInertia = ini->getFloat3(L"BASIC", L"INERTIA");
		this->body->setMassBox(this->mass, this->bodyInertia.x, this->bodyInertia.y, this->bodyInertia.z);
	}

	if (ini->hasSection(L"FUEL_EXT"))
	{
		this->fuelKG = ini->getFloat(L"FUEL_EXT", L"KG_PER_LITER");
	}

	this->ffMult = ini->getFloat(L"CONTROLS", L"FFMULT") * 0.001f;
	this->steerLock = ini->getFloat(L"CONTROLS", L"STEER_LOCK");
	this->steerRatio = ini->getFloat(L"CONTROLS", L"STEER_RATIO");

	this->steerLinearRatio = ini->getFloat(L"CONTROLS", L"LINEAR_STEER_ROD_RATIO");
	if (this->steerLinearRatio == 0.0f)
		this->steerLinearRatio = 0.003f;

	this->steerAssist = ini->getFloat(L"CONTROLS", L"STEER_ASSIST");
	if (this->steerAssist == 0.0f)
		this->steerAssist = 1.0f;

	this->fuelConsumptionK = ini->getFloat(L"FUEL", L"CONSUMPTION");
	this->fuel = ini->getFloat(L"FUEL", L"FUEL");
	this->maxFuel = ini->getFloat(L"FUEL", L"MAX_FUEL");

	if (this->maxFuel == 0.0f)
		this->maxFuel = 30.0f;
	if (this->fuel == 0.0f)
		this->fuel = 30.0f;
	this->requestedFuel = (float)this->fuel;

	float fPickup = ini->getFloat(L"RIDE", L"PICKUP_FRONT_HEIGHT");
	this->ridePickupPoint[0] = vec3f(0, fPickup, 0);

	fPickup = ini->getFloat(L"RIDE", L"PICKUP_REAR_HEIGHT");
	this->ridePickupPoint[1] = vec3f(0, fPickup, 0);

	this->fuelTankPos = ini->getFloat3(L"FUELTANK", L"POSITION");
	this->fuelTankBody->setMassBox(1.0f, 0.5f, 0.5f, 0.5f);
	this->fuelTankBody->setPosition(this->fuelTankPos);

	auto* pCore = this->ksPhysics->getCore();
	this->fuelTankJoint = pCore->createFixedJoint(this->fuelTankBody, this->body);

	this->water.tmass = 20.0f;
	this->water.coolSpeedK = 0.002f;

	this->initPitstopTimings();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

static bool _dump_car_step0 = true;

void _Car::_step(float dt)
{
	#if defined(AC_DUMP_CAR_STEP0)
	if (_dump_car_step0)
	{
		_dump_car_step0 = false;

		ac_ostream out;
		out << *this;
		auto str = out.str();

		#if defined(AC_ENABLE_CUSTOM_PHYSICS)
		auto dumpName = L"_custom.json";
		#else
		auto dumpName = L"_orig.json";
		#endif

		std::wofstream fd(this->unixName + dumpName);
		fd << str;
		fd.close();
	}
	#endif

	#if defined(AC_ENABLE_CUSTOM_PHYSICS)
		auto bEnableCustom = true;
	#else
		auto bEnableCustom = false;
	#endif

	if (!bEnableCustom)
	{
		auto orig = ORIG_METHOD(Car, step);
		THIS_CALL(orig)(dt);
		return;
	}

	if (!this->physicsGUID)
	{
		vec3f vBodyVelocity = this->body->getVelocity();
		float fVelSq = vBodyVelocity.sqlen();
		float fERP;

		if (fVelSq >= 1.0f)
		{
			fERP = 0.3f;
			for (auto* pSusp : this->suspensions)
			{
				pSusp->setERPCFM(fERP, pSusp->baseCFM);
			}
		}
		else
		{
			fERP = 0.9f;
			for (auto* pSusp : this->suspensions)
			{
				pSusp->setERPCFM(fERP, 0.0000001f);
			}
		}

		this->fuelTankJoint->setERPCFM(fERP, -1.0f);
	}

	bool bControlsLocked = (this->isControlsLocked || (this->lockControlsTime > this->ksPhysics->physicsTime));

	if (GetAsyncKeyState(VK_NUMPAD0))
	{
		bControlsLocked = false;
		this->lockControlsTime = this->ksPhysics->physicsTime;
		this->isControlsLocked = false;
		this->isGentleStopping = false;
	}

	this->pollControls(dt);

	if (this->controlsProvider)
	{
		bool bHeadlightsSwitch = this->controlsProvider->getAction(DriverActions::eHeadlightsSwitch);

		if (bHeadlightsSwitch && !this->lastLigthSwitchState)
			this->lightsOn = !this->lightsOn;

		this->lastLigthSwitchState = bHeadlightsSwitch;
	}

	if (this->blackFlagged && !this->isInPits())
	{
		this->forceRotation(vec3f(&this->pitPosition.M31) * -1.0f);
		this->forcePosition(vec3f(&this->pitPosition.M41), true);
	}

	this->updateAirPressure();

	float fRpmAbs = fabsf(this->drivetrain.getEngineRPM());
	float fTurboBoost = tmax(0.0f, this->drivetrain.acEngine.status.turboBoost);
	double fNewFuel = this->fuel - (fRpmAbs * dt * this->drivetrain.acEngine.gasUsage) * (fTurboBoost + 1.0) * this->fuelConsumptionK * 0.001 * this->ksPhysics->fuelConsumptionRate;
	this->fuel = fNewFuel;

	if (fNewFuel > 0.0f)
	{
		this->drivetrain.acEngine.fuelPressure = 1.0f;
	}
	else
	{
		this->fuel = 0.0f;
		this->drivetrain.acEngine.fuelPressure = 0.0f;
	}

	this->updateBodyMass();

	if (this->controlsProvider)
	{
		if (bControlsLocked)
		{
			this->controls.gas = 0;
			this->controls.brake = 1;
			this->controls.steer = 0;
			this->controls.clutch = 0;
		}

		if (this->isGentleStopping)
		{
			#if 1
			this->controls.gas = 0.0;
			this->controls.brake = 0.2;
			#endif
		}

		if (this->penaltyTime > 0.0f) // TODO: implement
		{
		}
	}

	float fSteerAngleSig = (this->steerLock * this->controls.steer) / this->steerRatio;
	if (!isfinite(fSteerAngleSig))
	{
		SHOULD_NOT_REACH_WARN;
		fSteerAngleSig = 0.0f;
	}

	this->finalSteerAngleSignal = fSteerAngleSig;

	bool bAllTyresLoaded = true;
	for (int i = 0; i < 4; ++i)
	{
		if (this->tyres[i].status.load <= 0.0f)
		{
			bAllTyresLoaded = false;
			break;
		}
	}

	this->autoClutch.step(dt);

	float fSpeed = getSpeedMS(this);
	vec3f fAngVel = this->body->getAngularVelocity();
	float fAngVelSq = fAngVel.sqlen();

	if (fSpeed >= 0.5f || fAngVelSq >= 1.0f)
	{
		this->sleepingFrames = 0;
	}
	else
	{
		if (bAllTyresLoaded 
			&& (this->controls.gas <= 0.01f 
				|| this->controls.clutch <= 0.01f 
				|| this->drivetrain.currentGear == 1))
		{
			this->sleepingFrames++;
		}
		else
		{
			this->sleepingFrames = 0;
		}

		if (this->sleepingFrames > this->framesToSleep)
		{
			this->body->stop(0.99f);
			this->fuelTankBody->stop(0.99f);
		}
	}

	vec3f vBodyVel = this->body->getVelocity();
	vec3f vAccel = (vBodyVel - this->lastVelocity) * (1.0f / dt) * 0.10197838f;
	this->lastVelocity = vBodyVel;
	this->accG = this->body->worldToLocalNormal(vAccel);

	this->stepThermalObjects(dt);
	this->stepComponents(dt);
	this->updateColliderStatus(dt);

	if (!this->physicsGUID)
		this->stepJumpStart(dt);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Car::_updateAirPressure()
{
	float fAirDensity = this->ksPhysics->getAirDensity();

	if (this->slipStreamEffectGain > 0.0f)
	{
		vec3f vPos = this->body->getPosition(0.0f);
		float fMinSlip = 1.0f;

		for (SlipStream* pSS : this->ksPhysics->slipStreams)
		{
			if (pSS != &this->slipStream)
			{
				float fSlip = tclamp((1.0f - (pSS->getSlipEffect(vPos) * this->slipStreamEffectGain)), 0.0f, 1.0f);

				if (fMinSlip > fSlip)
					fMinSlip = fSlip;
			}
		}

		fAirDensity = ((fAirDensity - (fMinSlip * fAirDensity)) * (0.75f / this->slipStreamEffectGain)) + (fMinSlip * fAirDensity);
	}

	this->aeroMap.airDensity = fAirDensity;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Car::_updateBodyMass()
{
	if (this->ksPhysics->physicsTime - this->lastBodyMassUpdateTime > 1000.0)
	{
		if (this->bodyInertia.x != 0.0f || this->bodyInertia.y != 0.0f || this->bodyInertia.z != 0.0f)
		{
			float fBodyMass = this->calcBodyMass();
			this->body->setMassBox(fBodyMass, this->bodyInertia.x, this->bodyInertia.y, this->bodyInertia.z);
		}
		else
		{
			TODO_WTF_IS_THIS;

			this->body->setMassExplicitInertia(this->mass, this->explicitInertia.x, this->explicitInertia.y, this->explicitInertia.z);
			log_printf(L"setMassExplicitInertia");
		}

		float fFuelMass = tmax(0.1f, (this->fuelKG * (float)this->fuel));
		this->fuelTankBody->setMassBox(fFuelMass, 0.5f, 0.5f, 0.5f); // TODO: check

		this->lastBodyMassUpdateTime = this->ksPhysics->physicsTime;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _Car::_calcBodyMass()
{
	float fSuspMass = 0;
	for (int i = 0; i < 4; ++i)
		fSuspMass += this->suspensions[i]->getMass();

	return (this->mass - fSuspMass) + this->ballastKG;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Car::_stepThermalObjects(float dt)
{
	float fRpm = this->drivetrain.getEngineRPM();
	if (fRpm > (this->drivetrain.acEngine.data.minimum * 0.8f))
	{
		float fLimiter = (float)this->drivetrain.acEngine.getLimiterRPM();
		this->water.addHeadSource((((fRpm / fLimiter) * 20.0f) * this->controls.gas) + 85.0f);
	}
	
	this->water.step(dt, this->ksPhysics->ambientTemperature, this->getSpeed());
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Car::_stepComponents(float dt)
{
	this->brakeSystem.step(dt);
	this->edl.step(dt);

	for (auto* susp : this->suspensions)
	{
		susp->step(dt);
	}

	for (int i = 0; i < 4; ++i)
	{
		this->tyres[i].step(dt);
	}

	for (int i = 0; i < 2; ++i)
	{
		if (this->heaveSprings[i].k != 0.0f)
			this->heaveSprings[i].step(dt);
	}

	this->drs.step(dt);
	this->aeroMap.step(dt);

	if (this->kers.present)
		this->kers.step(dt);

	if (this->ers.present)
		this->ers.step(dt);

	this->steeringSystem.step(dt);
	this->autoBlip.step(dt);
	this->autoShift.step(dt);
	this->gearChanger.step(dt);
	this->drivetrain.step(dt);

	for (int i = 0; i < 2; ++i)
	{
		this->antirollBars[i].step(dt);
	}

	this->abs.step(dt);
	this->tractionControl.step(dt);
	this->speedLimiter.step(dt);
	this->colliderManager.step(dt);
	this->setupManager.step(dt);

	if (!this->physicsGUID)
	{
		this->telemetry.step(dt);
		this->driftMode.step(dt);
		this->performanceMeter.step(dt);
		this->lapInvalidator.step(dt);
		this->penaltyManager.step(dt);
	}

	this->splineLocator.step(dt);
	this->stabilityControl.step(dt);
	this->transponder.step(dt);
	this->fuelLapEvaluator.step(dt);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Car::_onCollisionCallBack(
	void* userData0, void* shape0, 
	void* userData1, void* shape1, 
	const vec3f& normal, const vec3f& pos, float depth)
{
	if (!(this->body == userData0 || this->body == userData1))
		return;

	IRigidBody* pOtherBody;
	ICollisionObject* pOtherShape;

	if (this->body == userData0)
	{
		pOtherBody = (IRigidBody*)userData1;
		pOtherShape = (ICollisionObject*)shape1;
	}
	else
	{
		pOtherBody = (IRigidBody*)userData0;
		pOtherShape = (ICollisionObject*)shape0;
	}

	unsigned long ulOtherGroup = pOtherShape ? pOtherShape->getGroup() : 0;
	unsigned long ulGroup0 = 0;
	unsigned long ulGroup1 = 0;
	bool bFlag0 = false;
	bool bFlag1 = false;

	if (shape0)
	{
		ulGroup0 = ((ICollisionObject*)shape0)->getGroup();
		bFlag0 = (ulGroup0 == 1 || ulGroup0 == 16);
	}

	if (shape1)
	{
		ulGroup1 = ((ICollisionObject*)shape1)->getGroup();
		bFlag1 = (ulGroup1 == 1 || ulGroup1 == 16);
	}

	this->lastCollisionTime = this->ksPhysics->physicsTime;

    vec3f vPosLocal = this->body->worldToLocal(pos);
	vec3f vVelocity = this->body->getPointVelocity(pos);

	vec3f vOtherVelocity(0, 0, 0);
	if (pOtherBody)
	{
		vOtherVelocity = pOtherBody->getPointVelocity(pos);
	}

	vec3f vDeltaVelocity = vVelocity - vOtherVelocity;
	float fRelativeSpeed = -((vDeltaVelocity * normal) * 3.6f);
	float fDamage = fRelativeSpeed * this->ksPhysics->mechanicalDamageRate;

	if (userData0 && userData1 && !bFlag0 && !bFlag1)
		this->lastCollisionWithCarTime = this->ksPhysics->physicsTime;

	float* pDmg = this->damageZoneLevel;

	if (fRelativeSpeed > 0.0f && !bFlag0 && !bFlag1)
	{
		if (fRelativeSpeed * this->ksPhysics->mechanicalDamageRate > 150.0f)
			this->drivetrain.acEngine.blowUp();

		vec3f vNorm = vPosLocal.get_norm();
		int iZoneId;

		if (fabsf(vNorm.z) <= 0.70700002f)
		{
			iZoneId = (vPosLocal.x >= 0.0f) ? 2 : 3;
		}
		else
		{
			iZoneId = (vPosLocal.z <= 0.0f) ? 1 : 0;
		}
		
		pDmg[iZoneId] = tmax(pDmg[iZoneId], fDamage);
		pDmg[4] = tmax(pDmg[4], fDamage);
	}

	if (pDmg[0] > 0.0f && pDmg[2] > 0.0f)
		this->suspensions[0]->setDamage((pDmg[0] + pDmg[2]) * 0.5f);

	if (pDmg[0] > 0.0f && pDmg[3] > 0.0f)
		this->suspensions[1]->setDamage((pDmg[0] + pDmg[3]) * 0.5f);

	if (pDmg[1] > 0.0f && pDmg[2] > 0.0f)
		this->suspensions[2]->setDamage((pDmg[1] + pDmg[2]) * 0.5f);

	if (pDmg[1] > 0.0f && pDmg[3] > 0.0f)
		this->suspensions[3]->setDamage((pDmg[1] + pDmg[3]) * 0.5f);

	ACPhysicsEvent pe;
	pe.type = eACEventType::acEvent_OnCollision;
	pe.param1 = (float)this->physicsGUID;
	pe.param2 = depth;
	pe.param3 = -1;
	pe.param4 = fRelativeSpeed;
	pe.vParam1 = pos;
	pe.vParam2 = normal;
	pe.voidParam0 = nullptr;
	pe.voidParam1 = nullptr;
	pe.ulParam0 = ulOtherGroup; // TODO: ulGroup1 OR ulOtherGroup ?
	this->ksPhysics->eventQueue.push(pe); // processed by Sim::stepPhysicsEvent

	if (fRelativeSpeed > 0.0f && !bFlag0 && !bFlag1)
	{
		//log_printf(L"collision %u %u %.3f", (UINT)ulGroup0, (UINT)ulGroup1, fRelativeSpeed);

		OnCollisionEvent ce;
		ce.body = pOtherBody;
		ce.relativeSpeed = fRelativeSpeed;
		ce.worldPos = pos;
		ce.relPos = vPosLocal;
		ce.colliderGroup = ulOtherGroup; // TODO: ulGroup1 OR ulOtherGroup ?

		for (auto& h : this->evOnCollisionEvent.handlers)
		{
			if (h.second)
				(h.second)(ce);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
