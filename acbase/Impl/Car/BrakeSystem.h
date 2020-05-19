#pragma once

BEGIN_HOOK_OBJ(BrakeSystem)

	#define RVA_BrakeSystem_ctor 2539040
	#define RVA_BrakeSystem_init 2676368
	#define RVA_BrakeSystem_loadINI 2676848
	#define RVA_BrakeSystem_reset 2679952
	#define RVA_BrakeSystem_step 2680384
	#define RVA_BrakeSystem_stepTemps 2681120

	static void _hook()
	{
		HOOK_METHOD_RVA(BrakeSystem, ctor);
		HOOK_METHOD_RVA(BrakeSystem, init);
		HOOK_METHOD_RVA(BrakeSystem, loadINI);
		HOOK_METHOD_RVA(BrakeSystem, reset);
		HOOK_METHOD_RVA(BrakeSystem, step);
		HOOK_METHOD_RVA(BrakeSystem, stepTemps);
	}

	BrakeSystem* _ctor();
	void _init(Car* car);
	void _loadINI(const std::wstring& dataPath);
	void _reset();
	void _step(float dt);
	void _stepTemps(float dt);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

BrakeSystem* _BrakeSystem::_ctor() // TODO: cleanup
{
	AC_CTOR_THIS_POD(BrakeSystem);

	for (auto& iter : this->discs) { AC_CTOR_UDT(iter)(); }
	AC_CTOR_UDT(this->steerBrake.controller)();
	AC_CTOR_UDT(this->ebbController)();

	this->frontBias = 0.7;
	this->brakePowerMultiplier = 1.0;
	this->ebbInstant = 0.5;
	this->limitUp = 1.0;
	this->biasOverride = -1.0;
	this->hasCockpitBias = 1;
	this->biasStep = 0.005;
	this->ebbFrontMultiplier = 1.1;

	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _BrakeSystem::_init(Car* car)
{
	this->car = car;
	this->brakePower = 2000.0;
	this->frontBias = 0.7;
	this->brakePowerMultiplier = 1.0;

	auto strDataPath(this->car->carDataPath);
	this->loadINI(&strDataPath);

	auto strPath = car->carDataPath + L"ctrl_ebb.ini";
	if (Path::fileExists(strPath, false))
	{
		auto dc(new_udt_unique<DynamicController>(this->car, strPath));
		this->ebbController = *dc.get();
		this->ebbMode = EBBMode::DynamicController;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _BrakeSystem::_loadINI(const std::wstring& dataPath)
{
	auto ini(new_udt_unique<INIReader>(dataPath + L"brakes.ini"));
	if (!ini->ready)
	{
		SHOULD_NOT_REACH_FATAL;
		return;
	}

	this->brakePower = ini->getFloat(L"DATA", L"MAX_TORQUE");
	this->frontBias = ini->getFloat(L"DATA", L"FRONT_SHARE");
	this->handBrakeTorque = ini->getFloat(L"DATA", L"HANDBRAKE_TORQUE");
	this->hasCockpitBias = (ini->getInt(L"DATA", L"COCKPIT_ADJUSTABLE") != 0);
	this->biasStep = ini->getFloat(L"DATA", L"ADJUST_STEP") * 0.01f;
	
	if (ini->hasSection(L"EBB"))
	{
		this->ebbMode = EBBMode::Internal;
		this->ebbFrontMultiplier = tmax(1.1f, ini->getFloat(L"EBB", L"FRONT_SHARE_MULTIPLIER"));
	}

	auto strPath = dataPath + L"steer_brake_controller.ini";
	if (Path::fileExists(strPath, false))
	{
		auto dc(new_udt_unique<DynamicController>(this->car, strPath));
		this->steerBrake.controller = *dc.get();
		this->steerBrake.isActive = true;
	}

	if (ini->hasSection(L"TEMPS_FRONT") && ini->hasSection(L"TEMPS_REAR")) // test on F40
	{
		std::wstring strId[] = {L"FRONT", L"FRONT", L"REAR", L"REAR"};
		for (int id = 0; id < 4; ++id)
		{
			auto strTemps = strf(L"TEMPS_") + strId[id];
			this->discs[id].perfCurve = ini->getCurve(strTemps, L"PERF_CURVE");
			this->discs[id].torqueK = ini->getFloat(strTemps, L"TORQUE_K");
			this->discs[id].coolTransfer = ini->getFloat(strTemps, L"COOL_TRANSFER");
			this->discs[id].coolSpeedFactor = ini->getFloat(strTemps, L"COOL_SPEED_FACTOR");
		}
		this->hasBrakeTempsData = true;
	}
	else
	{
		this->hasBrakeTempsData = false;
	}

	ini = new_udt_unique<INIReader>(dataPath + L"setup.ini");
	if (ini->ready)
	{
		if (ini->hasSection(L"FRONT_BIAS"))
		{
			this->limitDown = ini->getFloat(L"FRONT_BIAS", L"MIN") * 0.01f;
			this->limitUp = ini->getFloat(L"FRONT_BIAS", L"MAX") * 0.01f;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _BrakeSystem::_reset()
{
	this->biasOverride = -1.0f;
	for (auto& iter : this->discs)
	{
		iter.t = this->car->ksPhysics->ambientTemperature;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _BrakeSystem::_step(float dt)
{
	float fFrontBias = this->frontBias;

	if (this->biasOverride != -1.0f)
		fFrontBias = this->biasOverride;

	if (this->ebbMode != EBBMode::Internal)
	{
		if (this->ebbMode == EBBMode::DynamicController)
			fFrontBias = this->ebbController.eval();
	}
	else
	{
		float fLoadFront = this->car->tyres[1].status.load + this->car->tyres[0].status.load;
		float fLoadAWD = (this->car->tyres[3].status.load + this->car->tyres[2].status.load) + fLoadFront;
		bool bFlag = false;

		if (fLoadAWD != 0.0f)
		{
			if (getSpeedKMH(this->car) > 10.0f)
				bFlag = true;
		}

		if (bFlag)
		{
			this->ebbInstant = tclamp(((fLoadFront / fLoadAWD) * this->ebbFrontMultiplier), 0.0f, 1.0f);
		}
		else
		{
			this->ebbInstant = this->frontBias;
		}

		fFrontBias = this->ebbInstant;
	}

	fFrontBias = tclamp(fFrontBias, this->limitDown, this->limitUp);

	float fBrakeInput = tmax(this->car->controls.brake, this->electronicOverride);
	float fBrakeTorq = (this->brakePower * this->brakePowerMultiplier) * fBrakeInput;

	this->car->tyres[0].inputs.brakeTorque = fBrakeTorq * fFrontBias;
	this->car->tyres[1].inputs.brakeTorque = fBrakeTorq * fFrontBias;

	float fRearBrakeTorq = ((1.0f - fFrontBias) * fBrakeTorq) - this->rearCorrectionTorque;
	if (fRearBrakeTorq < 0.0f)
		fRearBrakeTorq = 0.0f;

	this->car->tyres[2].inputs.brakeTorque = fRearBrakeTorq;
	this->car->tyres[3].inputs.brakeTorque = fRearBrakeTorq;

	this->car->tyres[2].inputs.handBrakeTorque = this->car->controls.handBrake * this->handBrakeTorque;
	this->car->tyres[3].inputs.handBrakeTorque = this->car->controls.handBrake * this->handBrakeTorque;

	if (this->steerBrake.isActive)
	{
		float fSteerBrake = this->steerBrake.controller.eval();
		if (fSteerBrake >= 0.0f)
			this->car->tyres[3].inputs.brakeTorque += fSteerBrake;
		else
			this->car->tyres[2].inputs.brakeTorque -= fSteerBrake;
	}

	if (this->hasBrakeTempsData && this->car->tyres[0].aiMult <= 1.0f)
		this->stepTemps(dt);

	this->electronicOverride = 0.0f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _BrakeSystem::_stepTemps(float dt)
{
	float fAmbientTemp = this->car->ksPhysics->ambientTemperature;
	float fSpeed = getSpeedKMH(this->car);

	Tyre* pTyre = this->car->tyres;
	BrakeDisc* pDisc = this->discs;

	for (int i = 0; i < 4; ++i)
	{
		pTyre->inputs.brakeTorque = pDisc->perfCurve.getValue(pDisc->t) * pTyre->inputs.brakeTorque;

		float fCool = ((fSpeed * pDisc->coolSpeedFactor) + 1.0f) * pDisc->coolTransfer;

		pDisc->t += (((fAmbientTemp - pDisc->t) * fCool) * dt);
		pDisc->t += (((fabsf(pTyre->status.angularVelocity) * (pTyre->inputs.brakeTorque * pDisc->torqueK)) * 0.001f) * dt);

		pTyre++;
		pDisc++;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
