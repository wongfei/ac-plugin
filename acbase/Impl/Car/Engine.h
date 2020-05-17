#pragma once

BEGIN_HOOK_OBJ(Engine)

	#define RVA_Engine_vtable 0x4F8FD8
	#define RVA_Engine_ctor 2642608
	#define RVA_Engine_init 2645520
	#define RVA_Engine_loadINI 2646272
	#define RVA_Engine_step 2654432
	#define RVA_Engine_getThrottleResponseGas 2644880
	#define RVA_Engine_stepTurbos 2656512

	static void _hook()
	{
		HOOK_METHOD_RVA(Engine, ctor);
		HOOK_METHOD_RVA(Engine, init);
		HOOK_METHOD_RVA(Engine, loadINI);
		HOOK_METHOD_RVA(Engine, step);
		HOOK_METHOD_RVA(Engine, getThrottleResponseGas);
		HOOK_METHOD_RVA(Engine, stepTurbos);
	}

	Engine* _ctor();
	void _init(Car* pCar);
	void _loadINI();
	void _step(const SACEngineInput& input, float dt);
	float _getThrottleResponseGas(float gas, float rpm);
	void _stepTurbos();

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

Engine* _Engine::_ctor()
{
	AC_CTOR_VCLASS(Engine);

	AC_CTOR_UDT(this->data)();
	AC_CTOR_UDT(this->throttleResponseCurve)();
	AC_CTOR_UDT(this->throttleResponseCurveMax)();
	AC_CTOR_UDT(this->gasCoastOffsetCurve)();

	this->fuelPressure = 1.0;
	this->starterTorque = 20.0;
	this->p2p.basePositionCoeff = 1;
	this->inertia = 1.0;
	this->maxPowerW_Dynamic = -1.0;
	this->throttleResponseCurveMaxRef = 6000.0;
	this->bovThreshold = 0.2;

	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Engine::_init(Car* pCar)
{
	this->car = pCar;
	this->physicsEngine = pCar->ksPhysics;
	this->limiterMultiplier = 1.0f;
	this->coastTorqueMultiplier = 1.0f;
	this->electronicOverride = 1.0f;

	this->loadINI();
	this->reset();
	this->precalculatePowerAndTorque();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Engine::_loadINI()
{
	auto ini(new_udt_unique<INIReader>(this->car->carDataPath + L"engine.ini"));
	if (!ini->ready)
	{
		SHOULD_NOT_REACH_FATAL;
		return;
	}

	// HEADER

	auto strPowerCurve = ini->getString(L"HEADER", L"POWER_CURVE");
	this->data.powerCurve.load(this->car->carDataPath + strPowerCurve);

	auto strCoastCurve = ini->getString(L"HEADER", L"COAST_CURVE");
	if (strCoastCurve == L"FROM_COAST_REF")
	{
		auto coast = this->loadCoastSettings(*ini.get(), L"COAST_REF");
		this->data.coast1 = coast.coast1;
		this->data.coast2 = coast.coast2;
	}

	// ENGINE_DATA

	this->inertia = ini->getFloat(L"ENGINE_DATA", L"INERTIA");

	this->data.minimum = ini->getInt(L"ENGINE_DATA", L"MINIMUM");
	if (!this->data.minimum)
		this->data.minimum = 1000;

	this->data.limiter = ini->getInt(L"ENGINE_DATA", L"LIMITER");
	this->defaultEngineLimiter = this->data.limiter;

	if (this->defaultEngineLimiter)
	{
		this->rpmDamageThreshold = this->defaultEngineLimiter * 1.05f;
		this->rpmDamageK = 10.0f;
	}

	this->data.limiterCycles = ini->getInt(L"ENGINE_DATA", L"LIMITER_HZ");
	if (this->data.limiterCycles)
		this->data.limiterCycles = 1000 / this->data.limiterCycles / 3; // TODO: check
	else
		this->data.limiterCycles = 50;

	// COAST_SETTINGS

	if (ini->hasSection(L"COAST_SETTINGS"))
	{
		this->gasCoastOffsetCurve = ini->getCurve(L"COAST_SETTINGS", L"LUT");
		this->coastSettingsDefaultIndex = ini->getInt(L"COAST_SETTINGS", L"DEFAULT");
		this->setCoastSettings(this->coastSettingsDefaultIndex);
		this->coastEntryRpm = this->data.minimum + ini->getInt(L"COAST_SETTINGS", L"ACTIVATION_RPM");
	}

	// TURBO

	for (int id = 0; ; id++)
	{
		auto sectionName = strf(L"TURBO_%d", id);
		if (!ini->hasSection(sectionName))
			break;

		TurboDef td;
		td.lagDN = (1.0f - ini->getFloat(sectionName, L"LAG_DN")) * 1.333333f * 333.3333f;
		td.lagUP = (1.0f - ini->getFloat(sectionName, L"LAG_UP")) * 1.333333f * 333.3333f;
		td.maxBoost = ini->getFloat(sectionName, L"MAX_BOOST");
		td.wastegate = ini->getFloat(sectionName, L"WASTEGATE");
		td.rpmRef = ini->getFloat(sectionName, L"REFERENCE_RPM");
		td.gamma = ini->getFloat(sectionName, L"GAMMA");
		td.isAdjustable = (ini->getInt(sectionName, L"COCKPIT_ADJUSTABLE") != 0);

		if (td.isAdjustable)
			this->turboAdjustableFromCockpit = true;

		auto turbo(new_udt_unique<Turbo>(td));
		this->turbos.push_back(*turbo.get());
	}

	if (this->turboAdjustableFromCockpit)
	{
		float fBoost = ini->getFloat(L"ENGINE_DATA", L"DEFAULT_TURBO_ADJUSTMENT");
		this->setTurboBoostLevel(fBoost);
	}

	// OVERLAP

	if (ini->hasSection(L"OVERLAP"))
	{
		this->data.overlapFreq = ini->getFloat(L"OVERLAP", L"FREQUENCY");
		this->data.overlapGain = ini->getFloat(L"OVERLAP", L"GAIN");
		this->data.overlapIdealRPM = ini->getFloat(L"OVERLAP", L"IDEAL_RPM");
	}

	// THROTTLE

	this->throttleResponseCurve.load(this->car->carDataPath + L"throttle.lut");

	// DAMAGE

	if (ini->hasSection(L"DAMAGE"))
	{
		if (!this->turbos.empty())
		{
			this->turboBoostDamageThreshold = ini->getFloat(L"DAMAGE", L"TURBO_BOOST_THRESHOLD");
			this->turboBoostDamageK = ini->getFloat(L"DAMAGE", L"TURBO_DAMAGE_K");
		}

		this->rpmDamageThreshold = ini->getFloat(L"DAMAGE", L"RPM_THRESHOLD");
		this->rpmDamageK = ini->getFloat(L"DAMAGE", L"RPM_DAMAGE_K");
	}

	// BOV

	if (ini->hasSection(L"BOV"))
	{
		this->bovThreshold = ini->getFloat(L"BOV", L"PRESSURE_THRESHOLD");
	}

	// TURBO CONTROLLER

	if (!this->turbos.empty())
	{
		int id = 0;
		for (auto& turbo : this->turbos)
		{
			auto strIniPath = this->car->carDataPath + strf(L"ctrl_turbo%d.ini", id);
			if (Path::fileExists(strIniPath, false))
			{
				auto dc(new_udt_unique<DynamicController>(this->car, strIniPath));
				auto tdc(new_udt_unique<TurboDynamicController>());
				tdc->turbo = &turbo;
				tdc->controller = *dc;
				this->turboControllers.push_back(*tdc.get());
			}

			strIniPath = this->car->carDataPath + strf(L"ctrl_wastegate%d.ini", id);
			if (Path::fileExists(strIniPath, false))
			{
				auto dc(new_udt_unique<DynamicController>(this->car, strIniPath));
				auto tdc(new_udt_unique<TurboDynamicController>());
				tdc->turbo = &turbo;
				tdc->controller = *dc;
				tdc->isWastegate = true;
				this->turboControllers.push_back(*tdc.get());
			}

			id++;
		}
	}

	// PUSH_TO_PASS

	#if 0
	if (ini->hasSection(L"PUSH_TO_PASS"))
	{
		TODO_NOT_IMPLEMENTED;
	}
	#endif

	// THROTTLE_RESPONSE

	if (ini->hasSection(L"THROTTLE_RESPONSE"))
	{
		this->throttleResponseCurveMaxRef = ini->getFloat(L"THROTTLE_RESPONSE", L"RPM_REFERENCE");
		this->throttleResponseCurveMax = ini->getCurve(L"THROTTLE_RESPONSE", L"LUT");
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Engine::_step(const SACEngineInput& input, float dt)
{
	this->lastInput = input;
	this->lastInput.gasInput = this->getThrottleResponseGas(input.gasInput, input.rpm);

	if (this->p2p.enabled)
	{
		this->stepP2P(dt);
	}

	if (this->gasCoastOffset > 0.0f)
	{
		float fGas1 = (input.rpm - (float)this->data.minimum) / (float)this->coastEntryRpm;
		fGas1 = tclamp(fGas1, 0.0f, 1.0f);

		float fGas2 = ((1.0f - (this->gasCoastOffset * fGas1)) * this->lastInput.gasInput) + (this->gasCoastOffset * fGas1);
		fGas2 = tclamp(fGas2, 0.0f, 1.0f);

		this->lastInput.gasInput = fGas2;
	}

	int iLimiter = this->data.limiter;
	if (iLimiter && (iLimiter * this->limiterMultiplier) < this->lastInput.rpm)
	{
		this->limiterOn = this->data.limiterCycles;
	}

	if (this->limiterOn > 0)
	{
		this->lastInput.gasInput = 0.0f;
		this->limiterOn--;
	}

	if (this->lifeLeft <= 0.0f)
	{
		this->fuelPressure = 0.0f;
	}

	float fGas = this->lastInput.gasInput * this->electronicOverride;
	this->lastInput.gasInput = fGas;
	this->gasUsage = fGas;

	float fPower = this->data.powerCurve.getValue(this->lastInput.rpm);
	float fCoastTorq = 0.0f;

	if (this->data.coast1 != 0.0f)
	{
		fCoastTorq = (this->lastInput.rpm - (float)this->data.minimum) * this->data.coast1;
	}

	this->stepTurbos();

	if (this->status.turboBoost != 0.0f)
	{
		fPower *= (this->status.turboBoost + 1.0f);
	}

	if (this->data.coast2 != 0.0f)
	{
		float fRpmDelta = this->lastInput.rpm - (float)this->data.minimum;
		fCoastTorq -= (((fRpmDelta * fRpmDelta) * this->data.coast2) * signf(this->lastInput.rpm));
	}

	this->status.externalCoastTorque = 0.0f;
	for (auto* pCoastGen : this->coastGenerators)
	{
		this->status.externalCoastTorque += pCoastGen->getCoastTorque();
	}
	fCoastTorq += (float)this->status.externalCoastTorque;

	if (this->lastInput.rpm <= (float)this->data.minimum)
	{
		this->status.externalCoastTorque = 0.0f;
		fCoastTorq = 0.0f;
	}

	float fTurboBoost = this->status.turboBoost;
	if (((1.0f - this->lastInput.gasInput) * fTurboBoost) <= this->bovThreshold)
		this->bov = 0.0f;
	else
		this->bov = 1.0f;

	float fTbDamageThresh = this->turboBoostDamageThreshold;
	if (fTbDamageThresh != 0.0f && fTurboBoost > fTbDamageThresh)
	{
		this->lifeLeft -= ((((fTurboBoost - fTbDamageThresh) * this->turboBoostDamageK) * 0.003f) * this->physicsEngine->mechanicalDamageRate);
	}

	float fRpmDamageThresh = this->rpmDamageThreshold;
	if (fRpmDamageThresh != 0.0f && this->lastInput.rpm > fRpmDamageThresh)
	{
		this->lifeLeft -= ((((this->lastInput.rpm - fRpmDamageThresh) * this->rpmDamageK) * 0.003f) * this->physicsEngine->mechanicalDamageRate);
	}

	float fAirAmount = this->physicsEngine->getAirDensity() * 0.82630974f;
	float fRestrictor = this->restrictor;
	if (fRestrictor > 0.0f)
	{
		fAirAmount -= (((fRestrictor * input.rpm) * 0.0001f) * fGas);
		if (fAirAmount < 0.0f)
			fAirAmount = 0.0f;
	}

	float fOutTorq = ((((fPower - fCoastTorq) * fGas) + fCoastTorq) * fAirAmount);
	this->status.outTorque = fOutTorq;

	bool bHasFuelPressure = (this->fuelPressure > 0.0f);
	if (bHasFuelPressure)
	{
		float fRpm = this->lastInput.rpm;
		if (fRpm >= (float)this->data.minimum)
		{
			float fOverlapGain = this->data.overlapGain;
			if (fOverlapGain != 0.0f)
			{
				float fOverlap = sinf((float)this->physicsEngine->physicsTime * 0.001f * this->data.overlapFreq * fRpm * 0.0003333333333333333f) * 0.5f - 0.5f;
				this->status.outTorque = (fOverlap * fabsf(fRpm - this->data.overlapIdealRPM) * fOverlapGain) + fOutTorq;
			}
		}
		else if (this->isEngineStallEnabled)
		{
			float fStallTorq = fRpm * -0.01f;
			if (GetAsyncKeyState(VK_BACK)) // LOL
				fStallTorq = this->starterTorque;

			this->status.outTorque = fStallTorq;
		}
		else
		{
			this->status.outTorque = tmax(15.0f, fOutTorq);
		}
	}

	if (this->fuelPressure < 1.0f)
	{
		this->status.outTorque = (this->status.outTorque - this->lastInput.rpm * -0.01f) * this->fuelPressure + this->lastInput.rpm * -0.01f;
	}

	for (auto* pTorqGen : this->torqueGenerators)
	{
		this->status.outTorque += pTorqGen->getOutputTorque();
	}

	bool bLimiterOn = (this->limiterOn != 0);
	this->status.isLimiterOn = bLimiterOn;
	this->electronicOverride = 1.0f;

	float fMaxPowerDyn = this->maxPowerW_Dynamic;
	float fCurPower = input.rpm * (float)this->status.outTorque * 0.1047f;
	if (fCurPower > fMaxPowerDyn)
	{
		this->maxPowerW_Dynamic = fCurPower;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _Engine::_getThrottleResponseGas(float gas, float rpm)
{
	float result = 0;

	if (this->throttleResponseCurve.getCount() && this->throttleResponseCurveMax.getCount())
	{
		float fTrc = tclamp(this->throttleResponseCurve.getValue(gas * 100.0f) * 0.01f, 0.0f, 1.0f);
		float fTrcMax =  tclamp(this->throttleResponseCurveMax.getValue(gas * 100.0f) * 0.01f, 0.0f, 1.0f);
		float fTrcScale = tclamp(rpm / this->throttleResponseCurveMaxRef, 0.0f, 1.0f);

		result = ((fTrcMax - fTrc) * fTrcScale) + fTrc;
	}
	else if (this->throttleResponseCurve.getCount())
	{
		result = tclamp(this->throttleResponseCurve.getValue(gas * 100.0f) * 0.01f, 0.0f, 1.0f);
	}
	else
	{
		result = gas;
	}

	return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Engine::_stepTurbos()
{
	for (auto& tc : this->turboControllers)
	{
		if (tc.isWastegate)
			tc.turbo->data.wastegate = tc.controller.eval();
		else
			tc.turbo->data.maxBoost = tc.controller.eval();
	}

	this->status.turboBoost = 0.0;
	for (auto& turbo : this->turbos)
	{
		turbo.step(this->lastInput.gasInput, this->lastInput.rpm, 0.003f); // TODO: check
		this->status.turboBoost += (turbo.getBoost() * this->fuelPressure);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
