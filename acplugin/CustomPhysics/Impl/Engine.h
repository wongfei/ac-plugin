#pragma once

BEGIN_HOOK_OBJ(Engine)

	#define RVA_Engine_step 2654432
	#define RVA_Engine_getThrottleResponseGas 2644880
	#define RVA_Engine_stepTurbos 2656512

	static void _hook()
	{
		HOOK_METHOD_RVA(Engine, step);
		HOOK_METHOD_RVA(Engine, getThrottleResponseGas);
		HOOK_METHOD_RVA(Engine, stepTurbos);
	}

	void _step(const SACEngineInput& input, float dt);
	float _getThrottleResponseGas(float gas, float rpm);
	void _stepTurbos();

END_HOOK_OBJ()

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
		fAirAmount -= (((fRestrictor * input.rpm) * 0.00009999999f) * fGas);
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
			float fStallTorq = fRpm * -0.009999999f;
			if (GetAsyncKeyState(8)) // LOL
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
	float fCurPower = input.rpm * (float)this->status.outTorque * 0.104699999f;
	if (fCurPower > fMaxPowerDyn)
	{
		this->maxPowerW_Dynamic = fCurPower;
	}
}

float _Engine::_getThrottleResponseGas(float gas, float rpm)
{
	float result = 0;

	if (this->throttleResponseCurve.getCount() && this->throttleResponseCurveMax.getCount())
	{
		float fTrc = tclamp(this->throttleResponseCurve.getValue(gas * 100.0f) * 0.0099999998f, 0.0f, 1.0f);
		float fTrcMax =  tclamp(this->throttleResponseCurveMax.getValue(gas * 100.0f) * 0.0099999998f, 0.0f, 1.0f);
		float fTrcScale = tclamp(rpm / this->throttleResponseCurveMaxRef, 0.0f, 1.0f);

		result = ((fTrcMax - fTrc) * fTrcScale) + fTrc;
	}
	else if (this->throttleResponseCurve.getCount())
	{
		result = tclamp(this->throttleResponseCurve.getValue(gas * 100.0f) * 0.0099999998f, 0.0f, 1.0f);
	}
	else
	{
		result = gas;
	}

	return result;
}

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
