#pragma once

BEGIN_HOOK_OBJ(Engine)

	#define RVA_Engine_step 2654432
	#define RVA_Engine_stepTurbos 2656512

	void _step(SACEngineInput* pInput, float dt);
	void _stepTurbos();

END_HOOK_OBJ()

void _Engine::_step(SACEngineInput* pInput, float dt)
{
	this->lastInput = *pInput;
	float fGas = this->getThrottleResponseGas(this->lastInput.gasInput, pInput->rpm);
	this->lastInput.gasInput = fGas;

	if (this->p2p.enabled)
	{
		this->stepP2P(dt);
	}

	float fCoastOffset = this->gasCoastOffset;
	if (fCoastOffset > 0.0f)
	{
		float fGas1 = (pInput->rpm - (float)this->data.minimum) / (float)this->coastEntryRpm;
		fGas1 = tclamp(fGas1, 0.0f, 1.0f);

		float fGas2 = ((1.0f - (fCoastOffset * fGas1)) * this->lastInput.gasInput) + (fCoastOffset * fGas1);
		fGas2 = tclamp(fGas2, 0.0f, 1.0f);

		this->lastInput.gasInput = fGas2;
	}

	int iLimiter = this->data.limiter;
	if (iLimiter && (iLimiter * this->limiterMultiplier) < this->lastInput.rpm)
	{
		this->limiterOn = this->data.limiterCycles;
	}

	int iLimiterOn = this->limiterOn;
	if (iLimiterOn > 0)
	{
		this->lastInput.gasInput = 0.0f;
		this->limiterOn = iLimiterOn - 1;
	}

	if (this->lifeLeft <= 0.0f)
	{
		this->fuelPressure = 0.0f;
	}

	float fGasOverride = this->electronicOverride * this->lastInput.gasInput;
	this->lastInput.gasInput = fGasOverride;
	this->gasUsage = fGasOverride;

	float fPower = this->data.powerCurve.getValue(this->lastInput.rpm);
	float fCoastTorq = 0.0f;

	float fCoast1 = this->data.coast1;
	if (fCoast1 != 0.0f)
	{
		fCoastTorq = (this->lastInput.rpm - (float)this->data.minimum) * fCoast1;
	}

	this->stepTurbos();

	float fTurboBoost = this->status.turboBoost;
	if (fTurboBoost != 0.0f)
	{
		fTurboBoost += 1.0f;
		fPower *= fTurboBoost;
	}

	float fCoast2 = this->data.coast2;
	if (fCoast2 != 0.0f)
	{
		if (this->lastInput.rpm <= 0.0f)
		{
			if (this->lastInput.rpm >= 0.0f)
				fTurboBoost = 0.0f;
			else
				fTurboBoost = -1.0f;
		}
		else
		{
			fTurboBoost = 1.0f;
		}
		float fRpmDelta = this->lastInput.rpm - (float)this->data.minimum;
		fCoastTorq -= (((fRpmDelta * fRpmDelta) * fCoast2) * fTurboBoost);
	}

	this->status.externalCoastTorque = 0.0f;
	for (auto& iter : this->coastGenerators)
	{
		this->status.externalCoastTorque += iter->getCoastTorque();
	}
	fCoastTorq += (float)this->status.externalCoastTorque;

	if (this->lastInput.rpm <= (float)this->data.minimum)
	{
		this->status.externalCoastTorque = 0.0f;
		fCoastTorq = 0.0f;
	}

	float fTurboBoost2 = this->status.turboBoost;
	if (((1.0f - this->lastInput.gasInput) * fTurboBoost2) <= this->bovThreshold)
		this->bov = 0.0f;
	else
		this->bov = 1.0f;

	float fTbDamageThresh = this->turboBoostDamageThreshold;
	if (fTbDamageThresh != 0.0f && fTurboBoost2 > fTbDamageThresh)
	{
		this->lifeLeft -= ((((fTurboBoost2 - fTbDamageThresh) * this->turboBoostDamageK) * 0.003f) * this->physicsEngine->mechanicalDamageRate);
	}

	float fRpmDamageThresh = this->rpmDamageThreshold;
	if (fRpmDamageThresh != 0.0f && this->lastInput.rpm > fRpmDamageThresh)
	{
		this->lifeLeft -= ((((this->lastInput.rpm - fRpmDamageThresh) * this->rpmDamageK) * 0.003f) * this->physicsEngine->mechanicalDamageRate);
	}

	float fAirDens = this->physicsEngine->getAirDensity();
	float fAirAmount = fAirDens * 0.82630974f;
	float fRestrictor = this->restrictor;
	if (fRestrictor > 0.0f)
	{
		fAirAmount -= (((fRestrictor * pInput->rpm) * 0.0001f) * fGasOverride);
		if (fAirAmount < 0.0f)
			fAirAmount = 0.0f;
	}

	float fOutTorq = ((((fPower - fCoastTorq) * fGasOverride) + fCoastTorq) * fAirAmount);
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
			//if (GetAsyncKeyState(8)) fStallTorq = this->starterTorque; // TODO: WTF?
			this->status.outTorque = fStallTorq;
		}
		else
		{
			this->status.outTorque = tmax(15.0f, fOutTorq);
		}
	}

	float fFuelPressure = this->fuelPressure;
	if (fFuelPressure < 1.0f)
	{
		this->status.outTorque = (this->status.outTorque - this->lastInput.rpm * -0.01f) * fFuelPressure + this->lastInput.rpm * -0.01f;
	}

	for (auto& iter : this->torqueGenerators)
	{
		this->status.outTorque += iter->getOutputTorque();
	}

	bool bLimiterOn = (this->limiterOn != 0);
	this->status.isLimiterOn = bLimiterOn;
	this->electronicOverride = 1.0f;

	float fMaxPowerDyn = this->maxPowerW_Dynamic;
	float fCurPower = pInput->rpm * (float)this->status.outTorque * 0.1047f;
	if (fCurPower > fMaxPowerDyn)
	{
		this->maxPowerW_Dynamic = fCurPower;
	}
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
		float fRpm = this->lastInput.rpm > 0.0f ? this->lastInput.rpm : 0.0f;
		turbo.step(this->lastInput.gasInput, fRpm, 0.003f);
		float fBoost = turbo.getBoost();
		this->status.turboBoost += (fBoost * this->fuelPressure);
	}
}
