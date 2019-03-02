#include "precompiled.h"
#include "GameHooks.h"

void Engine_step(Engine* pThis, SACEngineInput* pInput, float dt)
{
	pThis->lastInput = *pInput;
	float fGas = pThis->getThrottleResponseGas(pThis->lastInput.gasInput, pInput->rpm);
	pThis->lastInput.gasInput = fGas;

	if (pThis->p2p.enabled)
	{
		pThis->stepP2P(dt);
	}

	float fCoastOffset = pThis->gasCoastOffset;
	if (fCoastOffset > 0.0f)
	{
		float fGas1 = (pInput->rpm - (float)pThis->data.minimum) / (float)pThis->coastEntryRpm;
		fGas1 = tclamp(fGas1, 0.0f, 1.0f);

		float fGas2 = ((1.0f - (fCoastOffset * fGas1)) * pThis->lastInput.gasInput) + (fCoastOffset * fGas1);
		fGas2 = tclamp(fGas2, 0.0f, 1.0f);

		pThis->lastInput.gasInput = fGas2;
	}

	int iLimiter = pThis->data.limiter;
	if (iLimiter && (iLimiter * pThis->limiterMultiplier) < pThis->lastInput.rpm)
	{
		pThis->limiterOn = pThis->data.limiterCycles;
	}

	int iLimiterOn = pThis->limiterOn;
	if (iLimiterOn > 0)
	{
		pThis->lastInput.gasInput = 0.0f;
		pThis->limiterOn = iLimiterOn - 1;
	}

	if (pThis->lifeLeft <= 0.0f)
	{
		pThis->fuelPressure = 0.0f;
	}

	float fGasOverride = pThis->electronicOverride * pThis->lastInput.gasInput;
	pThis->lastInput.gasInput = fGasOverride;
	pThis->gasUsage = fGasOverride;

	float fPower = pThis->data.powerCurve.getValue(pThis->lastInput.rpm);
	float fCoastTorq = 0.0f;

	float fCoast1 = pThis->data.coast1;
	if (fCoast1 != 0.0f)
	{
		fCoastTorq = (pThis->lastInput.rpm - (float)pThis->data.minimum) * fCoast1;
	}

	pThis->stepTurbos();

	float fTurboBoost = pThis->status.turboBoost;
	if (fTurboBoost != 0.0f)
	{
		fTurboBoost += 1.0f;
		fPower *= fTurboBoost;
	}

	float fCoast2 = pThis->data.coast2;
	if (fCoast2 != 0.0f)
	{
		if (pThis->lastInput.rpm <= 0.0f)
		{
			if (pThis->lastInput.rpm >= 0.0f)
				fTurboBoost = 0.0f;
			else
				fTurboBoost = -1.0f;
		}
		else
		{
			fTurboBoost = 1.0f;
		}
		float fRpmDelta = pThis->lastInput.rpm - (float)pThis->data.minimum;
		fCoastTorq -= (((fRpmDelta * fRpmDelta) * fCoast2) * fTurboBoost);
	}

	pThis->status.externalCoastTorque = 0.0f;
	for (auto& iter : pThis->coastGenerators)
	{
		pThis->status.externalCoastTorque += iter->getCoastTorque();
	}
	fCoastTorq += (float)pThis->status.externalCoastTorque;

	if (pThis->lastInput.rpm <= (float)pThis->data.minimum)
	{
		pThis->status.externalCoastTorque = 0.0f;
		fCoastTorq = 0.0f;
	}

	float fTurboBoost2 = pThis->status.turboBoost;
	if (((1.0f - pThis->lastInput.gasInput) * fTurboBoost2) <= pThis->bovThreshold)
		pThis->bov = 0.0f;
	else
		pThis->bov = 1.0f;

	float fTbDamageThresh = pThis->turboBoostDamageThreshold;
	if (fTbDamageThresh != 0.0f && fTurboBoost2 > fTbDamageThresh)
	{
		pThis->lifeLeft -= ((((fTurboBoost2 - fTbDamageThresh) * pThis->turboBoostDamageK) * 0.003f) * pThis->physicsEngine->mechanicalDamageRate);
	}

	float fRpmDamageThresh = pThis->rpmDamageThreshold;
	if (fRpmDamageThresh != 0.0f && pThis->lastInput.rpm > fRpmDamageThresh)
	{
		pThis->lifeLeft -= ((((pThis->lastInput.rpm - fRpmDamageThresh) * pThis->rpmDamageK) * 0.003f) * pThis->physicsEngine->mechanicalDamageRate);
	}

	float fAirDens = pThis->physicsEngine->getAirDensity();
	float fAirAmount = fAirDens * 0.82630974f;
	float fRestrictor = pThis->restrictor;
	if (fRestrictor > 0.0f)
	{
		fAirAmount -= (((fRestrictor * pInput->rpm) * 0.000099999997f) * fGasOverride);
		if (fAirAmount < 0.0f)
			fAirAmount = 0.0f;
	}

	float fOutTorq = ((((fPower - fCoastTorq) * fGasOverride) + fCoastTorq) * fAirAmount);
	pThis->status.outTorque = fOutTorq;

	bool bHasFuelPressure = (pThis->fuelPressure > 0.0f);
	if (bHasFuelPressure)
	{
		float fRpm = pThis->lastInput.rpm;
		if (fRpm >= (float)pThis->data.minimum)
		{
			float fOverlapGain = pThis->data.overlapGain;
			if (fOverlapGain != 0.0f)
			{
				float fOverlap = sinf((float)pThis->physicsEngine->physicsTime * 0.001f * pThis->data.overlapFreq * fRpm * 0.0003333333333333333f) * 0.5f - 0.5f;
				pThis->status.outTorque = (fOverlap * fabsf(fRpm - pThis->data.overlapIdealRPM) * fOverlapGain) + fOutTorq;
			}
		}
		else if (pThis->isEngineStallEnabled)
		{
			float fStallTorq = fRpm * -0.0099999998f;
			//if (GetAsyncKeyState(8)) fStallTorq = pThis->starterTorque;
			pThis->status.outTorque = fStallTorq;
		}
		else
		{
			pThis->status.outTorque = tmax(15.0f, fOutTorq);
		}
	}

	float fFuelPressure = pThis->fuelPressure;
	if (fFuelPressure < 1.0f)
	{
		pThis->status.outTorque = (pThis->status.outTorque - pThis->lastInput.rpm * -0.01f) * fFuelPressure + pThis->lastInput.rpm * -0.01f;
	}

	for (auto& iter : pThis->torqueGenerators)
	{
		pThis->status.outTorque += iter->getOutputTorque();
	}

	bool bLimiterOn = (pThis->limiterOn != 0);
	pThis->status.isLimiterOn = bLimiterOn;
	pThis->electronicOverride = 1.0f;

	float fMaxPowerDyn = pThis->maxPowerW_Dynamic;
	float fCurPower = pInput->rpm * (float)pThis->status.outTorque * 0.1046999990940094f;
	if (fCurPower > fMaxPowerDyn)
	{
		pThis->maxPowerW_Dynamic = fCurPower;
	}
}

void Engine_stepTurbos(Engine* pThis)
{
	for (auto& tc : pThis->turboControllers)
	{
		if (tc.isWastegate)
			tc.turbo->data.wastegate = tc.controller.eval();
		else
			tc.turbo->data.maxBoost = tc.controller.eval();
	}

	pThis->status.turboBoost = 0.0;
	for (auto& turbo : pThis->turbos)
	{
		float fRpm = pThis->lastInput.rpm > 0.0f ? pThis->lastInput.rpm : 0.0f;
		turbo.step(pThis->lastInput.gasInput, fRpm, 0.003f);
		float fBoost = turbo.getBoost();
		pThis->status.turboBoost += (fBoost * pThis->fuelPressure);
	}
}
