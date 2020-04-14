#pragma once

#define RVA_BrakeSystem_step 2680384
#define RVA_BrakeSystem_stepTemps 2681120

void BrakeSystem_step(BrakeSystem* pThis, float dt)
{
	float fFrontBias = pThis->frontBias;
	if (pThis->biasOverride != -1.0f)
		fFrontBias = pThis->biasOverride;

	if (pThis->ebbMode != EBBMode::Internal)
	{
		if (pThis->ebbMode == EBBMode::DynamicController)
			fFrontBias = pThis->ebbController.eval();
	}
	else
	{
		float fLoadFront = pThis->car->tyres[1].status.load + pThis->car->tyres[0].status.load;
		float fLoadAWD = (pThis->car->tyres[3].status.load + pThis->car->tyres[2].status.load) + fLoadFront;
		bool bFlag = false;

		if (fLoadAWD != 0.0f)
		{
			float fSpeed = getSpeedV(pThis->car);
			if (fSpeed * 3.6f > 10.0f)
				bFlag = true;
		}

		if (bFlag)
		{
			float fEbbInstant = (fLoadFront / fLoadAWD) * pThis->ebbFrontMultiplier;
			pThis->ebbInstant = tclamp(fEbbInstant, 0.0f, 1.0f);
		}
		else
		{
			pThis->ebbInstant = pThis->frontBias;
		}

		fFrontBias = pThis->ebbInstant;
	}

	fFrontBias = tclamp<float>(fFrontBias, pThis->limitDown, pThis->limitUp);

	float fOverride = pThis->electronicOverride;
	if (fOverride <= pThis->car->controls.brake)
		fOverride = pThis->car->controls.brake;

	float fBrakeTorq = (pThis->brakePower * pThis->brakePowerMultiplier) * fOverride;

	pThis->car->tyres[0].inputs.brakeTorque = fBrakeTorq * fFrontBias;
	pThis->car->tyres[1].inputs.brakeTorque = fBrakeTorq * fFrontBias;

	float fRearBrakeTorq = ((1.0f - fFrontBias) * fBrakeTorq) - pThis->rearCorrectionTorque;
	if (fRearBrakeTorq < 0.0f)
		fRearBrakeTorq = 0.0f;

	pThis->car->tyres[2].inputs.brakeTorque = fRearBrakeTorq;
	pThis->car->tyres[3].inputs.brakeTorque = fRearBrakeTorq;
	pThis->car->tyres[2].inputs.handBrakeTorque = pThis->car->controls.handBrake * pThis->handBrakeTorque;
	pThis->car->tyres[3].inputs.handBrakeTorque = pThis->car->controls.handBrake * pThis->handBrakeTorque;

	if (pThis->steerBrake.isActive)
	{
		float fSteerBrake = pThis->steerBrake.controller.eval();
		if (fSteerBrake >= 0.0f)
			pThis->car->tyres[3].inputs.brakeTorque = pThis->car->tyres[3].inputs.brakeTorque + fSteerBrake;
		else
			pThis->car->tyres[2].inputs.brakeTorque = pThis->car->tyres[2].inputs.brakeTorque - fSteerBrake;
	}

	if (pThis->hasBrakeTempsData && pThis->car->tyres[0].aiMult <= 1.0f)
		pThis->stepTemps(dt);

	pThis->electronicOverride = 0.0f;
}

void BrakeSystem_stepTemps(BrakeSystem* pThis, float dt)
{
	Tyre* pTyre = pThis->car->tyres;
	BrakeDisc* pDisc = pThis->discs;

	for (int i = 0; i < 4; ++i)
	{
		pTyre->inputs.brakeTorque = pDisc->perfCurve.getValue(pDisc->t) * pTyre->inputs.brakeTorque;

		float fSpeed = getSpeedV(pThis->car);
		float fCool = (((fSpeed * 3.6f) * pDisc->coolSpeedFactor) + 1.0f) * pDisc->coolTransfer;

		pDisc->t += (((pThis->car->ksPhysics->ambientTemperature - pDisc->t) * fCool) * dt);
		pDisc->t += (((fabsf(pTyre->status.angularVelocity) * (pTyre->inputs.brakeTorque * pDisc->torqueK)) * 0.001f) * dt);

		pTyre++;
		pDisc++;
	}
}
