#pragma once

BEGIN_HOOK_OBJ(BrakeSystem)

	#define RVA_BrakeSystem_step 2680384
	#define RVA_BrakeSystem_stepTemps 2681120

	void _step(float dt);
	void _stepTemps(float dt);

END_HOOK_OBJ()

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
			float fSpeed = Car_getSpeedValue(this->car);
			if (fSpeed * 3.6f > 10.0f)
				bFlag = true;
		}

		if (bFlag)
		{
			float fEbbInstant = (fLoadFront / fLoadAWD) * this->ebbFrontMultiplier;
			this->ebbInstant = tclamp(fEbbInstant, 0.0f, 1.0f);
		}
		else
		{
			this->ebbInstant = this->frontBias;
		}

		fFrontBias = this->ebbInstant;
	}

	fFrontBias = tclamp(fFrontBias, this->limitDown, this->limitUp);

	float fOverride = this->electronicOverride;
	if (fOverride <= this->car->controls.brake)
		fOverride = this->car->controls.brake;

	float fBrakeTorq = (this->brakePower * this->brakePowerMultiplier) * fOverride;

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
			this->car->tyres[3].inputs.brakeTorque = this->car->tyres[3].inputs.brakeTorque + fSteerBrake;
		else
			this->car->tyres[2].inputs.brakeTorque = this->car->tyres[2].inputs.brakeTorque - fSteerBrake;
	}

	if (this->hasBrakeTempsData && this->car->tyres[0].aiMult <= 1.0f)
		this->stepTemps(dt);

	this->electronicOverride = 0.0f;
}

void _BrakeSystem::_stepTemps(float dt)
{
	Tyre* pTyre = this->car->tyres;
	BrakeDisc* pDisc = this->discs;

	for (int i = 0; i < 4; ++i)
	{
		pTyre->inputs.brakeTorque = pDisc->perfCurve.getValue(pDisc->t) * pTyre->inputs.brakeTorque;

		float fSpeed = Car_getSpeedValue(this->car);
		float fCool = (((fSpeed * 3.6f) * pDisc->coolSpeedFactor) + 1.0f) * pDisc->coolTransfer;

		pDisc->t += (((this->car->ksPhysics->ambientTemperature - pDisc->t) * fCool) * dt);
		pDisc->t += (((fabsf(pTyre->status.angularVelocity) * (pTyre->inputs.brakeTorque * pDisc->torqueK)) * 0.001f) * dt);

		pTyre++;
		pDisc++;
	}
}
