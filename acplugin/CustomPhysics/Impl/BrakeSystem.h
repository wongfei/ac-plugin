#pragma once

BEGIN_HOOK_OBJ(BrakeSystem)

	#define RVA_BrakeSystem_step 2680384
	#define RVA_BrakeSystem_stepTemps 2681120

	static void _hook()
	{
		HOOK_METHOD_RVA(BrakeSystem, step);
		HOOK_METHOD_RVA(BrakeSystem, stepTemps);
	}

	void _step(float dt);
	void _stepTemps(float dt);

END_HOOK_OBJ()

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
