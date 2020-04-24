#pragma once

BEGIN_HOOK_OBJ(Car)

	#define RVA_Car_step 2579872
	#define RVA_Car_stepComponents 2581712
	#define RVA_Car_stepThermalObjects 2583024
	#define RVA_Car_updateAirPressure 2583264
	#define RVA_Car_updateBodyMass 2583664
	#define RVA_Car_calcBodyMass 2554736

	void _step(float dt);
	void _stepComponents(float dt);
	void _stepThermalObjects(float dt);
	void _updateAirPressure();
	void _updateBodyMass();
	float _calcBodyMass();

END_HOOK_OBJ()

void _Car::_step(float dt)
{
	if (!this->physicsGUID)
	{
		vec3f bodyVelocity = this->body->getVelocity();
		float fVelSq = vdot(bodyVelocity, bodyVelocity);
		float ERP;

		if (fVelSq >= 1.0f)
		{
			ERP = 0.3f;
			for (auto& pSusp : this->suspensions)
			{
				pSusp->setERPCFM(ERP, pSusp->baseCFM);
			}
		}
		else
		{
			ERP = 0.9f;
			for (auto& pSusp : this->suspensions)
			{
				pSusp->setERPCFM(ERP, 0.0000001f);
			}
		}

		this->fuelTankJoint->setERPCFM(ERP, -1.0f);
	}

	bool bControlsLocked = this->isControlsLocked || this->lockControlsTime > this->ksPhysics->physicsTime;

	this->pollControls(dt);

	bool bHeadlightsSwitch = this->controlsProvider->getAction(DriverActions::eHeadlightsSwitch);

	if (this->controlsProvider && bHeadlightsSwitch && !this->lastLigthSwitchState)
		this->lightsOn = this->lightsOn == 0;

	this->lastLigthSwitchState = bHeadlightsSwitch;

	if (this->blackFlagged && !this->isInPits())
	{
		// TODO: check
		const float* M3 = &this->pitPosition.M31;
		vec3f vRot(-M3[0], -M3[1], -M3[2]);
		this->forceRotation(vRot);

		const float* M4 = &this->pitPosition.M41;
		vec3f vPos(M4[0], M4[1], M4[2]);
		this->forcePosition(vPos, true);
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
			this->controls.brake = 0;
			this->controls.steer = 0;
			this->controls.clutch = 0;
		}

		if (this->isGentleStopping)
		{
			this->controls.gas = 0.0;
			this->controls.brake = 0.2;
		}

		if (this->penaltyTime > 0.0f)
		{
			// TODO
		}
	}

	float fSteerAngleSig = (this->steerLock * this->controls.steer) / this->steerRatio;
	if (!isfinite(fSteerAngleSig))
	{
		log_printf(L"INF fSteerAngleSig");
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

	float fSpeed = Car_getSpeedValue(this);
	vec3f fAngVel = this->body->getAngularVelocity();
	float fAngVelMag = vdot(fAngVel, fAngVel);

	if (fSpeed >= 0.5f || fAngVelMag >= 1.0f)
	{
		this->sleepingFrames = 0;
	}
	else
	{
		if ((this->controls.gas <= 0.01f
			|| this->controls.clutch <= 0.01f
			|| this->drivetrain.currentGear == 1)
			&& bAllTyresLoaded)
		{
			++this->sleepingFrames;
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

	vec3f bodyVel = this->body->getVelocity();
	vec3f tmp = vsub(bodyVel, this->lastVelocity);
	tmp = vmul(tmp, (1.0f / dt) * 0.10197838f);
	this->lastVelocity = bodyVel;

	vec3f accG = this->body->worldToLocalNormal(tmp);
	this->accG = accG;

	this->stepThermalObjects(dt);
	this->stepComponents(dt);
	this->updateColliderStatus(dt);

	if (!this->physicsGUID)
		this->stepJumpStart(dt);
}

void _Car::_stepComponents(float dt)
{
	this->brakeSystem.step(dt);
	this->edl.step(dt);

	for (auto& susp : this->suspensions)
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

void _Car::_stepThermalObjects(float dt)
{
	float fRpm = this->drivetrain.getEngineRPM();
	if (fRpm > (this->drivetrain.acEngine.data.minimum * 0.8f))
	{
		float fLimiter = (float)this->drivetrain.acEngine.getLimiterRPM();
		this->water.addHeadSource((((fRpm / fLimiter) * 20.0f) * this->controls.gas) + 85.0f);
	}

	this->water.step(dt, this->ksPhysics->ambientTemperature, this->valueCache.speed);
}

void _Car::_updateAirPressure() // TODO: check this
{
	float fAirDensity = this->ksPhysics->getAirDensity();

	if (this->slipStreamEffectGain > 0.0f)
	{
		vec3f vPos = this->body->getPosition(0.0f); // TODO: not sure about param value -> interpolationT
		float fMinSlip = 1.0f;

		for (auto iterSS = this->ksPhysics->slipStreams.begin(); iterSS != this->ksPhysics->slipStreams.end(); ++iterSS)
		{
			SlipStream* pSS = *iterSS;
			if (pSS != &this->slipStream) // TODO: skip self?
			{
				float fSlip = 1.0f - (pSS->getSlipEffect(vPos) * this->slipStreamEffectGain);
				fSlip = tclamp(fSlip, 0.0f, 1.0f);

				if (fMinSlip > fSlip)
				{
					fMinSlip = fSlip;
				}
			}
		}

		fAirDensity = ((fAirDensity - (fMinSlip * fAirDensity)) * (0.75f / this->slipStreamEffectGain)) + (fMinSlip * fAirDensity);
	}

	this->aeroMap.airDensity = fAirDensity;
}

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
			this->body->setMassExplicitInertia(this->mass, this->explicitInertia.x, this->explicitInertia.y, this->explicitInertia.z);
		}

		float fFuelMass = this->fuelKG * (float)this->fuel;
		this->fuelTankBody->setMassBox(fFuelMass, tmax(0.1f, fFuelMass), 0.5f, 0.5f); // TODO: not sure

		this->lastBodyMassUpdateTime = this->ksPhysics->physicsTime;
	}
}

float _Car::_calcBodyMass()
{
	float fSuspMass = 0;
	for (int i = 0; i < 4; ++i)
		fSuspMass += this->suspensions[i]->getMass();

	return (this->mass - fSuspMass) + this->ballastKG;
}
