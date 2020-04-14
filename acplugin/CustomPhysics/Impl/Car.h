#pragma once

#define RVA_Car_step 2579872
#define RVA_Car_stepComponents 2581712
#define RVA_Car_pollControls 2575984
#define RVA_Car_stepThermalObjects 2583024
#define RVA_Car_updateAirPressure 2583264
#define RVA_Car_updateBodyMass 2583664
#define RVA_Car_calcBodyMass 2554736

void Car_step(Car* pThis, float dt)
{
	if (!pThis)
	{
		SHOULD_NOT_REACH;
		return;
	}

	if (!pThis->physicsGUID)
	{
		vec3f bodyVelocity = pThis->body->getVelocity();
		float fVelSq = vdot(bodyVelocity, bodyVelocity);
		float ERP;

		if (fVelSq >= 1.0f)
		{
			ERP = 0.3f;
			for (auto& pSusp : pThis->suspensions)
			{
				pSusp->setERPCFM(ERP, pSusp->baseCFM);
			}
		}
		else
		{
			ERP = 0.9f;
			for (auto& pSusp : pThis->suspensions)
			{
				pSusp->setERPCFM(ERP, 0.0000001f);
			}
		}

		pThis->fuelTankJoint->setERPCFM(ERP, -1.0f);
	}

	bool bControlsLocked = pThis->isControlsLocked || pThis->lockControlsTime > pThis->ksPhysics->physicsTime;

	pThis->pollControls(dt);

	bool bHeadlightsSwitch = pThis->controlsProvider->getAction(DriverActions::eHeadlightsSwitch);

	if (pThis->controlsProvider && bHeadlightsSwitch && !pThis->lastLigthSwitchState)
		pThis->lightsOn = pThis->lightsOn == 0;

	pThis->lastLigthSwitchState = bHeadlightsSwitch;

	if (pThis->blackFlagged && !pThis->isInPits())
	{
		// TODO
	}

	pThis->updateAirPressure();

	float fRpmAbs = fabsf(pThis->drivetrain.getEngineRPM());
	float fTurboBoost = tmax(0.0f, pThis->drivetrain.acEngine.status.turboBoost);
	double fNewFuel = pThis->fuel - (fRpmAbs * dt * pThis->drivetrain.acEngine.gasUsage) * (fTurboBoost + 1.0) * pThis->fuelConsumptionK * 0.001 * pThis->ksPhysics->fuelConsumptionRate;
	pThis->fuel = fNewFuel;

	if (fNewFuel > 0.0f)
	{
		pThis->drivetrain.acEngine.fuelPressure = 1.0f;
	}
	else
	{
		pThis->fuel = 0.0f;
		pThis->drivetrain.acEngine.fuelPressure = 0.0f;
	}

	pThis->updateBodyMass();

	if (pThis->controlsProvider)
	{
		if (bControlsLocked)
		{
			pThis->controls.gas = 0;
			pThis->controls.brake = 0;
			pThis->controls.steer = 0;
			pThis->controls.clutch = 0;
		}

		if (pThis->isGentleStopping)
		{
			pThis->controls.gas = 0.0;
			pThis->controls.brake = 0.2;
		}

		if (pThis->penaltyTime > 0.0f)
		{
			// TODO
		}
	}

	float fSteerAngleSig = (pThis->steerLock * pThis->controls.steer) / pThis->steerRatio;
	if (!isfinite(fSteerAngleSig))
	{
		DEBUG_BREAK;
		fSteerAngleSig = 0.0f;
	}

	pThis->finalSteerAngleSignal = fSteerAngleSig;

	bool bAllTyresLoaded = true;
	for (int i = 0; i < 4; ++i)
	{
		if (pThis->tyres[i].status.load <= 0.0f)
		{
			bAllTyresLoaded = false;
			break;
		}
	}

	pThis->autoClutch.step(dt);

	float fSpeed = getSpeedV(pThis);
	vec3f fAngVel = pThis->body->getAngularVelocity();
	float fAngVelMag = vdot(fAngVel, fAngVel);

	if (fSpeed >= 0.5f || fAngVelMag >= 1.0f)
	{
		pThis->sleepingFrames = 0;
	}
	else
	{
		if ((pThis->controls.gas <= 0.01f
			|| pThis->controls.clutch <= 0.01f
			|| pThis->drivetrain.currentGear == 1)
			&& bAllTyresLoaded)
		{
			++pThis->sleepingFrames;
		}
		else
		{
			pThis->sleepingFrames = 0;
		}

		if (pThis->sleepingFrames > pThis->framesToSleep)
		{
			pThis->body->stop(0.99f);
			pThis->fuelTankBody->stop(0.99f);
		}
	}

	vec3f bodyVel = pThis->body->getVelocity();
	vec3f tmp = vsub(bodyVel, pThis->lastVelocity);
	tmp = vmul(tmp, (1.0f / dt) * 0.10197838f);
	pThis->lastVelocity = bodyVel;

	vec3f accG = pThis->body->worldToLocalNormal(tmp);
	pThis->accG = accG;

	pThis->stepThermalObjects(dt);
	pThis->stepComponents(dt);
	pThis->updateColliderStatus(dt);

	if (!pThis->physicsGUID)
		pThis->stepJumpStart(dt);
}

void Car_stepComponents(Car* pThis, float dt)
{
	pThis->brakeSystem.step(dt);
	pThis->edl.step(dt);

	for (auto& susp : pThis->suspensions)
	{
		susp->step(dt);
	}

	for (int i = 0; i < 4; ++i)
	{
		pThis->tyres[i].step(dt);
	}

	for (int i = 0; i < 2; ++i)
	{
		if (pThis->heaveSprings[i].k != 0.0f)
			pThis->heaveSprings[i].step(dt);
	}

	pThis->drs.step(dt);
	pThis->aeroMap.step(dt);

	if (pThis->kers.present)
		pThis->kers.step(dt);

	if (pThis->ers.present)
		pThis->ers.step(dt);

	pThis->steeringSystem.step(dt);
	pThis->autoBlip.step(dt);
	pThis->autoShift.step(dt);
	pThis->gearChanger.step(dt);
	pThis->drivetrain.step(dt);

	for (int i = 0; i < 2; ++i)
	{
		pThis->antirollBars[i].step(dt);
	}

	pThis->abs.step(dt);
	pThis->tractionControl.step(dt);
	pThis->speedLimiter.step(dt);
	pThis->colliderManager.step(dt);
	pThis->setupManager.step(dt);

	if (!pThis->physicsGUID)
	{
		pThis->telemetry.step(dt);
		pThis->driftMode.step(dt);
		pThis->performanceMeter.step(dt);
		pThis->lapInvalidator.step(dt);
		pThis->penaltyManager.step(dt);
	}

	pThis->splineLocator.step(dt);
	pThis->stabilityControl.step(dt);
	pThis->transponder.step(dt);
	pThis->fuelLapEvaluator.step(dt);
}

void Car_stepThermalObjects(Car* pThis, float dt)
{
	float fRpm = pThis->drivetrain.getEngineRPM();
	if (fRpm > (pThis->drivetrain.acEngine.data.minimum * 0.8f))
	{
		float fLimiter = (float)pThis->drivetrain.acEngine.getLimiterRPM();
		pThis->water.addHeadSource((((fRpm / fLimiter) * 20.0f) * pThis->controls.gas) + 85.0f);
	}

	pThis->water.step(dt, pThis->ksPhysics->ambientTemperature, pThis->valueCache.speed);
}

void Car_updateAirPressure(Car* pThis) // TODO: check this
{
	float fAirDensity = pThis->ksPhysics->getAirDensity();

	if (pThis->slipStreamEffectGain > 0.0f)
	{
		vec3f vPos = pThis->body->getPosition(0.0f); // TODO: not sure about param value -> interpolationT
		float fMinSlip = 1.0f;

		for (auto iterSS = pThis->ksPhysics->slipStreams.begin(); iterSS != pThis->ksPhysics->slipStreams.end(); ++iterSS)
		{
			SlipStream* pSS = *iterSS;
			if (pSS != &pThis->slipStream) // TODO: skip self?
			{
				float fSlip = 1.0f - (pSS->getSlipEffect(vPos) * pThis->slipStreamEffectGain);
				fSlip = tclamp<float>(fSlip, 0.0f, 1.0f);

				if (fMinSlip > fSlip)
				{
					fMinSlip = fSlip;
				}
			}
		}

		fAirDensity = ((fAirDensity - (fMinSlip * fAirDensity)) * (0.75f / pThis->slipStreamEffectGain)) + (fMinSlip * fAirDensity);
	}

	pThis->aeroMap.airDensity = fAirDensity;
}

void Car_updateBodyMass(Car* pThis)
{
	if (pThis->ksPhysics->physicsTime - pThis->lastBodyMassUpdateTime > 1000.0)
	{
		if (pThis->bodyInertia.x != 0.0f || pThis->bodyInertia.y != 0.0f || pThis->bodyInertia.z != 0.0f)
		{
			float fBodyMass = pThis->calcBodyMass();
			pThis->body->setMassBox(fBodyMass, pThis->bodyInertia.x, pThis->bodyInertia.y, pThis->bodyInertia.z);
		}
		else
		{
			pThis->body->setMassExplicitInertia(pThis->mass, pThis->explicitInertia.x, pThis->explicitInertia.y, pThis->explicitInertia.z);
		}

		float fFuelMass = pThis->fuelKG * (float)pThis->fuel;
		pThis->fuelTankBody->setMassBox(fFuelMass, tmax<float>(0.1f, fFuelMass), 0.5f, 0.5f); // TODO: not sure

		pThis->lastBodyMassUpdateTime = pThis->ksPhysics->physicsTime;
	}
}

float Car_calcBodyMass(Car* pThis)
{
	float fSuspMass = 0;
	for (int i = 0; i < 4; ++i)
		fSuspMass += pThis->suspensions[i]->getMass();

	return (pThis->mass - fSuspMass) + pThis->ballastKG;
}
