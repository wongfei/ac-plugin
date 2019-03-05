#include "precompiled.h"
#include "GameHooks.h"

void Car_step(Car* pThis, float dt)
{
	if (!pThis->physicsGUID)
	{
		vec3f bodyVelocity = pThis->body->getVelocity();
		float fVelSq = vdot(bodyVelocity, bodyVelocity);
		float ERP;

		if (fVelSq >= 1.0f)
		{
			ERP = 0.30000001f;
			for (auto& pSusp : pThis->suspensions)
			{
				pSusp->setERPCFM(ERP, pSusp->baseCFM);
			}
		}
		else
		{
			ERP = 0.89999998f;
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

	/*if (pThis->blackFlagged && !pThis->isInPits())
	{
		TODO
	}*/

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
		/*if (bControlsLocked)
		{
			pThis->controls.gas = 0;
			pThis->controls.brake = 0;
			pThis->controls.steer = 0;
			pThis->controls.clutch = 0;
		}*/

		/*if (pThis->isGentleStopping)
		{
			pThis->controls.gas = 0.0;
			pThis->controls.brake = 0.2;
		}*/

		/*if (pThis->penaltyTime > 0.0f)
		{
			TODO
		}*/
	}

	float fSteerAngleSig = (pThis->steerLock * pThis->controls.steer) / pThis->steerRatio;

	float fTmp = fSteerAngleSig;
	if (_fdtest(&fTmp) > 0)
		fSteerAngleSig = 0.0f;

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

	float fSpeed = pThis->valueCache.speed.value; // getSpeed()
	vec3f fAngVel = pThis->body->getAngularVelocity();
	float fAngVelMag = vdot(fAngVel, fAngVel);

	if (fSpeed >= 0.5f || fAngVelMag >= 1.0f)
	{
		pThis->sleepingFrames = 0;
	}
	else
	{
		if ((pThis->controls.gas <= 0.0099999998f
			|| pThis->controls.clutch <= 0.0099999998f
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
			pThis->body->stop(0.99000001f);
			pThis->fuelTankBody->stop(0.99000001f);
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
