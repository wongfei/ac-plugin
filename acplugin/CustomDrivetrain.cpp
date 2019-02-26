#include "precompiled.h"
#include "CustomHooks.h"
#include "utils.h"

void Drivetrain_step(Drivetrain* pThis, float dt)
{
	pThis->outShaftLF.oldVelocity = pThis->outShaftLF.velocity;
	pThis->outShaftRF.oldVelocity = pThis->outShaftRF.velocity;
	pThis->outShaftL.oldVelocity = pThis->outShaftL.velocity;
	pThis->outShaftR.oldVelocity = pThis->outShaftR.velocity;

	pThis->locClutch = powf(pThis->car->controls.clutch, 1.5f);
	pThis->currentClutchTorque = 0.0f;

	pThis->stepControllers(dt);

	switch (pThis->tractionType)
	{
		case TractionType::RWD:
		case TractionType::FWD:
			pThis->step2WD(dt);
			break;

		case TractionType::AWD:
			pThis->step4WD(dt);
			break;

		case TractionType::AWD_NEW:
			pThis->step4WD_new(dt);
			break;
	}
}

void Drivetrain_stepControllers(Drivetrain* pThis, float dt)
{
	if (pThis->controllers.awdFrontShare.get())
	{
		pThis->awdFrontShare = pThis->controllers.awdFrontShare->eval();
	}

	if (pThis->controllers.awdCenterLock.get())
	{
		float fScale = ((pThis->car->getSpeed().value * 3.5999999f) - 5.0f) * 0.050000001f;
		fScale = tclamp(fScale, 0.0f, 1.0f);

		pThis->awdCenterDiff.preload = ((pThis->controllers.awdCenterLock->eval() - 20.0f) * fScale) + 20.0f;
		pThis->awdCenterDiff.power = 0.0f;
	}

	if (pThis->controllers.singleDiffLock.get())
	{
		pThis->diffPreLoad = pThis->controllers.singleDiffLock->eval();
		pThis->diffPowerRamp = 0.0f;
	}
}

void Drivetrain_step2WD(Drivetrain* pThis, float dt)
{
	auto pCar = pThis->car;

	int iGearRequest = (int)pThis->gearRequest.request - 1;
	if ((!iGearRequest || iGearRequest == 1) && pThis->gearRequest.timeout < pThis->gearRequest.timeAccumulator)
	{
		pThis->currentGear = pThis->gearRequest.requestedGear;
		pThis->gearRequest.request = GearChangeRequest::eNoGearRequest;
	}

	if ((int)pThis->gearRequest.request)
	{
		pThis->gearRequest.timeAccumulator += dt;
	}

	const SGearRatio& curGear = pThis->gears[pThis->currentGear]; // pThis->getCurrentGear()
	pThis->ratio = pThis->finalRatio * curGear.ratio;

	pThis->engine.inertia = pThis->acEngine.inertia;

	for (auto& TorqGen : pThis->wheelTorqueGenerators)
	{
		float fTorq = TorqGen->getOutputTorque() * 0.5f;
		pThis->tyreLeft->status.feedbackTorque += fTorq;
		pThis->tyreRight->status.feedbackTorque += fTorq;
	}

	if (pThis->lastRatio != pThis->ratio)
	{
		pThis->reallignSpeeds(dt);
		pThis->lastRatio = pThis->ratio;
	}

	SACEngineInput input;
	memset(&input, 0, sizeof(input));

	if (pThis->cutOff > 0.0)
	{
		pThis->cutOff -= dt;
	}
	else
	{
		input.gasInput = pCar->controls.gas;
	}

	input.rpm = (float)((pThis->engine.velocity * 0.15915507) * 60.0);
	pThis->acEngine.step(input, dt);

	if (pThis->locClutch < 1.0f)
	{
		pThis->clutchOpenState = true;
	}
	else if (pThis->engine.velocity != 0.0)
	{
		if (fabs(pThis->rootVelocity / pThis->engine.velocity - 1.0) < 0.1)
			pThis->clutchOpenState = false;
		else
			pThis->clutchOpenState = true;
	}
	else
	{
		pThis->clutchOpenState = (0.0 == pThis->rootVelocity ? false : true);
	}

	double fEngineInertia = pThis->engine.inertia;
	double fNewEngineInertia = fEngineInertia;

	if (pThis->ratio != 0.0)
	{
		double fInertiaSum;
		if (pThis->tractionType == TractionType::AWD) // ???
		{
			fInertiaSum = 
				pThis->drive.inertia
				+ pThis->outShaftL.inertia
				+ pThis->outShaftR.inertia
				+ pThis->outShaftLF.inertia
				+ pThis->outShaftRF.inertia;
		}
		else
		{
			fInertiaSum =
				pThis->drive.inertia
				+ pThis->outShaftL.inertia
				+ pThis->outShaftR.inertia;
		}
		fNewEngineInertia = fInertiaSum / (pThis->ratio * pThis->ratio) + pThis->clutchInertia + fEngineInertia;
	}

	double fInertiaFromWheels = pThis->getInertiaFromWheels();
	double fDeltaDriveV = 0.0;
	double fClutchTorq = 0.0;

	if (!pThis->clutchOpenState)
	{
		double fDeltaRootV = (pThis->acEngine.status.outTorque / fNewEngineInertia) * dt;
		pThis->rootVelocity += fDeltaRootV;

		if (pThis->ratio == 0.0)
		{
			fDeltaDriveV = (pThis->tyreRight->status.feedbackTorque + pThis->tyreLeft->status.feedbackTorque) / fInertiaFromWheels * dt;
			pThis->drive.velocity += fDeltaDriveV;
		}
		else
		{
			pThis->accelerateDrivetrainBlock(fDeltaRootV / pThis->ratio, true);

			fDeltaDriveV = (pThis->tyreRight->status.feedbackTorque + pThis->tyreLeft->status.feedbackTorque) / fInertiaFromWheels * dt;
			pThis->rootVelocity += fDeltaDriveV * pThis->ratio;
			pThis->drive.velocity += fDeltaDriveV;
		}
	}
	else
	{
		fClutchTorq = -((pThis->engine.velocity - pThis->rootVelocity) / (fabs(pThis->engine.velocity - pThis->rootVelocity) + 4.0) * (pThis->locClutch * pThis->clutchMaxTorque));
		pThis->currentClutchTorque = (float)fClutchTorq;

		if (pThis->ratio != 0.0)
		{
			pThis->engine.velocity += (fClutchTorq + pThis->acEngine.status.outTorque) / fEngineInertia * dt;

			double fDeltaRootV = (-fClutchTorq / (fNewEngineInertia - fEngineInertia)) * dt;
			pThis->rootVelocity += fDeltaRootV;

			pThis->accelerateDrivetrainBlock(fDeltaRootV / pThis->ratio, true);

			fDeltaDriveV = (pThis->tyreRight->status.feedbackTorque + pThis->tyreLeft->status.feedbackTorque) / fInertiaFromWheels * dt;
			pThis->rootVelocity += fDeltaDriveV * pThis->ratio;
			pThis->drive.velocity += fDeltaDriveV;
		}
		else
		{
			double fNewEngineVelocity = pThis->engine.velocity + pThis->acEngine.status.outTorque / fEngineInertia * dt;
			pThis->engine.velocity = fNewEngineVelocity;
			pThis->rootVelocity = fNewEngineVelocity;

			fDeltaDriveV = (pThis->tyreRight->status.feedbackTorque + pThis->tyreLeft->status.feedbackTorque) / fInertiaFromWheels * dt;
			pThis->drive.velocity += fDeltaDriveV;
		}
	}

	if (pThis->tractionType == TractionType::AWD) // ???
	{
		fDeltaDriveV = fDeltaDriveV * 0.5 * 2.0;
		pThis->outShaftLF.velocity += fDeltaDriveV;
		pThis->outShaftRF.velocity += fDeltaDriveV;
	}

	pThis->outShaftL.velocity += fDeltaDriveV;
	pThis->outShaftR.velocity += fDeltaDriveV;

	if (pThis->diffType == DifferentialType::Spool)
	{
		pThis->outShaftL.velocity = pThis->drive.velocity;
		pThis->outShaftR.velocity = pThis->drive.velocity;
	}
	else if (pThis->diffType == DifferentialType::LSD)
	{
		double fOutClutchTorq;
		if (fClutchTorq != 0.0)
			fOutClutchTorq = -fClutchTorq;
		else
			fOutClutchTorq = pThis->locClutch * pThis->acEngine.status.outTorque;

		double fDiffLoad;
		if (fOutClutchTorq <= 0.0)
			fDiffLoad = fabs(pThis->ratio * pThis->diffCoastRamp * fOutClutchTorq);
		else
			fDiffLoad = fabs(pThis->ratio) * (pThis->diffPowerRamp * fOutClutchTorq);

		double fDiffTotalLoad = fDiffLoad + pThis->diffPreLoad;

		if (fabs(pThis->outShaftL.velocity - pThis->drive.velocity) >= 0.1000000014901161
			|| fabs(pThis->tyreRight->status.feedbackTorque - pThis->tyreLeft->status.feedbackTorque) > fDiffTotalLoad)
		{
			double fUnk1 = -((pThis->outShaftL.velocity - pThis->outShaftR.velocity) / (fabs(pThis->outShaftL.velocity - pThis->outShaftR.velocity) + 0.009999999776482582) * fDiffTotalLoad);
			double fDeltaV1 = dt * (fUnk1 / pThis->outShaftL.inertia * 0.5);
			pThis->outShaftL.velocity += fDeltaV1;
			pThis->outShaftR.velocity -= fDeltaV1;

			double fDeltaV2 = dt * ((pThis->tyreRight->status.feedbackTorque - pThis->tyreLeft->status.feedbackTorque) / pThis->outShaftR.inertia * 0.5);
			pThis->outShaftL.velocity -= fDeltaV2;
			pThis->outShaftR.velocity += fDeltaV2;
		}
		else
		{
			pThis->outShaftL.velocity = pThis->drive.velocity;
			pThis->outShaftR.velocity = pThis->drive.velocity;
		}
	}
	else
	{
		// invalid diff
		return;
	}

	if (pThis->tyreLeft->status.isLocked && pThis->tyreRight->status.isLocked)
	{
		float fTorqL = (pThis->tyreLeft->absOverride * pThis->tyreLeft->inputs.brakeTorque) + pThis->tyreLeft->inputs.handBrakeTorque;
		float fTorqR = (pThis->tyreRight->absOverride * pThis->tyreRight->inputs.brakeTorque) + pThis->tyreRight->inputs.handBrakeTorque;

		bool bFlag = true;
		if (fabs(pThis->ratio * pThis->acEngine.status.outTorque) <= (fTorqL + fTorqR))
		{
			if (pCar->valueCache.speed.value <= 1.0f) // pCar->getSpeed()
				bFlag = false;
		}

		if (bFlag)
		{
			pThis->tyreLeft->status.isLocked = false;
			pThis->tyreRight->status.isLocked = false;
		}
		else if (pThis->clutchOpenState)
		{
			pThis->rootVelocity = 0.0;
			pThis->drive.velocity = 0.0;
			pThis->outShaftL.velocity = 0.0;
			pThis->outShaftR.velocity = 0.0;
		}
	}

	if (pThis->tractionType != TractionType::AWD_NEW) // ???
	{
		if (!pThis->clutchOpenState)
			pThis->engine.velocity = pThis->rootVelocity;

		if (pThis->ratio != 0.0)
		{
			double fComputed = pThis->rootVelocity / pThis->ratio;
			if (fabs(pThis->drive.velocity - fComputed) > 0.5)
				printf("EPIC FAIL: driveVelocity=%f computed=%f\n", pThis->drive.velocity, fComputed);
		}

		pThis->tyreLeft->status.angularVelocity = (float)pThis->outShaftL.velocity;
		pThis->tyreRight->status.angularVelocity = (float)pThis->outShaftR.velocity;

		pThis->tyreLeft->stepRotationMatrix(dt);
		pThis->tyreRight->stepRotationMatrix(dt);
	}

	if (pThis->ratio == 0.0)
	{
		pThis->totalTorque = (float)fabs(pThis->acEngine.status.outTorque * pThis->locClutch);
	}
	else
	{
		pThis->totalTorque = (float)fabs((fabs(pThis->ratio) * (pThis->acEngine.status.outTorque * pThis->locClutch)) - (pThis->tyreLeft->status.feedbackTorque + pThis->tyreRight->status.feedbackTorque));
	}

	double fGearTorque = pThis->locClutch * pThis->acEngine.status.outTorque * curGear.ratio;

	if (pCar->suspensionTypeR == SuspensionType::Axle)
	{
		vec3f v;
		v.x = 0;
		v.y = 0;
		v.z = (float)(fGearTorque * pCar->axleTorqueReaction);
		pCar->body->addLocalTorque(v);

		v.x = 0;
		v.y = 0;
		v.z = -(float)(fGearTorque * pCar->axleTorqueReaction);
		pCar->rigidAxle->addLocalTorque(v);

		if (pCar->torqueModeEx == TorqueModeEX::reactionTorques)
		{
			v.x = -(float)fGearTorque;
			v.y = 0;
			v.z = 0;

			if (pCar->axleTorqueReaction == 0.0f)
				pCar->body->addLocalTorque(v);
			else
				pCar->rigidAxle->addLocalTorque(v);
		}
	}
	else if (pCar->torqueModeEx == TorqueModeEX::reactionTorques && !pThis->tyreLeft->status.isLocked && !pThis->tyreRight->status.isLocked)
	{
		int suspId[2];
		if (pThis->tractionType == TractionType::FWD)
		{
			suspId[0] = 0;
			suspId[1] = 1;
		}
		else
		{
			suspId[0] = 2;
			suspId[1] = 3;
		}

		for (int i = 0; i < 2; ++i)
		{
			auto pSusp = pCar->suspensions[suspId[i]];

			// TODO
			MessageBoxA(nullptr, "_FIXME", "_FIXME", MB_OK);

			/*vPointVelocity = (float *)(*(__int64(__fastcall **)(ISuspension *, SGearRatio *))pSusp->vfptr->gap8)(
				pSusp,
				&pCurGear);

			fX = *vPointVelocity;
			fY = vPointVelocity[1];
			fZ = vPointVelocity[2];

			LODWORD(v70) = LODWORD(fGearTorque) ^ _xmm;
			input.gasInput = (float)(fX * v70) * 0.5;
			input.carSpeed = (float)(fY * v70) * 0.5;
			input.altitude = (float)(fZ * v70) * 0.5;
			pCar4->body->vfptr->addTorque(pCar4->body, (vec3f *)&input);*/
		}
	}
}
