#pragma once

void Drivetrain_step2WD(Drivetrain* pThis, float dt)
{
	auto pCar = pThis->car;

	int iGearRequest = (int)pThis->gearRequest.request - 1;
	if ((!iGearRequest || iGearRequest == 1) && (pThis->gearRequest.timeout < pThis->gearRequest.timeAccumulator))
	{
		pThis->currentGear = pThis->gearRequest.requestedGear;
		pThis->gearRequest.request = GearChangeRequest::eNoGearRequest;
	}

	if (pThis->gearRequest.request != GearChangeRequest::eNoGearRequest)
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
		pThis->clutchOpenState = (fabs(pThis->rootVelocity / pThis->engine.velocity - 1.0) >= 0.1);
	}
	else
	{
		pThis->clutchOpenState = (pThis->rootVelocity != 0.0);
	}

	double fEngineInertia = pThis->engine.inertia;
	double fNewEngineInertia = fEngineInertia;

	if (pThis->ratio != 0.0)
	{
		double fInertiaSum;
		if (pThis->tractionType == TractionType::AWD) // TODO: AWD in step2WD???
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

	if (pThis->tractionType == TractionType::AWD) // TODO: AWD in step2WD???
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
		double fOutClutchTorq, fDiffLoad;

		if (fClutchTorq != 0.0)
			fOutClutchTorq = -fClutchTorq;
		else
			fOutClutchTorq = pThis->locClutch * pThis->acEngine.status.outTorque;

		if (fOutClutchTorq <= 0.0)
			fDiffLoad = fabs(pThis->ratio * pThis->diffCoastRamp * fOutClutchTorq);
		else
			fDiffLoad = fabs(pThis->ratio) * (pThis->diffPowerRamp * fOutClutchTorq);

		double fDiffTotalLoad = fDiffLoad + pThis->diffPreLoad;

		if (fabs(pThis->outShaftL.velocity - pThis->drive.velocity) >= 0.1
			|| fabs(pThis->tyreRight->status.feedbackTorque - pThis->tyreLeft->status.feedbackTorque) > fDiffTotalLoad)
		{
			double fUnk1 = -((pThis->outShaftL.velocity - pThis->outShaftR.velocity) / (fabs(pThis->outShaftL.velocity - pThis->outShaftR.velocity) + 0.01) * fDiffTotalLoad);
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
		SHOULD_NOT_REACH;
		return;
	}

	if (pThis->tyreLeft->status.isLocked && pThis->tyreRight->status.isLocked)
	{
		float fTorqL = (pThis->tyreLeft->absOverride * pThis->tyreLeft->inputs.brakeTorque) + pThis->tyreLeft->inputs.handBrakeTorque;
		float fTorqR = (pThis->tyreRight->absOverride * pThis->tyreRight->inputs.brakeTorque) + pThis->tyreRight->inputs.handBrakeTorque;
		bool bFlag = true;

		if (fabs(pThis->ratio * pThis->acEngine.status.outTorque) <= (fTorqL + fTorqR))
		{
			if (Car_getSpeedValue(pCar) <= 1.0f)
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

	if (pThis->tractionType != TractionType::AWD_NEW) // TODO: AWD_NEW in step2WD???
	{
		if (!pThis->clutchOpenState)
			pThis->engine.velocity = pThis->rootVelocity;

		if (pThis->ratio != 0.0)
		{
			DEBUG_ASSERT((fabs(pThis->drive.velocity - (pThis->rootVelocity / pThis->ratio)) <= 0.5));
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

	float fGearTorque = (float)(pThis->locClutch * pThis->acEngine.status.outTorque * curGear.ratio);

	if (pCar->suspensionTypeR == SuspensionType::Axle)
	{
		float fAxleTorq = fGearTorque * pCar->axleTorqueReaction;

		vec3f vTorq = vec3f(0, 0, fAxleTorq);
		pCar->body->addLocalTorque(vTorq);

		vTorq = vec3f(0, 0, -fAxleTorq);
		pCar->rigidAxle->addLocalTorque(vTorq);

		if (pCar->torqueModeEx == TorqueModeEX::reactionTorques)
		{
			vTorq = vec3f(-fGearTorque, 0, 0);

			if (pCar->axleTorqueReaction == 0.0f)
				pCar->body->addLocalTorque(vTorq);
			else
				pCar->rigidAxle->addLocalTorque(vTorq);
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

		float fNegGearTorque = -fGearTorque;

		for (int i = 0; i < 2; ++i)
		{
			ISuspension* pSusp = pCar->suspensions[suspId[i]];
			mat44f mxHubWorld = pSusp->getHubWorldMatrix();

			vec3f vTorq(
				mxHubWorld.M11 * fNegGearTorque * 0.5f,
				mxHubWorld.M12 * fNegGearTorque * 0.5f,
				mxHubWorld.M13 * fNegGearTorque * 0.5f);

			pCar->body->addTorque(vTorq);
		}
	}
}
