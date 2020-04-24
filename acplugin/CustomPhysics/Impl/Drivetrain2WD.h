#pragma once

void _Drivetrain::_step2WD(float dt)
{
	auto pCar = this->car;

	int iGearRequest = (int)this->gearRequest.request - 1;
	if ((!iGearRequest || iGearRequest == 1) && (this->gearRequest.timeout < this->gearRequest.timeAccumulator))
	{
		this->currentGear = this->gearRequest.requestedGear;
		this->gearRequest.request = GearChangeRequest::eNoGearRequest;
	}

	if (this->gearRequest.request != GearChangeRequest::eNoGearRequest)
	{
		this->gearRequest.timeAccumulator += dt;
	}

	const SGearRatio& curGear = this->gears[this->currentGear]; // this->getCurrentGear()
	this->ratio = this->finalRatio * curGear.ratio;
	this->engine.inertia = this->acEngine.inertia;

	for (auto& TorqGen : this->wheelTorqueGenerators)
	{
		float fTorq = TorqGen->getOutputTorque() * 0.5f;
		this->tyreLeft->status.feedbackTorque += fTorq;
		this->tyreRight->status.feedbackTorque += fTorq;
	}

	if (this->lastRatio != this->ratio)
	{
		this->reallignSpeeds(dt);
		this->lastRatio = this->ratio;
	}

	SACEngineInput input;
	memset(&input, 0, sizeof(input));

	if (this->cutOff > 0.0)
	{
		this->cutOff -= dt;
	}
	else
	{
		input.gasInput = pCar->controls.gas;
	}

	input.rpm = (float)((this->engine.velocity * 0.15915507) * 60.0);
	this->acEngine.step(input, dt);

	if (this->locClutch < 1.0f)
	{
		this->clutchOpenState = true;
	}
	else if (this->engine.velocity != 0.0)
	{
		this->clutchOpenState = (fabs(this->rootVelocity / this->engine.velocity - 1.0) >= 0.1);
	}
	else
	{
		this->clutchOpenState = (this->rootVelocity != 0.0);
	}

	double fEngineInertia = this->engine.inertia;
	double fNewEngineInertia = fEngineInertia;

	if (this->ratio != 0.0)
	{
		double fInertiaSum;
		if (this->tractionType == TractionType::AWD) // TODO: AWD in step2WD???
		{
			fInertiaSum =
				this->drive.inertia
				+ this->outShaftL.inertia
				+ this->outShaftR.inertia
				+ this->outShaftLF.inertia
				+ this->outShaftRF.inertia;
		}
		else
		{
			fInertiaSum =
				this->drive.inertia
				+ this->outShaftL.inertia
				+ this->outShaftR.inertia;
		}

		fNewEngineInertia = fInertiaSum / (this->ratio * this->ratio) + this->clutchInertia + fEngineInertia;
	}

	double fInertiaFromWheels = this->getInertiaFromWheels();
	double fDeltaDriveV = 0.0;
	double fClutchTorq = 0.0;

	if (!this->clutchOpenState)
	{
		double fDeltaRootV = (this->acEngine.status.outTorque / fNewEngineInertia) * dt;
		this->rootVelocity += fDeltaRootV;

		if (this->ratio == 0.0)
		{
			fDeltaDriveV = (this->tyreRight->status.feedbackTorque + this->tyreLeft->status.feedbackTorque) / fInertiaFromWheels * dt;
			this->drive.velocity += fDeltaDriveV;
		}
		else
		{
			this->accelerateDrivetrainBlock(fDeltaRootV / this->ratio, true);

			fDeltaDriveV = (this->tyreRight->status.feedbackTorque + this->tyreLeft->status.feedbackTorque) / fInertiaFromWheels * dt;
			this->rootVelocity += fDeltaDriveV * this->ratio;
			this->drive.velocity += fDeltaDriveV;
		}
	}
	else
	{
		fClutchTorq = -((this->engine.velocity - this->rootVelocity) / (fabs(this->engine.velocity - this->rootVelocity) + 4.0) * (this->locClutch * this->clutchMaxTorque));
		this->currentClutchTorque = (float)fClutchTorq;

		if (this->ratio != 0.0)
		{
			this->engine.velocity += (fClutchTorq + this->acEngine.status.outTorque) / fEngineInertia * dt;

			double fDeltaRootV = (-fClutchTorq / (fNewEngineInertia - fEngineInertia)) * dt;
			this->rootVelocity += fDeltaRootV;

			this->accelerateDrivetrainBlock(fDeltaRootV / this->ratio, true);

			fDeltaDriveV = (this->tyreRight->status.feedbackTorque + this->tyreLeft->status.feedbackTorque) / fInertiaFromWheels * dt;
			this->rootVelocity += fDeltaDriveV * this->ratio;
			this->drive.velocity += fDeltaDriveV;
		}
		else
		{
			double fNewEngineVelocity = this->engine.velocity + this->acEngine.status.outTorque / fEngineInertia * dt;
			this->engine.velocity = fNewEngineVelocity;
			this->rootVelocity = fNewEngineVelocity;

			fDeltaDriveV = (this->tyreRight->status.feedbackTorque + this->tyreLeft->status.feedbackTorque) / fInertiaFromWheels * dt;
			this->drive.velocity += fDeltaDriveV;
		}
	}

	if (this->tractionType == TractionType::AWD) // TODO: AWD in step2WD???
	{
		fDeltaDriveV = fDeltaDriveV * 0.5 * 2.0;
		this->outShaftLF.velocity += fDeltaDriveV;
		this->outShaftRF.velocity += fDeltaDriveV;
	}

	this->outShaftL.velocity += fDeltaDriveV;
	this->outShaftR.velocity += fDeltaDriveV;

	if (this->diffType == DifferentialType::Spool)
	{
		this->outShaftL.velocity = this->drive.velocity;
		this->outShaftR.velocity = this->drive.velocity;
	}
	else if (this->diffType == DifferentialType::LSD)
	{
		double fOutClutchTorq, fDiffLoad;

		if (fClutchTorq != 0.0)
			fOutClutchTorq = -fClutchTorq;
		else
			fOutClutchTorq = this->locClutch * this->acEngine.status.outTorque;

		if (fOutClutchTorq <= 0.0)
			fDiffLoad = fabs(this->ratio * this->diffCoastRamp * fOutClutchTorq);
		else
			fDiffLoad = fabs(this->ratio) * (this->diffPowerRamp * fOutClutchTorq);

		double fDiffTotalLoad = fDiffLoad + this->diffPreLoad;

		if (fabs(this->outShaftL.velocity - this->drive.velocity) >= 0.1
			|| fabs(this->tyreRight->status.feedbackTorque - this->tyreLeft->status.feedbackTorque) > fDiffTotalLoad)
		{
			double fUnk1 = -((this->outShaftL.velocity - this->outShaftR.velocity) / (fabs(this->outShaftL.velocity - this->outShaftR.velocity) + 0.01) * fDiffTotalLoad);
			double fDeltaV1 = dt * (fUnk1 / this->outShaftL.inertia * 0.5);
			this->outShaftL.velocity += fDeltaV1;
			this->outShaftR.velocity -= fDeltaV1;

			double fDeltaV2 = dt * ((this->tyreRight->status.feedbackTorque - this->tyreLeft->status.feedbackTorque) / this->outShaftR.inertia * 0.5);
			this->outShaftL.velocity -= fDeltaV2;
			this->outShaftR.velocity += fDeltaV2;
		}
		else
		{
			this->outShaftL.velocity = this->drive.velocity;
			this->outShaftR.velocity = this->drive.velocity;
		}
	}
	else
	{
		SHOULD_NOT_REACH;
		return;
	}

	if (this->tyreLeft->status.isLocked && this->tyreRight->status.isLocked)
	{
		float fTorqL = (this->tyreLeft->absOverride * this->tyreLeft->inputs.brakeTorque) + this->tyreLeft->inputs.handBrakeTorque;
		float fTorqR = (this->tyreRight->absOverride * this->tyreRight->inputs.brakeTorque) + this->tyreRight->inputs.handBrakeTorque;
		bool bFlag = true;

		if (fabs(this->ratio * this->acEngine.status.outTorque) <= (fTorqL + fTorqR))
		{
			if (Car_getSpeedValue(pCar) <= 1.0f)
				bFlag = false;
		}

		if (bFlag)
		{
			this->tyreLeft->status.isLocked = false;
			this->tyreRight->status.isLocked = false;
		}
		else if (this->clutchOpenState)
		{
			this->rootVelocity = 0.0;
			this->drive.velocity = 0.0;
			this->outShaftL.velocity = 0.0;
			this->outShaftR.velocity = 0.0;
		}
	}

	if (this->tractionType != TractionType::AWD_NEW) // TODO: AWD_NEW in step2WD???
	{
		if (!this->clutchOpenState)
			this->engine.velocity = this->rootVelocity;

		if (this->ratio != 0.0)
		{
			DEBUG_ASSERT((fabs(this->drive.velocity - (this->rootVelocity / this->ratio)) <= 0.5));
		}

		this->tyreLeft->status.angularVelocity = (float)this->outShaftL.velocity;
		this->tyreRight->status.angularVelocity = (float)this->outShaftR.velocity;

		this->tyreLeft->stepRotationMatrix(dt);
		this->tyreRight->stepRotationMatrix(dt);
	}

	if (this->ratio == 0.0)
	{
		this->totalTorque = (float)fabs(this->acEngine.status.outTorque * this->locClutch);
	}
	else
	{
		this->totalTorque = (float)fabs((fabs(this->ratio) * (this->acEngine.status.outTorque * this->locClutch)) - (this->tyreLeft->status.feedbackTorque + this->tyreRight->status.feedbackTorque));
	}

	float fGearTorque = (float)(this->locClutch * this->acEngine.status.outTorque * curGear.ratio);

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
	else if (pCar->torqueModeEx == TorqueModeEX::reactionTorques && !this->tyreLeft->status.isLocked && !this->tyreRight->status.isLocked)
	{
		int suspId[2];
		if (this->tractionType == TractionType::FWD)
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
