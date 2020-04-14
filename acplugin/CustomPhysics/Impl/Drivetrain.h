#pragma once

#define RVA_Drivetrain_step 2535728
#define RVA_Drivetrain_stepControllers 2535936
#define RVA_Drivetrain_step2WD 2528480
#define RVA_Drivetrain_getInertiaFromWheels 2518048
#define RVA_Drivetrain_getInertiaFromEngine 2517920
#define RVA_Drivetrain_reallignSpeeds 2527488
#define RVA_Drivetrain_accelerateDrivetrainBlock 2516160
#define RVA_Drivetrain_getEngineRPM 2517888

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
			pThis->step4WD(dt); // TODO
			//NOT_IMPLEMENTED;
			break;

		case TractionType::AWD_NEW:
			pThis->step4WD_new(dt); // TODO
			//NOT_IMPLEMENTED;
			break;

		default:
			SHOULD_NOT_REACH;
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
		float fScale = ((getSpeedV(pThis->car) * 3.6f) - 5.0f) * 0.05f;
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

double Drivetrain_getInertiaFromWheels(Drivetrain* pThis)
{
	switch (pThis->tractionType)
	{
		case TractionType::RWD:
		case TractionType::FWD:
		case TractionType::AWD_NEW: // TODO: why not with AWD?
		{
			double fRatio = pThis->ratio;

			if (fRatio == 0.0)
				return pThis->outShaftL.inertia + pThis->drive.inertia + pThis->outShaftR.inertia;

			double fRatioSq = fRatio * fRatio;

			if (pThis->clutchOpenState)
				return fRatioSq * pThis->clutchInertia + pThis->drive.inertia + pThis->outShaftL.inertia + pThis->outShaftR.inertia;
			else
				return fRatioSq * (pThis->clutchInertia + pThis->engine.inertia) + pThis->drive.inertia + pThis->outShaftL.inertia + pThis->outShaftR.inertia;
		}

		case TractionType::AWD:
		{
			double fRatio = pThis->ratio;

			if (fRatio == 0.0)
				return pThis->outShaftL.inertia + pThis->drive.inertia + pThis->outShaftR.inertia + (pThis->outShaftLF.inertia + pThis->outShaftRF.inertia);

			double fRatioSq = fRatio * fRatio;

			if (pThis->clutchOpenState)
				return fRatioSq * pThis->clutchInertia + pThis->drive.inertia + pThis->outShaftL.inertia + pThis->outShaftR.inertia + (pThis->outShaftLF.inertia + pThis->outShaftRF.inertia);
			else
				return fRatioSq * (pThis->clutchInertia + pThis->engine.inertia) + pThis->drive.inertia + pThis->outShaftL.inertia + pThis->outShaftR.inertia + (pThis->outShaftLF.inertia + pThis->outShaftRF.inertia);
		}
	}

	SHOULD_NOT_REACH;
	return 0;
}

double Drivetrain_getInertiaFromEngine(Drivetrain* pThis)
{
	double fRatio = pThis->ratio;
	if (fRatio == 0.0)
		return pThis->engine.inertia;

	switch (pThis->tractionType)
	{
		case TractionType::RWD:
		case TractionType::FWD:
		case TractionType::AWD_NEW: // TODO: why not with AWD?
		{
			double fOutInertia = pThis->outShaftL.inertia + pThis->drive.inertia + pThis->outShaftR.inertia;
			return fOutInertia / (fRatio * fRatio) + pThis->clutchInertia + pThis->engine.inertia;
		}

		case TractionType::AWD:
		{
			double fOutInertia = pThis->outShaftL.inertia + pThis->drive.inertia + pThis->outShaftR.inertia + pThis->outShaftLF.inertia + pThis->outShaftRF.inertia;
			return fOutInertia / (fRatio * fRatio) + pThis->clutchInertia + pThis->engine.inertia;
		}
	}

	SHOULD_NOT_REACH;
	return 0;
}

void Drivetrain_reallignSpeeds(Drivetrain* pThis, float dt)
{
	double fRatio = pThis->ratio;
	if (fRatio != 0.0)
	{
		double fDriveVel = pThis->drive.velocity;
		if (pThis->locClutch <= 0.9)
		{
			pThis->rootVelocity = fDriveVel * fRatio;
		}
		else
		{
			double fRootVelocity = pThis->rootVelocity;
			pThis->rootVelocity = fRootVelocity - (1.0 - pThis->engine.inertia / pThis->getInertiaFromEngine()) * (fRootVelocity / fRatio - fDriveVel) * fabs(fRatio);
		}

		pThis->accelerateDrivetrainBlock((pThis->rootVelocity / fRatio - fDriveVel), false);

		if (!pThis->clutchOpenState)
			pThis->engine.velocity = pThis->rootVelocity;

		DEBUG_ASSERT((fabs(pThis->drive.velocity - (pThis->rootVelocity / fRatio)) <= 0.5));
	}
}

void Drivetrain_accelerateDrivetrainBlock(Drivetrain* pThis, double acc, bool fromEngine) // TODO: check, weird logic
{
	pThis->drive.velocity += acc;

	if (pThis->tractionType == TractionType::AWD)
	{
		double fShare = 0.5;
		if (fromEngine)
			fShare = pThis->awdFrontShare;

		double fDeltaVel = fShare * acc * 2.0;
		pThis->outShaftRF.velocity += fDeltaVel;
		pThis->outShaftLF.velocity += fDeltaVel;

		fDeltaVel = (1.0 - fShare) * acc * 2.0;
		pThis->outShaftR.velocity += fDeltaVel;
		pThis->outShaftL.velocity += fDeltaVel;
	}
	else if (pThis->diffType == DifferentialType::LSD || pThis->diffType == DifferentialType::Spool)
	{
		pThis->outShaftR.velocity = acc + pThis->outShaftR.velocity;
		pThis->outShaftL.velocity = acc + pThis->outShaftL.velocity;
	}
}

float Drivetrain_getEngineRPM(Drivetrain* pThis)
{
	return (float)((pThis->engine.velocity * 0.15915507) * 60.0);
}
