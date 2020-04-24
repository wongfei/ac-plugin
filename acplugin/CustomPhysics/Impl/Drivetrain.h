#pragma once

BEGIN_HOOK_OBJ(Drivetrain)

	#define RVA_Drivetrain_step 2535728
	#define RVA_Drivetrain_stepControllers 2535936
	#define RVA_Drivetrain_step2WD 2528480
	#define RVA_Drivetrain_getInertiaFromWheels 2518048
	#define RVA_Drivetrain_getInertiaFromEngine 2517920
	#define RVA_Drivetrain_reallignSpeeds 2527488
	#define RVA_Drivetrain_accelerateDrivetrainBlock 2516160
	#define RVA_Drivetrain_getEngineRPM 2517888

	void _step(float dt);
	void _stepControllers(float dt);
	void _step2WD(float dt);
	double _getInertiaFromWheels();
	double _getInertiaFromEngine();
	void _reallignSpeeds(float dt);
	void _accelerateDrivetrainBlock(double acc, bool fromEngine);
	float _getEngineRPM();

END_HOOK_OBJ()

void _Drivetrain::_step(float dt)
{
	this->outShaftLF.oldVelocity = this->outShaftLF.velocity;
	this->outShaftRF.oldVelocity = this->outShaftRF.velocity;
	this->outShaftL.oldVelocity = this->outShaftL.velocity;
	this->outShaftR.oldVelocity = this->outShaftR.velocity;

	this->locClutch = powf(this->car->controls.clutch, 1.5f);
	this->currentClutchTorque = 0.0f;

	this->stepControllers(dt);

	switch (this->tractionType)
	{
		case TractionType::RWD:
		case TractionType::FWD:
			this->step2WD(dt);
			break;

		case TractionType::AWD:
			this->step4WD(dt); // TODO
			//NOT_IMPLEMENTED;
			break;

		case TractionType::AWD_NEW:
			this->step4WD_new(dt); // TODO
			//NOT_IMPLEMENTED;
			break;

		default:
			SHOULD_NOT_REACH;
			break;
	}
}

void _Drivetrain::_stepControllers(float dt)
{
	if (this->controllers.awdFrontShare.get())
	{
		this->awdFrontShare = this->controllers.awdFrontShare->eval();
	}

	if (this->controllers.awdCenterLock.get())
	{
		float fScale = ((Car_getSpeedValue(this->car) * 3.6f) - 5.0f) * 0.05f;
		fScale = tclamp(fScale, 0.0f, 1.0f);

		this->awdCenterDiff.preload = ((this->controllers.awdCenterLock->eval() - 20.0f) * fScale) + 20.0f;
		this->awdCenterDiff.power = 0.0f;
	}

	if (this->controllers.singleDiffLock.get())
	{
		this->diffPreLoad = this->controllers.singleDiffLock->eval();
		this->diffPowerRamp = 0.0f;
	}
}

double _Drivetrain::_getInertiaFromWheels()
{
	switch (this->tractionType)
	{
		case TractionType::RWD:
		case TractionType::FWD:
		case TractionType::AWD_NEW: // TODO: why not with AWD?
		{
			double fRatio = this->ratio;

			if (fRatio == 0.0)
				return this->outShaftL.inertia + this->drive.inertia + this->outShaftR.inertia;

			double fRatioSq = fRatio * fRatio;

			if (this->clutchOpenState)
				return fRatioSq * this->clutchInertia + this->drive.inertia + this->outShaftL.inertia + this->outShaftR.inertia;
			else
				return fRatioSq * (this->clutchInertia + this->engine.inertia) + this->drive.inertia + this->outShaftL.inertia + this->outShaftR.inertia;
		}

		case TractionType::AWD:
		{
			double fRatio = this->ratio;

			if (fRatio == 0.0)
				return this->outShaftL.inertia + this->drive.inertia + this->outShaftR.inertia + (this->outShaftLF.inertia + this->outShaftRF.inertia);

			double fRatioSq = fRatio * fRatio;

			if (this->clutchOpenState)
				return fRatioSq * this->clutchInertia + this->drive.inertia + this->outShaftL.inertia + this->outShaftR.inertia + (this->outShaftLF.inertia + this->outShaftRF.inertia);
			else
				return fRatioSq * (this->clutchInertia + this->engine.inertia) + this->drive.inertia + this->outShaftL.inertia + this->outShaftR.inertia + (this->outShaftLF.inertia + this->outShaftRF.inertia);
		}
	}

	SHOULD_NOT_REACH;
	return 0;
}

double _Drivetrain::_getInertiaFromEngine()
{
	double fRatio = this->ratio;
	if (fRatio == 0.0)
		return this->engine.inertia;

	switch (this->tractionType)
	{
		case TractionType::RWD:
		case TractionType::FWD:
		case TractionType::AWD_NEW: // TODO: why not with AWD?
		{
			double fOutInertia = this->outShaftL.inertia + this->drive.inertia + this->outShaftR.inertia;
			return fOutInertia / (fRatio * fRatio) + this->clutchInertia + this->engine.inertia;
		}

		case TractionType::AWD:
		{
			double fOutInertia = this->outShaftL.inertia + this->drive.inertia + this->outShaftR.inertia + this->outShaftLF.inertia + this->outShaftRF.inertia;
			return fOutInertia / (fRatio * fRatio) + this->clutchInertia + this->engine.inertia;
		}
	}

	SHOULD_NOT_REACH;
	return 0;
}

void _Drivetrain::_reallignSpeeds(float dt)
{
	double fRatio = this->ratio;
	if (fRatio != 0.0)
	{
		double fDriveVel = this->drive.velocity;
		if (this->locClutch <= 0.9)
		{
			this->rootVelocity = fDriveVel * fRatio;
		}
		else
		{
			double fRootVelocity = this->rootVelocity;
			this->rootVelocity = fRootVelocity - (1.0 - this->engine.inertia / this->getInertiaFromEngine()) * (fRootVelocity / fRatio - fDriveVel) * fabs(fRatio);
		}

		this->accelerateDrivetrainBlock((this->rootVelocity / fRatio - fDriveVel), false);

		if (!this->clutchOpenState)
			this->engine.velocity = this->rootVelocity;

		DEBUG_ASSERT((fabs(this->drive.velocity - (this->rootVelocity / fRatio)) <= 0.5));
	}
}

void _Drivetrain::_accelerateDrivetrainBlock(double acc, bool fromEngine) // TODO: check, weird logic
{
	this->drive.velocity += acc;

	if (this->tractionType == TractionType::AWD)
	{
		double fShare = 0.5;
		if (fromEngine)
			fShare = this->awdFrontShare;

		double fDeltaVel = fShare * acc * 2.0;
		this->outShaftRF.velocity += fDeltaVel;
		this->outShaftLF.velocity += fDeltaVel;

		fDeltaVel = (1.0 - fShare) * acc * 2.0;
		this->outShaftR.velocity += fDeltaVel;
		this->outShaftL.velocity += fDeltaVel;
	}
	else if (this->diffType == DifferentialType::LSD || this->diffType == DifferentialType::Spool)
	{
		this->outShaftR.velocity = acc + this->outShaftR.velocity;
		this->outShaftL.velocity = acc + this->outShaftL.velocity;
	}
}

float _Drivetrain::_getEngineRPM()
{
	return (float)((this->engine.velocity * 0.15915507) * 60.0);
}
