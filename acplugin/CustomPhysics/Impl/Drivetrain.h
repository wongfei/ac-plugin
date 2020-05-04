#pragma once

BEGIN_HOOK_OBJ(Drivetrain)

	#define RVA_Drivetrain_setCurrentGear 2527968
	#define RVA_Drivetrain_gearUp 2517488
	#define RVA_Drivetrain_gearDown 2516576
	#define RVA_Drivetrain_step 2535728
	#define RVA_Drivetrain_stepControllers 2535936
	#define RVA_Drivetrain_step2WD 2528480
	#define RVA_Drivetrain_reallignSpeeds 2527488
	#define RVA_Drivetrain_accelerateDrivetrainBlock 2516160
	#define RVA_Drivetrain_getInertiaFromWheels 2518048
	#define RVA_Drivetrain_getInertiaFromEngine 2517920
	#define RVA_Drivetrain_getEngineRPM 2517888

	static void _hook()
	{
		HOOK_METHOD_RVA(Drivetrain, setCurrentGear);
		HOOK_METHOD_RVA(Drivetrain, gearUp);
		HOOK_METHOD_RVA(Drivetrain, gearDown);
		HOOK_METHOD_RVA(Drivetrain, step);
		HOOK_METHOD_RVA(Drivetrain, stepControllers);
		HOOK_METHOD_RVA(Drivetrain, step2WD);
		HOOK_METHOD_RVA(Drivetrain, reallignSpeeds);
		HOOK_METHOD_RVA(Drivetrain, accelerateDrivetrainBlock);
		HOOK_METHOD_RVA(Drivetrain, getInertiaFromWheels);
		HOOK_METHOD_RVA(Drivetrain, getInertiaFromEngine);
		HOOK_METHOD_RVA(Drivetrain, getEngineRPM);
	}

	void _setCurrentGear(int index, bool force);
	bool _gearUp();
	bool _gearDown();
	void _step(float dt);
	void _stepControllers(float dt);
	void _step2WD(float dt);
	void _reallignSpeeds(float dt);
	void _accelerateDrivetrainBlock(double acc, bool fromEngine);
	double _getInertiaFromWheels();
	double _getInertiaFromEngine();
	float _getEngineRPM();

END_HOOK_OBJ()

void _Drivetrain::_setCurrentGear(int index, bool force)
{
	if (index != 1 && this->isGearboxLocked())
	{
		this->currentGear = 1;
	}
	else
	{
		this->isGearGrinding = false;

		if (index >= 0 && index < (int)this->gears.size() && index != this->currentGear)
		{
			float v8 = (float)fabs(
				this->engine.velocity - this->gears[index].ratio * this->drive.velocity * this->finalRatio);

			float v9 = ((this->car->controls.gas * this->locClutch * v8 - this->locClutch * v8) 
				* (float)this->controlsWindowGain + this->locClutch * v8)
				* (1.0f / (2.0f * M_PI)) * 60.0f;

			if (index == 1 || force || v9 < (float)this->validShiftRPMWindow)
			{
				this->currentGear = index;
			}
			else
			{
				this->isGearGrinding = true;

				if (this->validShiftRPMWindow > 0.0)
				{
					float fRate = this->car->ksPhysics->mechanicalDamageRate;
					if (fRate > 0.0f)
						this->validShiftRPMWindow -= this->damageRpmWindow * 0.003f * fRate;
				}
			}
		}
	}
}

bool _Drivetrain::_gearUp()
{
	if (this->isGearboxLocked())
		return false;

	int iCurGear = this->currentGear;
	int iReqGear = iCurGear + 1;

	if (iReqGear >= (int)this->gears.size())
		return false;

	if (this->gearRequest.request != GearChangeRequest::eNoGearRequest)
		return false;

	this->gearRequest.request = GearChangeRequest::eChangeUp;
	this->gearRequest.timeAccumulator = 0.0;
	this->gearRequest.timeout = this->gearUpTime;
	this->gearRequest.requestedGear = iReqGear;

	OnGearRequestEvent e;
	e.request = this->gearRequest.request;
	e.nextGear = iReqGear;

	for (auto& h : this->evOnGearRequest.handlers)
	{
		if (h.second)
			(h.second)(e);
	}

	if (this->autoCutOffTime != 0.0f)
		this->cutOff = this->autoCutOffTime;

	this->currentGear = 1;
	return true;
}

bool _Drivetrain::_gearDown()
{
	if (this->isGearboxLocked())
		return false;

	if (this->downshiftProtection.isActive) // TODO: a real men doesnt need this shit
	{
	}

	int iCurGear = this->currentGear;
	int iReqGear = iCurGear - 1;

	if (iCurGear <= 0)
		return false;

	if (this->gearRequest.request != GearChangeRequest::eNoGearRequest)
		return false;

	this->gearRequest.request = GearChangeRequest::eChangeDown;
	this->gearRequest.timeAccumulator = 0.0;
	this->gearRequest.timeout = this->gearDnTime;
	this->gearRequest.requestedGear = iReqGear;

	OnGearRequestEvent e;
	e.request = this->gearRequest.request;
	e.nextGear = iReqGear;

	for (auto& h : this->evOnGearRequest.handlers)
	{
		if (h.second)
			(h.second)(e);
	}

	this->currentGear = 1;
	return true;
}

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
			this->step4WD(dt); // TODO: implement
			break;

		case TractionType::AWD_NEW:
			this->step4WD_new(dt); // TODO: implement
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
		float fCenterLock = this->controllers.awdCenterLock->eval();
		float fScale = tclamp(((getSpeedKMH(this->car) - 5.0f) * 0.05f), 0.0f, 1.0f);

		this->awdCenterDiff.preload = ((fCenterLock - 20.0f) * fScale) + 20.0f;
		this->awdCenterDiff.power = 0.0f;
	}

	if (this->controllers.singleDiffLock.get())
	{
		this->diffPreLoad = this->controllers.singleDiffLock->eval();
		this->diffPowerRamp = 0.0f;
	}
}

void _Drivetrain::_reallignSpeeds(float dt)
{
	double fRatio = this->ratio;
	if (fRatio != 0.0)
	{
		double fDriveVel = this->drive.velocity;
		if (this->locClutch <= 0.8999999)
		{
			this->rootVelocity = fDriveVel * fRatio;
		}
		else
		{
			this->rootVelocity -= 
				(1.0 - this->engine.inertia / this->getInertiaFromEngine()) 
				* (this->rootVelocity / fRatio - fDriveVel) 
				* fabs(fRatio);
		}

		this->accelerateDrivetrainBlock((this->rootVelocity / fRatio - fDriveVel), false);

		if (!this->clutchOpenState)
			this->engine.velocity = this->rootVelocity;

		DEBUG_ASSERT((fabs(this->drive.velocity - (this->rootVelocity / fRatio)) <= 0.5));
	}
}

void _Drivetrain::_accelerateDrivetrainBlock(double acc, bool fromEngine) // TODO: check
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
		this->outShaftR.velocity += acc;
		this->outShaftL.velocity += acc;
	}
}

double _Drivetrain::_getInertiaFromWheels()
{
	double fRatio = this->ratio;
	double fRatioSq = fRatio * fRatio;
	double fRWD = this->drive.inertia + (this->outShaftL.inertia + this->outShaftR.inertia);

	switch (this->tractionType)
	{
		case TractionType::RWD:
		case TractionType::FWD:
		case TractionType::AWD_NEW: // TODO: why not with AWD?
		{
			if (fRatio == 0.0)
				return fRWD;

			if (this->clutchOpenState)
				return fRWD + (this->clutchInertia * fRatioSq);
			else
				return fRWD + ((this->clutchInertia + this->engine.inertia) * fRatioSq);
		}

		case TractionType::AWD:
		{
			double fAWD = fRWD + (this->outShaftLF.inertia + this->outShaftRF.inertia);

			if (fRatio == 0.0)
				return fAWD;

			if (this->clutchOpenState)
				return fAWD + (this->clutchInertia * fRatioSq);
			else
				return fAWD + ((this->clutchInertia + this->engine.inertia) * fRatioSq);
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

	double fRWD = this->drive.inertia + this->outShaftL.inertia + this->outShaftR.inertia;

	switch (this->tractionType)
	{
		case TractionType::RWD:
		case TractionType::FWD:
		case TractionType::AWD_NEW: // TODO: why not with AWD?
		{
			return fRWD / (fRatio * fRatio) + this->clutchInertia + this->engine.inertia;
		}

		case TractionType::AWD:
		{
			double fAWD = fRWD + (this->outShaftLF.inertia + this->outShaftRF.inertia);
			return fAWD / (fRatio * fRatio) + this->clutchInertia + this->engine.inertia;
		}
	}

	SHOULD_NOT_REACH;
	return 0;
}

float _Drivetrain::_getEngineRPM()
{
	return (float)((this->engine.velocity * 0.15915507) * 60.0);
}
