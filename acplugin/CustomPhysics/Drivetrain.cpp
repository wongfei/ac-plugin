#include "precompiled.h"
#include "GameHooks.h"

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
