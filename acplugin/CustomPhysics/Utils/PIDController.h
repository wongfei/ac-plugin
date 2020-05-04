#pragma once

BEGIN_HOOK_OBJ(PIDController)

	#define RVA_PIDController_eval 4515616
	#define RVA_PIDController_setPID 4515824
	#define RVA_PIDController_reset 4515808

	static void _hook()
	{
		HOOK_METHOD_RVA(PIDController, eval);
		HOOK_METHOD_RVA(PIDController, setPID);
		HOOK_METHOD_RVA(PIDController, reset);
	}

	float _eval(float targetv, float currentv, float dt);
	void _setPID(float p, float i, float d);
	void _reset();

END_HOOK_OBJ()

float _PIDController::_eval(float targetv, float currentv, float dt)
{
	float fErr = targetv - currentv;
	this->currentError = fErr;

	float fIntegral = (fErr * dt) + this->integral;
	this->integral = fIntegral;

	float fValue = (((fErr - this->currentError) / dt) * this->D) + ((fIntegral * this->I) + (fErr * this->P));

	if (isfinite(fErr) && isfinite(fIntegral) && isfinite(fValue))
	{
		return fValue;
	}

	SHOULD_NOT_REACH;
	reset();
	return 0;
}

void _PIDController::_setPID(float p, float i, float d)
{
	this->P = p;
	this->I = i;
	this->D = d;
}

void _PIDController::_reset()
{
	this->currentError = 0;
	this->integral = 0;
}
