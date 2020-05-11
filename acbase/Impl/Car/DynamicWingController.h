#pragma once

BEGIN_HOOK_OBJ(DynamicWingController)

	#define RVA_DynamicWingController_step 2796144
	#define RVA_DynamicWingController_getInput 2793200

	static void _hook()
	{
		HOOK_METHOD_RVA(DynamicWingController, step);
		HOOK_METHOD_RVA(DynamicWingController, getInput);
	}

	void _step();
	float _getInput();

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

void _DynamicWingController::_step()
{
	float fInput = this->getInput();
	float fAngle = this->lut.getValue(fInput);

	if (fabsf(fAngle - this->outputAngle) >= 0.001f)
	{
		fAngle = this->outputAngle + (fAngle - this->outputAngle) * tclamp(this->filter * 0.003f, 0.0f, 1.0f);
	}

	this->outputAngle = fAngle;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _DynamicWingController::_getInput()
{
	if (this->car)
	{
		auto* pCar = this->car;
		switch (this->inputVar)
		{
			case DynamicWingController_eInputVar::eBrake:
				return pCar->controls.brake;

			case DynamicWingController_eInputVar::eGas:
				return pCar->controls.gas;

			case DynamicWingController_eInputVar::eLatG:
				return pCar->accG.x;

			case DynamicWingController_eInputVar::eLonG:
				return pCar->accG.z;

			case DynamicWingController_eInputVar::eSteer:
				return pCar->controls.steer;

			case DynamicWingController_eInputVar::eSpeed:
				return getSpeedKMH(this->car);

			case DynamicWingController_eInputVar::SusTravelLR:
				return pCar->suspensions[2]->getStatus().travel;

			case DynamicWingController_eInputVar::SusTravelRR:
				return pCar->suspensions[3]->getStatus().travel;
		}
	}
	else if (this->state)
	{
		auto* pState = this->state;
		switch (this->inputVar)
		{
			case DynamicWingController_eInputVar::eBrake:
				return pState->brake;

			case DynamicWingController_eInputVar::eGas:
				return pState->gas;

			case DynamicWingController_eInputVar::eLatG:
				return pState->accG.x;

			case DynamicWingController_eInputVar::eLonG:
				return pState->accG.z;

			case DynamicWingController_eInputVar::eSteer:
				return pState->steer;

			case DynamicWingController_eInputVar::eSpeed:
				return pState->speed.value * 3.6f;

			case DynamicWingController_eInputVar::SusTravelLR:
				return pState->suspensionTravel[2] * 1000.0f;

			case DynamicWingController_eInputVar::SusTravelRR:
				return pState->suspensionTravel[3] * 1000.0f;
		}
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
