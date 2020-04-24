#pragma once

BEGIN_HOOK_OBJ(DynamicController)

	#define RVA_DynamicController_eval 2821120
	#define RVA_DynamicController_getInput 2821488
	#define RVA_DynamicController_getOversteerFactor 2822608
	#define RVA_DynamicController_getRearSpeedRatio 2822704

	float _eval();
	float _getInput(DynamicControllerInput input);
	float _getOversteerFactor(Car *car);
	float _getRearSpeedRatio(Car *car);

END_HOOK_OBJ()

float _DynamicController::_eval()
{
	float i = 0;

	auto pStage = this->stages.begin();
	auto pLast = this->stages.end();

	for (; pStage != pLast; ++pStage)
	{
		float fCurValue = pStage->currentValue;
		float fNewValue = 0;

		if (pStage->inputVar == DynamicControllerInput::Const)
		{
			fNewValue = pStage->constValue;
		}
		else
		{
			float fInput = this->getInput(pStage->inputVar);
			fNewValue = pStage->lut.getValue(fInput);
		}

		if (fabsf(fNewValue - fCurValue) >= 0.001f)
		{
			float fFilter = tclamp<float>(pStage->filter * 0.003f, 0.0f, 1.0f);
			fNewValue = ((fNewValue - fCurValue) * fFilter) + fCurValue;
		}

		pStage->currentValue = fNewValue;

		switch (pStage->combinatorMode)
		{
			case DynamicControllerCombinatorMode::eAdd:
				i = i + fNewValue;
				break;
			case DynamicControllerCombinatorMode::eMult:
				i = i * fNewValue;
				break;
			default:
				i = 0;
				break;
		}

		if (pStage->downLimit != 0.0f || pStage->upLimit != 0.0f)
		{
			i = tclamp<float>(i, pStage->downLimit, pStage->upLimit);
		}
	}

	return i;
}

float _DynamicController::_getInput(DynamicControllerInput input)
{
	switch (input)
	{
		case DynamicControllerInput::Brake:
			return this->car->controls.brake;

		case DynamicControllerInput::Gas:
			return this->car->controls.gas;

		case DynamicControllerInput::LatG:
			return this->car->accG.x;

		case DynamicControllerInput::LonG:
			return this->car->accG.z;

		case DynamicControllerInput::Steer:
			return this->car->controls.steer;

		case DynamicControllerInput::Speed:
			return (Car_getSpeedValue(this->car) * 3.6f);

		case DynamicControllerInput::Gear:
			return (float)(this->car->drivetrain.currentGear - 1);

		case DynamicControllerInput::SlipRatioMAX:
			if (this->car->drivetrain.tractionType == TractionType::RWD)
			{
				return tmax<float>(this->car->tyres[2].status.slipRatio, this->car->tyres[3].status.slipRatio);
			}
			else if (this->car->drivetrain.tractionType == TractionType::FWD)
			{
				return tmax<float>(this->car->tyres[0].status.slipRatio, this->car->tyres[1].status.slipRatio);
			}
			else
			{
				return tmax<float>(
					tmax<float>(this->car->tyres[0].status.slipRatio, this->car->tyres[1].status.slipRatio),
					tmax<float>(this->car->tyres[2].status.slipRatio, this->car->tyres[3].status.slipRatio));
			}

		case DynamicControllerInput::SlipRatioAVG:
			if (this->car->drivetrain.tractionType == TractionType::RWD)
			{
				return (this->car->tyres[2].status.slipRatio + this->car->tyres[3].status.slipRatio) * 0.5f;
			}
			else if (this->car->drivetrain.tractionType == TractionType::FWD)
			{
				return (this->car->tyres[0].status.slipRatio + this->car->tyres[1].status.slipRatio) * 0.5f;
			}
			else
			{
				return (
					this->car->tyres[0].status.slipRatio + this->car->tyres[1].status.slipRatio +
					this->car->tyres[2].status.slipRatio + this->car->tyres[3].status.slipRatio) * 0.25f;
			}

		case DynamicControllerInput::SlipAngleFrontAVG:
			return ((this->car->tyres[0].status.slipAngleRAD + this->car->tyres[1].status.slipAngleRAD) * 57.29578f) * 0.5f;

		case DynamicControllerInput::SlipAngleRearAVG:
			return ((this->car->tyres[2].status.slipAngleRAD + this->car->tyres[3].status.slipAngleRAD) * 57.29578f) * 0.5f;

		case DynamicControllerInput::SlipAngleFrontMAX:
			return tmax<float>(fabsf(this->car->tyres[0].status.slipAngleRAD), fabsf(this->car->tyres[1].status.slipAngleRAD)) * 57.29578f;

		case DynamicControllerInput::SlipAngleRearMAX:
			return tmax<float>(fabsf(this->car->tyres[2].status.slipAngleRAD), fabsf(this->car->tyres[3].status.slipAngleRAD)) * 57.29578f;

		case DynamicControllerInput::OversteerFactor:
			return DynamicController::getOversteerFactor(this->car);

		case DynamicControllerInput::RearSpeedRatio:
			return DynamicController::getRearSpeedRatio(this->car);

		case DynamicControllerInput::SteerDEG:
			return this->car->steerLock * this->car->controls.steer;

		case DynamicControllerInput::RPMS:
			return this->car->drivetrain.getEngineRPM();

		case DynamicControllerInput::WheelSteerDEG:
			return this->car->finalSteerAngleSignal;

		case DynamicControllerInput::LoadSpreadLF:
			return this->car->tyres[0].status.load / (this->car->tyres[0].status.load + this->car->tyres[1].status.load);

		case DynamicControllerInput::LoadSpreadRF:
			return this->car->tyres[1].status.load / (float)(this->car->tyres[1].status.load + this->car->tyres[0].status.load);

		case DynamicControllerInput::AvgTravelRear:
			return (this->car->suspensions[2]->getStatus().travel + this->car->suspensions[3]->getStatus().travel) * 0.5f * 1000.0f;

		case DynamicControllerInput::SusTravelLR:
			return this->car->suspensions[2]->getStatus().travel * 1000.0f;

		case DynamicControllerInput::SusTravelRR:
			return this->car->suspensions[3]->getStatus().travel * 1000.0f;
	}

	return 0;
}

float _DynamicController::_getOversteerFactor(Car *car)
{
	return
		((fabsf(car->tyres[2].status.slipAngleRAD) + fabsf(car->tyres[3].status.slipAngleRAD) * 0.5f) -
		(fabsf(car->tyres[0].status.slipAngleRAD) + fabsf(car->tyres[1].status.slipAngleRAD) * 0.5f)) * 57.29578f;
}

float _DynamicController::_getRearSpeedRatio(Car *car)
{
	float result = 0;
	float front = (car->tyres[1].status.angularVelocity + car->tyres[0].status.angularVelocity) * 0.5f;
	if (front != 0.0f)
		result = ((car->tyres[3].status.angularVelocity + car->tyres[2].status.angularVelocity) * 0.5f) / front;
	return result;
}
