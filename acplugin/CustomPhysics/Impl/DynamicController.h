#pragma once

#define RVA_DynamicController_eval 2821120
#define RVA_DynamicController_getInput 2821488
#define RVA_DynamicController_getOversteerFactor 2822608
#define RVA_DynamicController_getRearSpeedRatio 2822704

float DynamicController_eval(DynamicController* pThis)
{
	float i = 0;

	auto pStage = pThis->stages.begin();
	auto pLast = pThis->stages.end();

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
			float fInput = pThis->getInput(pStage->inputVar);
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

float DynamicController_getInput(DynamicController* pThis, DynamicControllerInput input)
{
	switch (input)
	{
		case DynamicControllerInput::Brake:
			return pThis->car->controls.brake;

		case DynamicControllerInput::Gas:
			return pThis->car->controls.gas;

		case DynamicControllerInput::LatG:
			return pThis->car->accG.x;

		case DynamicControllerInput::LonG:
			return pThis->car->accG.z;

		case DynamicControllerInput::Steer:
			return pThis->car->controls.steer;

		case DynamicControllerInput::Speed:
			return (Car_getSpeedValue(pThis->car) * 3.6f);

		case DynamicControllerInput::Gear:
			return (float)(pThis->car->drivetrain.currentGear - 1);

		case DynamicControllerInput::SlipRatioMAX:
			if (pThis->car->drivetrain.tractionType == TractionType::RWD)
			{
				return tmax<float>(pThis->car->tyres[2].status.slipRatio, pThis->car->tyres[3].status.slipRatio);
			}
			else if (pThis->car->drivetrain.tractionType == TractionType::FWD)
			{
				return tmax<float>(pThis->car->tyres[0].status.slipRatio, pThis->car->tyres[1].status.slipRatio);
			}
			else
			{
				return tmax<float>(
					tmax<float>(pThis->car->tyres[0].status.slipRatio, pThis->car->tyres[1].status.slipRatio),
					tmax<float>(pThis->car->tyres[2].status.slipRatio, pThis->car->tyres[3].status.slipRatio));
			}

		case DynamicControllerInput::SlipRatioAVG:
			if (pThis->car->drivetrain.tractionType == TractionType::RWD)
			{
				return (pThis->car->tyres[2].status.slipRatio + pThis->car->tyres[3].status.slipRatio) * 0.5f;
			}
			else if (pThis->car->drivetrain.tractionType == TractionType::FWD)
			{
				return (pThis->car->tyres[0].status.slipRatio + pThis->car->tyres[1].status.slipRatio) * 0.5f;
			}
			else
			{
				return (
					pThis->car->tyres[0].status.slipRatio + pThis->car->tyres[1].status.slipRatio +
					pThis->car->tyres[2].status.slipRatio + pThis->car->tyres[3].status.slipRatio) * 0.25f;
			}

		case DynamicControllerInput::SlipAngleFrontAVG:
			return ((pThis->car->tyres[0].status.slipAngleRAD + pThis->car->tyres[1].status.slipAngleRAD) * 57.29578f) * 0.5f;

		case DynamicControllerInput::SlipAngleRearAVG:
			return ((pThis->car->tyres[2].status.slipAngleRAD + pThis->car->tyres[3].status.slipAngleRAD) * 57.29578f) * 0.5f;

		case DynamicControllerInput::SlipAngleFrontMAX:
			return tmax<float>(fabsf(pThis->car->tyres[0].status.slipAngleRAD), fabsf(pThis->car->tyres[1].status.slipAngleRAD)) * 57.29578f;

		case DynamicControllerInput::SlipAngleRearMAX:
			return tmax<float>(fabsf(pThis->car->tyres[2].status.slipAngleRAD), fabsf(pThis->car->tyres[3].status.slipAngleRAD)) * 57.29578f;

		case DynamicControllerInput::OversteerFactor:
			return DynamicController::getOversteerFactor(pThis->car);

		case DynamicControllerInput::RearSpeedRatio:
			return DynamicController::getRearSpeedRatio(pThis->car);

		case DynamicControllerInput::SteerDEG:
			return pThis->car->steerLock * pThis->car->controls.steer;

		case DynamicControllerInput::RPMS:
			return pThis->car->drivetrain.getEngineRPM();

		case DynamicControllerInput::WheelSteerDEG:
			return pThis->car->finalSteerAngleSignal;

		case DynamicControllerInput::LoadSpreadLF:
			return pThis->car->tyres[0].status.load / (pThis->car->tyres[0].status.load + pThis->car->tyres[1].status.load);

		case DynamicControllerInput::LoadSpreadRF:
			return pThis->car->tyres[1].status.load / (float)(pThis->car->tyres[1].status.load + pThis->car->tyres[0].status.load);

		case DynamicControllerInput::AvgTravelRear:
			return (pThis->car->suspensions[2]->getStatus().travel + pThis->car->suspensions[3]->getStatus().travel) * 0.5f * 1000.0f;

		case DynamicControllerInput::SusTravelLR:
			return pThis->car->suspensions[2]->getStatus().travel * 1000.0f;

		case DynamicControllerInput::SusTravelRR:
			return pThis->car->suspensions[3]->getStatus().travel * 1000.0f;
	}

	return 0;
}

float DynamicController_getOversteerFactor(Car *car)
{
	return
		((fabsf(car->tyres[2].status.slipAngleRAD) + fabsf(car->tyres[3].status.slipAngleRAD) * 0.5f) -
		(fabsf(car->tyres[0].status.slipAngleRAD) + fabsf(car->tyres[1].status.slipAngleRAD) * 0.5f)) * 57.29578f;
}

float DynamicController_getRearSpeedRatio(Car *car)
{
	float result = 0;
	float front = (car->tyres[1].status.angularVelocity + car->tyres[0].status.angularVelocity) * 0.5f;
	if (front != 0.0f)
		result = ((car->tyres[3].status.angularVelocity + car->tyres[2].status.angularVelocity) * 0.5f) / front;
	return result;
}
