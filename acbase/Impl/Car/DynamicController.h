#pragma once

BEGIN_HOOK_OBJ(DynamicController)

	#define RVA_DynamicController_ctor0 2819936
	#define RVA_DynamicController_ctor2 2814768
	#define RVA_DynamicController_eval 2821120
	#define RVA_DynamicController_getInput 2821488
	#define RVA_DynamicController_getOversteerFactor 2822608
	#define RVA_DynamicController_getRearSpeedRatio 2822704

	static void _hook()
	{
		HOOK_OV_METHOD_RVA(DynamicController, ctor, ctor0);
		HOOK_OV_METHOD_RVA(DynamicController, ctor, ctor2, Car*, const std::wstring&);
		HOOK_METHOD_RVA(DynamicController, eval);
		HOOK_METHOD_RVA(DynamicController, getInput);
		HOOK_METHOD_RVA(DynamicController, getOversteerFactor);
		HOOK_METHOD_RVA(DynamicController, getRearSpeedRatio);
	}

	DynamicController* _ctor();
	DynamicController* _ctor(Car* car, const std::wstring& filename);
	float _eval();
	float _getInput(DynamicControllerInput input);
	static float _getOversteerFactor(Car *car);
	static float _getRearSpeedRatio(Car *car);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

DynamicController* _DynamicController::_ctor()
{
	AC_CTOR_THIS_POD(DynamicController);
	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

inline float lagToLerpDeltaK(float lag, float orgdt, float dt)
{
  return (((1.0f / dt) * orgdt) * (1.0f - lag)) * (1.0f / dt);
}

DynamicController* _DynamicController::_ctor(Car* car, const std::wstring& filename)
{
	AC_CTOR_THIS_POD(DynamicController);

	this->car = car;

	auto ini(new_udt_unique<INIReader>(filename));
	if (!ini->ready)
	{
		SHOULD_NOT_REACH_WARN;
		return this;
	}

	std::map<std::wstring, DynamicControllerInput> inputMap; // TODO: stupid shit

	#define ADD_KV(str, val)\
		inputMap.insert({str, DynamicControllerInput::val});

	ADD_KV(L"BRAKE", Brake);
	ADD_KV(L"GAS", Gas);
	ADD_KV(L"LATG", LatG);
	ADD_KV(L"LONG", LonG);
	ADD_KV(L"STEER", Steer);
	ADD_KV(L"SPEED_KMH", Speed);
	ADD_KV(L"GEAR", Gear);
	ADD_KV(L"SLIPRATIO_MAX", SlipRatioMAX);
	ADD_KV(L"SLIPRATIO_AVG", SlipRatioAVG);
	ADD_KV(L"SLIPANGLE_FRONT_AVG", SlipAngleFrontAVG);
	ADD_KV(L"SLIPANGLE_REAR_AVG", SlipAngleRearAVG);
	ADD_KV(L"SLIPANGLE_FRONT_MAX", SlipAngleFrontMAX);
	ADD_KV(L"SLIPANGLE_REAR_MAX", SlipAngleRearMAX);
	ADD_KV(L"OVERSTEER_FACTOR", OversteerFactor);
	ADD_KV(L"REAR_SPEED_RATIO", RearSpeedRatio);
	ADD_KV(L"STEER_DEG", SteerDEG);
	ADD_KV(L"CONST", Const);
	ADD_KV(L"RPMS", RPMS);
	ADD_KV(L"WHEEL_STEER_DEG", WheelSteerDEG);
	ADD_KV(L"LOAD_SPREAD_LF", LoadSpreadLF);
	ADD_KV(L"LOAD_SPREAD_RF", LoadSpreadRF);
	ADD_KV(L"AVG_TRAVEL_REAR", AvgTravelRear);
	ADD_KV(L"SUS_TRAVEL_LR", SusTravelLR);
	ADD_KV(L"SUS_TRAVEL_RR", SusTravelRR);
	#undef ADD_KV

	for (int id = 0; ; ++id)
	{
		auto strId = strf(L"CONTROLLER_%d", id);
		if (!ini->hasSection(strId))
			break;

		auto strInput = ini->getString(strId, L"INPUT");
		auto iterInput = inputMap.find(strInput);
		if (iterInput == inputMap.end())
		{
			SHOULD_NOT_REACH_WARN;
			continue;
		}

		auto eCombinator = DynamicControllerCombinatorMode::eUndefinedMode;
		auto strCombinator = ini->getString(strId, L"COMBINATOR");

		if (strCombinator == L"ADD")
			eCombinator = DynamicControllerCombinatorMode::eAdd;
		else if (strCombinator == L"MULT")
			eCombinator = DynamicControllerCombinatorMode::eMult;

		if (eCombinator == DynamicControllerCombinatorMode::eUndefinedMode)
		{
			SHOULD_NOT_REACH_WARN;
			continue;
		}

		auto stage(new_udt_unique<DynamicControllerStage>());
		
		stage->inputVar = iterInput->second;
		stage->combinatorMode = eCombinator;

		if (stage->inputVar == DynamicControllerInput::Const)
		{
			stage->constValue = ini->getFloat(strId, L"CONST_VALUE");
		}
		else
		{
			stage->lut = ini->getCurve(strId, L"LUT");
		}

		stage->filter = lagToLerpDeltaK(ini->getFloat(strId, L"FILTER"), 0.004f, 0.003f);
		stage->upLimit = ini->getFloat(strId, L"UP_LIMIT");
		stage->downLimit = ini->getFloat(strId, L"DOWN_LIMIT");

		this->stages.push_back(*stage.get());
	}

	this->ready = !this->stages.empty();

	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

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
			float fFilter = tclamp(pStage->filter * 0.003f, 0.0f, 1.0f);
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
			i = tclamp(i, pStage->downLimit, pStage->upLimit);
		}
	}

	return i;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

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
			return getSpeedKMH(this->car);

		case DynamicControllerInput::Gear:
			return (float)(this->car->drivetrain.currentGear - 1);

		case DynamicControllerInput::SlipRatioMAX:
			if (this->car->drivetrain.tractionType == TractionType::RWD)
			{
				return tmax(this->car->tyres[2].status.slipRatio, this->car->tyres[3].status.slipRatio);
			}
			else if (this->car->drivetrain.tractionType == TractionType::FWD)
			{
				return tmax(this->car->tyres[0].status.slipRatio, this->car->tyres[1].status.slipRatio);
			}
			else
			{
				return tmax(
					tmax(this->car->tyres[0].status.slipRatio, this->car->tyres[1].status.slipRatio),
					tmax(this->car->tyres[2].status.slipRatio, this->car->tyres[3].status.slipRatio));
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

///////////////////////////////////////////////////////////////////////////////////////////////////

float _DynamicController::_getOversteerFactor(Car *car)
{
	return
		((fabsf(car->tyres[2].status.slipAngleRAD) + fabsf(car->tyres[3].status.slipAngleRAD) * 0.5f) -
		(fabsf(car->tyres[0].status.slipAngleRAD) + fabsf(car->tyres[1].status.slipAngleRAD) * 0.5f)) * 57.29578f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _DynamicController::_getRearSpeedRatio(Car *car)
{
	float result = 0;
	float front = (car->tyres[1].status.angularVelocity + car->tyres[0].status.angularVelocity) * 0.5f;
	if (front != 0.0f)
		result = ((car->tyres[3].status.angularVelocity + car->tyres[2].status.angularVelocity) * 0.5f) / front;
	return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
