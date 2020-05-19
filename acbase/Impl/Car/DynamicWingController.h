#pragma once

BEGIN_HOOK_OBJ(DynamicWingController)

	#define RVA_DynamicWingController_ctor1 2839024
	#define RVA_DynamicWingController_ctor3 2792944
	#define RVA_DynamicWingController_ctor4 2793072
	#define RVA_DynamicWingController_initCommon 2793680
	#define RVA_DynamicWingController_step 2796144
	#define RVA_DynamicWingController_getInput 2793200

	static void _hook()
	{
		HOOK_OV_METHOD_RVA(DynamicWingController, ctor, ctor1, const DynamicWingController&);
		HOOK_OV_METHOD_RVA(DynamicWingController, ctor, ctor3, Car*, INIReader*, const std::wstring&);
		HOOK_OV_METHOD_RVA(DynamicWingController, ctor, ctor4, CarPhysicsState*, const std::wstring&, INIReader*, const std::wstring&);
		HOOK_METHOD_RVA(DynamicWingController, initCommon);
		HOOK_METHOD_RVA(DynamicWingController, step);
		HOOK_METHOD_RVA(DynamicWingController, getInput);
	}

	DynamicWingController* _ctor(const DynamicWingController& other);
	DynamicWingController* _ctor(CarPhysicsState* state, const std::wstring& carUnixName, INIReader* ini, const std::wstring& section);
	DynamicWingController* _ctor(Car* car, INIReader* ini, const std::wstring& section);
	void _initCommon(const std::wstring& carUnixName, INIReader* ini, const std::wstring& section);
	void _step();
	float _getInput();

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

DynamicWingController* _DynamicWingController::_ctor(const DynamicWingController& other)
{
	*(DynamicWingController*)this = other;
	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

DynamicWingController* _DynamicWingController::_ctor(Car* car, INIReader* ini, const std::wstring& section)
{
	AC_CTOR_THIS_POD(DynamicWingController);
	AC_CTOR_UDT(this->lut)();
	this->car = car;
	this->initCommon(car->unixName, *ini, section);
	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

DynamicWingController* _DynamicWingController::_ctor(CarPhysicsState* state, const std::wstring& carUnixName, INIReader* ini, const std::wstring& section)
{
	AC_CTOR_THIS_POD(DynamicWingController);
	AC_CTOR_UDT(this->lut)();
	this->state = state;
	this->initCommon(carUnixName, *ini, section);
	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _DynamicWingController::_initCommon(const std::wstring& carUnixName, INIReader* ini, const std::wstring& section)
{
	this->inputVar = DynamicWingController_eInputVar::eUndefined;
	this->combinatorMode = DynamicWingController_eCombinatorMode::eUndefinedMode;
	this->outputAngle = 0.0f;

	std::map<std::wstring, DynamicWingController_eInputVar> inputMap;

	#define ADD_KV(str, val)\
		inputMap.insert({str, DynamicWingController_eInputVar::val});

	ADD_KV(L"BRAKE", eBrake);
	ADD_KV(L"GAS", eGas);
	ADD_KV(L"LATG", eLatG);
	ADD_KV(L"LONG", eLonG);
	ADD_KV(L"STEER", eSteer);
	ADD_KV(L"SPEED_KMH", eSpeed);
	ADD_KV(L"SUS_TRAVEL_LR", SusTravelLR);
	ADD_KV(L"SUS_TRAVEL_RR", SusTravelRR);
	#undef ADD_KV

	auto strInput = ini->getString(section, L"INPUT");
	auto iterInput = inputMap.find(strInput);
	if (iterInput == inputMap.end())
	{
		SHOULD_NOT_REACH_WARN;
		return;
	}

	this->inputVar = iterInput->second;

	auto strCombinator = ini->getString(section, L"COMBINATOR");
	if (strCombinator == L"ADD")
		this->combinatorMode = DynamicWingController_eCombinatorMode::eAdd;
	else if (strCombinator == L"MULT")
		this->combinatorMode = DynamicWingController_eCombinatorMode::eMult;

	if (this->combinatorMode == DynamicWingController_eCombinatorMode::eUndefinedMode)
	{
		SHOULD_NOT_REACH_WARN;
		return;
	}

	// cant use car->carDataPath here
	std::wstring strDataPath(L"content/cars/" + carUnixName + L"/data/");
	auto strLut = strDataPath + ini->getString(section, L"LUT");
	this->lut.load(strLut);

	this->filter = ((1.0f - ini->getFloat(section, L"FILTER")) * 1.3333334f) * 333.33334f;
	this->upLimit = ini->getFloat(section, L"UP_LIMIT");
	this->downLimit = ini->getFloat(section, L"DOWN_LIMIT");
}

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
