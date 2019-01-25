#pragma once

#include "PluginForm.h"
#include <unordered_map>

struct CarIni
{
	std::wstring name;
	float mass;
};

struct DriverState
{
	std::wstring name;
	std::wstring carName;
	std::wstring configName;
	CarIni* carIni;
	float maxRpm;
	float maxVel;
	float maxAcc;
	float maxDecel;
	float maxPowerKw;

	DriverState() { reset(); }

	void reset() {
		carIni = nullptr;
		maxRpm = 0;
		maxAcc = 0;
		maxDecel = 0;
		maxVel = 0;
		maxPowerKw = 0;
	}
};

class CheaterDetector : public PluginForm
{
public:

	CheaterDetector(ACPlugin* plugin);
	virtual ~CheaterDetector();

	virtual bool acpUpdate(ACCarState* carState, float deltaT);
	virtual bool acpOnGui(ACPluginContext* context);

protected:

	AC_OVERRIDE_METHOD(CheaterDetector, ksgui_Form, ksgui_Control, onMouseDown_vf10, 10, bool, (OnMouseDownEvent &ev), (ev));

	void updatePlayer();
	void updateDrivers();
	void dumpState();
	void changeCar(DriverState* driver, std::wstring& carName, std::wstring& configName);
	void parseCarIni(CarIni* car);
	float getCarPowerW(const vec3f& accG, const vec3f& vel, float mass, float gas, float brake);

protected:

	ksgui_Label* _lbSpec = nullptr;
	ksgui_Label* _lbPower = nullptr;
	ksgui_Label* _lbVectors = nullptr;
	ksgui_ActiveButton* _btnDump = nullptr;

	std::unordered_map<std::wstring, CarIni*> _carIni;
	std::unordered_map<std::wstring, DriverState*> _drivers;
};
