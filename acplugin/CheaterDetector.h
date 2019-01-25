#pragma once

#include "PluginBase.h"
#include <unordered_map>

struct CarIni
{
	std::wstring unixName;
	float mass = 0;
};

struct DriverState
{
	std::wstring carName;
	CarAvatar* avatar = nullptr;
	CarIni* carIni = nullptr;

	vec3f vel;
	float accum = 0;
	float maxRpm = 0;
	float maxVel = 0;
	float maxAcc = 0;
	float maxDecel = 0;
	float power = 0;
	float maxPower = 0;

	void resetStats() {
		vset(vel, 0, 0, 0);
		accum = 0;
		maxRpm = 0;
		maxVel = 0;
		maxAcc = 0;
		maxDecel = 0;
		power = 0;
		maxPower = 0;
	}
};

class CheaterDetector : public PluginBase
{
public:

	CheaterDetector(ACPlugin* plugin);
	virtual ~CheaterDetector();

	virtual bool acpUpdate(ACCarState* carState, float deltaT);
	virtual bool acpOnGui(ACPluginContext* context);

protected:

	UDT_OVERRIDE_METHOD(CheaterDetector, ksgui_Form, ksgui_Control, onMouseDown_vf10, 10, bool, (OnMouseDownEvent &ev), (ev));

	void updatePlayer();
	void updateDrivers(float deltaT);
	void dumpState();

	enum class EGetMode { GetExisting = 0, GetOrCreate };
	DriverState* getDriver(CarAvatar* avatar, EGetMode mode = EGetMode::GetExisting);
	CarIni* getCarIni(const std::wstring& unixName, EGetMode mode = EGetMode::GetExisting);
	void parseCarIni(CarIni* car);

	float computeCarPowerW(const vec3f& accG, const vec3f& vel, float mass, float gas, float brake);

protected:

	ksgui_Label* _lbSpec = nullptr;
	ksgui_Label* _lbPower = nullptr;
	ksgui_Label* _lbVectors = nullptr;
	ksgui_ActiveButton* _btnDump = nullptr;

	std::unordered_map<std::wstring, CarIni*> _carIni;
	std::unordered_map<CarAvatar*, DriverState*> _drivers;
};
