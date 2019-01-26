#pragma once

#include "PluginBase.h"
#include <unordered_map>

struct CarIni
{
	std::wstring unixName;
	float mass = 0;
};

struct CarPerf
{
	// instant
	float irpm;
	float ivel;
	float iacc;
	float idec;

	// avg
	float acc;
	float dec;
	float accPower;
	float decPower;

	void reset() { memset(this, 0, sizeof(CarPerf)); }
};

struct DriverState
{
	std::wstring carName;
	CarAvatar* avatar = nullptr;
	CarIni* carIni = nullptr;

	CarPerf perf;
	CarPerf perfMax;
	CarPerf perfBest;

	vec3f vel;
	float accum;

	DriverState() { resetPerf(); }

	void resetPerf() {
		perf.reset();
		perfMax.reset();
		perfBest.reset();
		vset(vel, 0, 0, 0);
		accum = 0;
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

	std::unique_ptr<GridView> _grid;

	std::unordered_map<std::wstring, CarIni*> _carIni;
	std::unordered_map<CarAvatar*, DriverState*> _drivers;
};
