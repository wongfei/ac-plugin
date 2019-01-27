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
	union {
		float values[8];
		struct {
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
		};
	};

	void reset() { memset(values, 0, sizeof(values)); }
	void assignMax(const CarPerf& other) {
		for (int i = 0; i < _countof(values); ++i) {
			values[i] = tmax(values[i], other.values[i]);
		}
	}
};

enum class EPerfMode : int {
	Actual = 0,
	ActualMax,
	PrevLap,
	BestLap,
	COUNT
};

struct DriverState
{
	std::wstring carName;
	CarAvatar* avatar = nullptr;
	CarIni* carIni = nullptr;

	CarPerf perf[(int)EPerfMode::COUNT];
	vec3f vel;
	float accum;

	DriverState() { resetPerf(); }

	void resetPerf() {
		for (int i = 0; i < (int)EPerfMode::COUNT; ++i) {
			perf[i].reset();
		}
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

	void togglePerfMode();

protected:

	ksgui_Label* _lbSpec = nullptr;
	ksgui_Label* _lbPower = nullptr;
	ksgui_ActiveButton* _btnDump = nullptr;

	std::unique_ptr<GridView> _gridPerf;
	ksgui_ActiveButton* _btnPerf = nullptr;
	EPerfMode _perfMode = EPerfMode::Actual;

	std::unordered_map<std::wstring, CarIni*> _carIni;
	std::unordered_map<CarAvatar*, DriverState*> _drivers;
};
