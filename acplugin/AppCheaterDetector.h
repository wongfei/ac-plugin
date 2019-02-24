#pragma once

#include "PluginApp.h"
#include "DriverState.h"
#include "GridView.h"

struct CarIni
{
	std::wstring unixName;
	float mass = 0;
};

class AppCheaterDetector : public PluginApp
{
public:

	AppCheaterDetector(ACPlugin* plugin);
	virtual ~AppCheaterDetector();

	virtual bool acpUpdate(ACCarState* carState, float deltaT);
	virtual bool acpOnGui(ACPluginContext* context);

public:

	UDT_OVERRIDE_METHOD(AppCheaterDetector, ksgui_Form, ksgui_Control, onMouseDown_vf10, 10, bool, (OnMouseDownEvent &ev), (ev));

	void updateState(float deltaT);
	void updateDriver(DriverState* driver, CarPhysicsState* state, float deltaT);

	void writeDatalog();
	void analyzeDatalog(DriverState* driver, ePerfParam paramId, float unitScale, float step, const wchar_t* prefix);

	void redrawControls();
	void updatePlayerStats();
	void updatePerfGrid();

	void dumpState();

	DriverState* initDriver(CarAvatar* avatar);
	DriverState* getDriver(CarAvatar* avatar);
	DriverState* getDriver(int id);

	CarIni* loadCarIni(const std::wstring& unixName);
	CarIni* getCarIni(const std::wstring& unixName);
	CarPhysicsState* getCarState(CarAvatar* avatar);

protected:

	ksgui_Label* _lbSpec = nullptr;
	ksgui_Label* _lbPower = nullptr;
	ksgui_ActiveButton* _btnDump = nullptr;

	std::unique_ptr<GridView> _gridPerf;
	ksgui_ActiveButton* _btnPerf = nullptr;

	std::unordered_map<std::wstring, CarIni*> _carIni;
	std::unordered_map<CarAvatar*, DriverState*> _drivers;
	std::vector<PerfTable<float> > _datalogTmp;
	std::vector<int> _histogram;
};
