#pragma once

#include "plugin.h"

#include "ac_sdk.h"
using namespace acsdk;

#include "udt.h"
#include "utils.h"

typedef std::function<void (float deltaT)> TimerCallback;

struct TimerNode
{
	TimerCallback callback;
	float rate;
	float accum;
};

class PluginApp
{
public:

	PluginApp(ACPlugin* plugin, const std::wstring& appName);
	virtual ~PluginApp();

	virtual bool acpUpdate(ACCarState* carState, float deltaT);
	virtual bool acpOnGui(ACPluginContext* context);

	void addTimer(float rate, TimerCallback callback);
	void updateTimers(float deltaT);

	std::wstring getConfigPath();
	void loadFormConfig();
	void writeFormConfig();

	void writeConsole(const std::wstring& text, bool writeToLog = false);

	inline ksgui_Form* getForm() { return _form; }

protected:

	ACPlugin* _plugin = nullptr;
	Sim* _sim = nullptr;
	Game* _game = nullptr;
	ksgui_Form* _form = nullptr;

	std::wstring _appName;
	std::vector<TimerNode> _timers;
};