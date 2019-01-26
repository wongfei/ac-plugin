#pragma once

#include "plugin.h"

#include "ac_sdk.h"
using namespace acsdk;

#include "utils.h"
#include "gui.h"

class PluginBase
{
public:

	PluginBase(ACPlugin* plugin);
	virtual ~PluginBase();

	virtual bool acpUpdate(ACCarState* carState, float deltaT);
	virtual bool acpOnGui(ACPluginContext* context);

	inline ksgui_Form* getForm() { return _form; }

protected:

	void writeConsole(const std::wstring& text);
	std::wstring getConfigPath();
	void loadFormConfig();
	void writeFormConfig();

protected:

	ACPlugin* _plugin = nullptr;
	Sim* _sim = nullptr;
	Game* _game = nullptr;
	ksgui_Form* _form = nullptr;
};
