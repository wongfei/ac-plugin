#pragma once

#include "plugin.h"
#include "ac_sdk.h"
#include "utils.h"

using namespace acsdk;

class PluginForm
{
public:

	PluginForm(ACPlugin* plugin);
	virtual ~PluginForm();

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
