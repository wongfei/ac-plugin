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

	// overrides
	AC_OVERRIDE_METHOD(PluginForm, ksgui_Form, ksgui_Control, onMouseDown_vf10, 10, bool, (OnMouseDownEvent &ev), (ev));
	AC_OVERRIDE_METHOD(PluginForm, ksgui_Form, ksgui_Form, onTitleClicked_vf21, 21, void, (ksgui_OnControlClicked &ev), (ev));

	// utils
	void dumpState();
	void loadConfig();
	void writeConfig();
	std::wstring getConfigPath();
	void writeConsole(const std::wstring& text);

protected:

	ACPlugin* _plugin = nullptr;
	Sim* _sim = nullptr;
	Game* _game = nullptr;
	
	ksgui_Form* _form = nullptr;
	ksgui_Label* _lbStat = nullptr;
	ksgui_ActiveButton* _btnDump = nullptr;

	float _runTime = 0;
};
