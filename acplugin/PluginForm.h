#pragma once

#include "plugin.h"
#include "ac_sdk.h"

using namespace acsdk;

class __declspec(novtable) PluginForm
{
private:

	// extend ksgui_Form
	uint8_t _formBytes[sizeof(ksgui_Form)];
	ksgui_Form* _form;
	//

	ACPlugin* _plugin = nullptr;
	Sim* _sim = nullptr;
	Game* _game = nullptr;

	float _runTime = 0;
	ksgui_Label* _statLabel = nullptr;
	ksgui_ActiveButton* _button = nullptr;

public:

	PluginForm(ACPlugin* plugin);
	~PluginForm();
	void acpUpdate(float deltaT);
	inline ksgui_Form* getForm() { return _form; }

private:

	// overrides
	AC_OVERRIDE_METHOD(ksgui_Control, onMouseDown_vf10, bool, (OnMouseDownEvent &));

	// utils
	void loadConfig();
	void writeConfig();
	std::wstring getConfigPath();
	void console_printf(const wchar_t* format, ...);
};
