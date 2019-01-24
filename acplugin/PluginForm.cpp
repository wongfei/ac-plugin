#include "PluginForm.h"

PluginForm::PluginForm(ACPlugin* plugin)
{
	auto pthis = this;

	log_printf(L"+PluginForm %p", this);
	_plugin = plugin;
	_sim = plugin->sim;
	_game = plugin->sim->game;

	std::wstring name(AC_PLUGIN_NAME);
	std::wstring title(AC_PLUGIN_TITLE);
	std::wstring text;
	bool canBeScaled = false;

	_form = new_udt<ksgui_Form>(&name, _game->gui, canBeScaled);
	_form->formTitle->setText(title);
	_form->devApp = false;
	_form->setSize(500, 200);
	_form->setAutoHideMode(true);

	// hook methods
	set_udt_tag(_form, this);
	hook_onMouseDown_vf10(_form);
	//hook_onTitleClicked_vf21(_form);

	name.assign(L"stat");
	_lbStat = new_udt<ksgui_Label>(&name, _game->gui);
	_lbStat->font.reset(new_udt<Font>(eFontType::eFontProportional, 16.0f, false, false));
	_lbStat->setPosition(10, 40);
	_lbStat->foreColor.ctor(1, 0, 0, 1);
	_form->addControl(_lbStat);

	name.assign(L"dump");
	text.assign(L"dump");
	_btnDump = new_udt<ksgui_ActiveButton>(&name, _game->gui);
	_btnDump->font.reset(new_udt<Font>(eFontType::eFontProportional, 16.0f, false, false));
	_btnDump->setPosition(10, 100);
	_btnDump->setSize(80, 25);
	_btnDump->setText(text);
	_btnDump->drawBorder = true;
	_btnDump->borderColor.ctor(1, 0, 0, 1);
	_btnDump->drawBackground = false;
	_btnDump->unselectedColor.ctor(0, 0, 0, 0);
	_btnDump->selectedColor.ctor(0, 0, 0, 0);
	_btnDump->rollOnColor.ctor(1, 0, 0, 0.3f);
	_btnDump->inactiveColor.ctor(0, 0, 0, 0);
	auto btnHandler = [pthis](const ksgui_OnControlClicked&) { 
		pthis->dumpState();
	};
	_btnDump->evClicked.addHandler(_btnDump, btnHandler);
	_form->addControl(_btnDump);

	loadConfig();
	_form->scaleByMult_impl();

	plugin->sim->gameScreen->addControl(_form, false);
	plugin->sim->game->gui->taskbar->addForm(_form);

	writeConsole(strf(L"PLUGIN \"%s\" initialized", AC_PLUGIN_NAME));
}

PluginForm::~PluginForm()
{
	log_printf(L"~PluginForm %p", this);
	#if 1
		writeConfig();
		_form->setVisible(false);
		remove_elem(_plugin->sim->game->gui->taskbar->forms, _form);
		remove_elem(_plugin->sim->gameScreen->controls, _form);
		_form->dtor();
	#endif
	return;
}

bool PluginForm::acpUpdate(ACCarState* carState, float deltaT)
{
	_runTime += deltaT;

	auto car = _plugin->car;
	auto& acc = car->accG;
	auto maxTorq = car->drivetrain.acEngine.maxTorqueNM;
	auto maxPower = car->drivetrain.acEngine.maxPowerW;
	auto maxPowerDyn = car->drivetrain.acEngine.maxPowerW_Dynamic;
	auto powerRatio = maxPowerDyn * 0.001f / car->mass;
	
	auto text(strf(L"%.1fkg ; %.1fNm ; %.2fkW ; %.2fkW(d) ; %.2fkW/kg(d)", 
		car->mass, maxTorq, maxPower * 0.001f, maxPowerDyn * 0.001f, powerRatio));

	_lbStat->setText(text);

	return true;
}

bool PluginForm::acpOnGui(ACPluginContext* context)
{
	return true;
}

void PluginForm::dumpState()
{
	writeConsole(L"dump state");

	log_printf(L"# simulated cars #");
	for (auto av : _sim->cars) {
		// av->physics is null in multiplayer
		if (!av || !av->physics) continue;

		auto car = av->physics;
		auto maxTorq = car->drivetrain.acEngine.maxTorqueNM;
		auto maxPower = car->drivetrain.acEngine.maxPowerW;
		auto powerRatio = maxPower * 0.001f / car->mass;

		log_printf(L"%s ; %s ; %.1fkg ; %.1fNm ; %.2fkW ; %.2fkW/kg", 
			av->unixName.c_str(), av->guiName.c_str(), 
			car->mass, maxTorq, maxPower * 0.001f, powerRatio
		);
	}
}

//
// OVERRIDES
//

bool PluginForm::onMouseDown_vf10(OnMouseDownEvent& ev)
{
	//writeConsole(L"onMouseDown_vf10");
	bool res = _form->onMouseDown_impl(ev);
	return res;
}

void PluginForm::onTitleClicked_vf21(ksgui_OnControlClicked& ev)
{
	//writeConsole(L"onTitleClicked_vf21");
	_form->onTitleClicked_impl(ev);
}

//
// UTILS
//

void PluginForm::loadConfig()
{
	auto path = getConfigPath();
	auto ini = new_udt<INIReaderDocuments>(&path, false);
	std::wstring section, key;

	section.assign(L"form_config");
	if (ini->hasSection(section)) {

		key.assign(L"POSX");
		float px = ini->getFloat(section, key);
		key.assign(L"POSY");
		float py = ini->getFloat(section, key);
		_form->setPosition(px, py);

		key.assign(L"VISIBLE");
		int visible = ini->getInt(section, key);
		_form->setVisible(visible ? true : false);

		key.assign(L"BLOCKED");
		int blocked = ini->getInt(section, key);
		_form->setBlocked(blocked ? true : false);
	}

	del_udt(ini);
}

void PluginForm::writeConfig()
{
	auto path = getConfigPath();
	std::wofstream out;
	out.open(path.c_str(), std::wofstream::out);
	{
		out << L"[form_config]" << std::endl;
		out << L"POSX=" << _form->rect.left << std::endl;
		out << L"POSY=" << _form->rect.top << std::endl;
		out << L"VISIBLE=" << (int)_form->visible << std::endl;
		out << L"BLOCKED=" << (int)_form->blocked << std::endl;
	}
	out.close();
}

std::wstring PluginForm::getConfigPath()
{
	auto path = getDocumentsPath();
	path.append(L"\\");
	path.append(_game->gui->applicationName);
	path.append(L"\\cfg\\apps\\");
	ensureDirExists(path);
	path.append(AC_PLUGIN_NAME L".ini");
	return path;
}

void PluginForm::writeConsole(const std::wstring& text)
{
	std::wstring msg(text);
	_sim->console->operator<<(&msg);
	std::wstring nl(L"\n");
	_sim->console->operator<<(&nl);
}
