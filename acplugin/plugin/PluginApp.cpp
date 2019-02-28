#include "precompiled.h"
#include "PluginApp.h"

PluginApp::PluginApp(ACPlugin* plugin, const std::wstring& appName)
{
	_plugin = plugin;
	_sim = plugin->sim;
	_game = plugin->sim->game;
	_appName = appName;
}

PluginApp::~PluginApp()
{
	destroyForm();
}

bool PluginApp::acpUpdate(ACCarState* carState, float deltaT)
{
	updateTimers(deltaT);
	return true;
}

bool PluginApp::acpOnGui(ACPluginContext* context)
{
	return true;
}

void PluginApp::initForm(const std::wstring& title, float width, float height, unsigned int flags, void* tag)
{
	std::wstring tmp_name(_appName);
	std::wstring tmp_title(title);

	_form = new_udt<ksgui_Form>(&tmp_name, _game->gui, ((flags & FFCanBeScaled) ? true : false));
	_form->formTitle->setText(tmp_title);
	_form->setSize(width, height);
	_form->setAutoHideMode((flags & FFAutoHide) ? true : false);
	_form->devApp = false;
	set_udt_tag(_form, tag);

	loadFormConfig();
}

void PluginApp::activateForm()
{
	_form->scaleByMult();
	_sim->gameScreen->addControl(_form, false);
	_game->gui->taskbar->addForm(_form);
}

void PluginApp::destroyForm()
{
	if (_form)
	{
		writeFormConfig();
		_form->setVisible(false);
		remove_elem(_game->gui->taskbar->forms, _form);
		remove_elem(_sim->gameScreen->controls, _form);
		_form->dtor();
		_form = nullptr;
	}
}

void PluginApp::loadFormConfig()
{
	auto path = getAppConfigPath();
	scoped_udt<INIReaderDocuments> ini(new_udt<INIReaderDocuments>(&path, false));
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
}

void PluginApp::writeFormConfig()
{
	auto path = getAppConfigPath();
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

void PluginApp::addTimer(float rate, TimerCallback callback)
{
	TimerNode node;
	node.callback = callback;
	node.rate = rate;
	node.accum = 0;
	_timers.push_back(node);
}

void PluginApp::updateTimers(float deltaT)
{
	for (auto& tm : _timers) {

		tm.accum += deltaT;
		if (tm.accum >= tm.rate) {
			tm.accum -= tm.rate;

			if (tm.accum >= tm.rate) {
				tm.accum = 0;
			}

			tm.callback(tm.rate);
		}
	}
}

void PluginApp::writeConsole(const std::wstring& text, bool writeToLog)
{
	std::wstring msg(text);
	_sim->console->operator<<(&msg);
	std::wstring nl(L"\n");
	_sim->console->operator<<(&nl);

	if (writeToLog) {
		log_printf(L"%s", text.c_str());
	}
}

std::wstring PluginApp::getAppConfigPath()
{
	auto path = getDocumentsPath();
	path.append(L"\\");
	path.append(_game->gui->applicationName);
	path.append(L"\\cfg\\apps\\");
	ensureDirExists(path);
	path.append(_appName);
	path.append(L".ini");
	return path;
}
