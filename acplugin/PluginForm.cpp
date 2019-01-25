#include "PluginForm.h"

PluginForm::PluginForm(ACPlugin* plugin)
{
	_plugin = plugin;
	_sim = plugin->sim;
	_game = plugin->sim->game;
}

PluginForm::~PluginForm()
{
}

bool PluginForm::acpUpdate(ACCarState* carState, float deltaT)
{
	return true;
}

bool PluginForm::acpOnGui(ACPluginContext* context)
{
	return true;
}

void PluginForm::writeConsole(const std::wstring& text)
{
	std::wstring msg(text);
	_sim->console->operator<<(&msg);
	std::wstring nl(L"\n");
	_sim->console->operator<<(&nl);
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

void PluginForm::loadFormConfig()
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

void PluginForm::writeFormConfig()
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
