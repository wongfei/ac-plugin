#include "precompiled.h"
#include "PluginBase.h"

PluginBase::PluginBase(ACPlugin* plugin)
{
	_plugin = plugin;
	_sim = plugin->sim;
	_game = plugin->sim->game;
}

PluginBase::~PluginBase()
{
	for (auto& iter : _carIni) {
		delete(iter.second);
	}
	_carIni.clear();
}

bool PluginBase::acpUpdate(ACCarState* carState, float deltaT)
{
	return true;
}

bool PluginBase::acpOnGui(ACPluginContext* context)
{
	return true;
}

void PluginBase::addTimer(float rate, TimerCallback callback)
{
	TimerNode node;
	node.callback = callback;
	node.rate = rate;
	node.accum = 0;
	_timers.push_back(node);
}

void PluginBase::updateTimers(float deltaT)
{
	for (auto& tm : _timers) {
		tm.accum += deltaT;
		if (tm.accum >= tm.rate) {
			tm.accum -= tm.rate;
			tm.callback(tm.rate);
		}
	}
}

CarIni* PluginBase::loadCarIni(const std::wstring& unixName)
{
	log_printf(L"loadCarIni %s", unixName.c_str());

	auto car = new CarIni();
	car->unixName = unixName;
	_carIni.insert(std::pair<const std::wstring, CarIni*>(unixName, car));

	std::wstring path(L"content/cars/");
	path.append(unixName);
	path.append(L"/data/car.ini");

	auto ini = new_udt<INIReader>(path);
	{
		std::wstring section(L"BASIC");
		std::wstring key(L"TOTALMASS");
		car->mass = ini->getFloat(section, key);
	}
	del_udt(ini);

	return car;
}

CarIni* PluginBase::getCarIni(const std::wstring& unixName)
{
	auto icar = _carIni.find(unixName);
	if (icar != _carIni.end()) {
		return icar->second;
	}
	return nullptr;
}

std::wstring PluginBase::getConfigPath()
{
	auto path = getDocumentsPath();
	path.append(L"\\");
	path.append(_game->gui->applicationName);
	path.append(L"\\cfg\\apps\\");
	ensureDirExists(path);
	path.append(AC_PLUGIN_NAME L".ini");
	return path;
}

void PluginBase::loadFormConfig()
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

void PluginBase::writeFormConfig()
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

void PluginBase::writeConsole(const std::wstring& text, bool writeToLog)
{
	std::wstring msg(text);
	_sim->console->operator<<(&msg);
	std::wstring nl(L"\n");
	_sim->console->operator<<(&nl);

	if (writeToLog) {
		log_printf(L"%s", text.c_str());
	}
}
