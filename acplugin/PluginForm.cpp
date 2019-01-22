#include "PluginForm.h"
#include "vthook.h"

PluginForm::PluginForm(ACPlugin* plugin)
{
	static_assert(offsetof(PluginForm, _formBytes) == 0, "epic fail!");
	auto pthis = this;

	log_printf(L"+PluginForm %p", this);
	_plugin = plugin;
	_sim = plugin->sim;
	_game = plugin->sim->game;

	std::wstring name(AC_PLUGIN_NAME);
	std::wstring title(AC_PLUGIN_TITLE);
	std::wstring text;
	bool canBeScaled = false;

	_form = (ksgui_Form*)_formBytes;
	_form->ctor(&name, _game->gui, canBeScaled);
	_form->formTitle->setText(title);
	_form->devApp = false;
	_form->setSize(300, 200);
	_form->setAutoHideMode(true);
	//_form->drawBorder = 0;
	//_form->backColor.w = 0;
	vtablehook_hook(this, method_ptr(&PluginForm::onMouseDown_vf10), 10);

	name.assign(L"stat_label");
	_statLabel = new_udt<ksgui_Label>(&name, _game->gui);
	_statLabel->setPosition(10, 50);
	_statLabel->font.reset(new_udt<Font>(eFontType::eFontProportional, 20.0f, false, false));
	_statLabel->foreColor.ctor(1, 0, 0, 1);
	_form->addControl(_statLabel);

	name.assign(L"btn");
	text.assign(L"press me!");
	_button = new_udt<ksgui_ActiveButton>(&name, _game->gui);
	_button->setPosition(10, 100);
	_button->setSize(80, 30);
	_button->setText(text);
	_form->addControl(_button);

	auto btnHandler = [pthis](const ksgui_OnControlClicked&) { pthis->console_printf(L"btn click"); };
	_button->evClicked.handlers.push_back(std::pair<void*, std::function<void (const ksgui_OnControlClicked&)> >(this, btnHandler));

	loadConfig();

	if (canBeScaled) {
		_form->scaleByMult_impl();
	}

	plugin->sim->gameScreen->addControl(_form, false);
	plugin->sim->game->gui->taskbar->addForm(_form);

	console_printf(L"PLUGIN \"%s\" initialized", AC_PLUGIN_NAME);
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

void PluginForm::acpUpdate(float deltaT)
{
	_runTime += deltaT;
	wchar_t buf[64];
	swprintf(buf, _countof(buf)-1, L"runTime=%.3f; cars=%d", _runTime, (int)_sim->getCarsCount());
	std::wstring text(buf);
	_statLabel->setText(text);
}

//
// OVERRIDES
//

bool PluginForm::onMouseDown_vf10(OnMouseDownEvent & message)
{
	return _form->onMouseDown_impl(message);
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

void PluginForm::console_printf(const wchar_t* format, ...)
{
	wchar_t buf[1024] = {0};
	va_list args;
	va_start(args, format);
	const int count = _vsnwprintf_s(buf, _countof(buf) - 1, _TRUNCATE, format, args);
	va_end(args);

	if (count > 0)
	{
		std::wstring msg(buf);
		_sim->console->operator<<(&msg);
		msg.assign(L"\n");
		_sim->console->operator<<(&msg);
	}
}
