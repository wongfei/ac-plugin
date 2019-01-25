#include "CheaterDetector.h"

#define CONST_G 9.80665f
#define WATTS_PER_HP 745.7f

CheaterDetector::CheaterDetector(ACPlugin* plugin) : PluginForm(plugin)
{
	log_printf(L"+CheaterDetector %p", this);
	auto pthis = this;

	std::wstring name(AC_PLUGIN_NAME);
	std::wstring title(AC_PLUGIN_TITLE);
	std::wstring text;
	bool canBeScaled = false;

	_form = new_udt<ksgui_Form>(&name, _game->gui, canBeScaled);
	_form->formTitle->setText(title);
	_form->setSize(500, 200);
	_form->setAutoHideMode(true);
	_form->devApp = false;

	set_udt_tag(_form, this);
	hook_onMouseDown_vf10(_form);

	name.assign(L"spec");
	_lbSpec = new_udt<ksgui_Label>(&name, _game->gui);
	_lbSpec->font.reset(new_udt<Font>(eFontType::eFontProportional, 16.0f, false, false));
	_lbSpec->setPosition(10, 40);
	_lbSpec->foreColor.ctor(1, 0, 0, 1);
	_form->addControl(_lbSpec);

	name.assign(L"power");
	_lbPower = new_udt<ksgui_Label>(&name, _game->gui);
	_lbPower->font.reset(new_udt<Font>(eFontType::eFontProportional, 16.0f, false, false));
	_lbPower->setPosition(10, 70);
	_lbPower->foreColor.ctor(1, 0, 0, 1);
	_form->addControl(_lbPower);

	name.assign(L"vectors");
	_lbVectors = new_udt<ksgui_Label>(&name, _game->gui);
	_lbVectors->font.reset(new_udt<Font>(eFontType::eFontProportional, 16.0f, false, false));
	_lbVectors->setPosition(10, 100);
	_lbVectors->foreColor.ctor(1, 0, 0, 1);
	_form->addControl(_lbVectors);

	name.assign(L"dump");
	text.assign(L"dump");
	_btnDump = new_udt<ksgui_ActiveButton>(&name, _game->gui);
	_btnDump->font.reset(new_udt<Font>(eFontType::eFontProportional, 16.0f, false, false));
	_btnDump->setPosition(10, 130);
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

	loadFormConfig();
	_form->scaleByMult_impl();

	plugin->sim->gameScreen->addControl(_form, false);
	plugin->sim->game->gui->taskbar->addForm(_form);

	writeConsole(strf(L"PLUGIN \"%s\" initialized", AC_PLUGIN_NAME));
}

CheaterDetector::~CheaterDetector()
{
	log_printf(L"~CheaterDetector %p", this);

	for (auto& iter : _drivers) {
		delete(iter.second);
	}
	_drivers.clear();

	for (auto& iter : _carIni) {
		delete(iter.second);
	}
	_carIni.clear();

	writeFormConfig();
	_form->setVisible(false);
	remove_elem(_plugin->sim->game->gui->taskbar->forms, _form);
	remove_elem(_plugin->sim->gameScreen->controls, _form);
	_form->dtor();

	return;
}

bool CheaterDetector::acpUpdate(ACCarState* carState, float deltaT)
{
	updatePlayer();
	updateDrivers();

	return true;
}

bool CheaterDetector::acpOnGui(ACPluginContext* context)
{
	return true;
}

bool CheaterDetector::onMouseDown_vf10(OnMouseDownEvent& ev)
{
	bool res = _form->onMouseDown_impl(ev);
	return res;
}

//
// INTERNALS
//

void CheaterDetector::updatePlayer()
{
	auto avatar = _plugin->carAvatar;
	auto car = avatar->physics;

	auto maxTorq = car->drivetrain.acEngine.maxTorqueNM;
	auto maxPower = car->drivetrain.acEngine.maxPowerW;
	auto maxPowerDyn = car->drivetrain.acEngine.maxPowerW_Dynamic;

	auto accG = avatar->physicsState.accG;
	auto vel = avatar->physicsState.localVelocity;
	float W = getCarPowerW(accG, vel, car->mass, car->controls.gas, car->controls.brake);

	std::wstring text;
	text.assign(strf(L"%.1fkg ; %.1fNm ; %.2fkW ; %.2fkW(d)", car->mass, maxTorq, maxPower * 0.001f, maxPowerDyn * 0.001f));
	_lbSpec->setText(text);

	text.assign(strf(L"% 6.0f kW ; % 6.0f HP", W * 0.001, W / WATTS_PER_HP));
	_lbPower->setText(text);

	text.assign(strf(L"A %.1f %.1f %.1f ; V %.1f %.1f %.1f", accG.x, accG.y, accG.z, vel.x, vel.y, vel.z));
	_lbVectors->setText(text);
}

void CheaterDetector::updateDrivers()
{
	for (auto* avatar : _sim->cars) {

		CarPhysicsState* state;
		if (avatar->physics) {
			// local player
			state = &avatar->physicsState;
		}
		else if (avatar->netCarStateProvider) {
			// net player
			if (avatar->netCarStateProvider->isDisconnected) continue;
			state = &avatar->netCarStateProvider->state;
		}
		else {
			continue;
		}

		const auto& name = avatar->driverInfo.name;
		if (name.empty()) continue;

		DriverState* driver;
		auto idriver = _drivers.find(name);
		if (idriver != _drivers.end()) {
			driver = idriver->second;
		}
		else {
			log_printf(L"new driver \"%s\"", name.c_str());
			driver = new DriverState();
			driver->name = name;
			_drivers.insert(std::pair<const std::wstring, DriverState*>(name, driver));
		}

		if (driver->carName.compare(avatar->unixName) || driver->configName.compare(avatar->configName)) {
			changeCar(driver, avatar->unixName, avatar->configName);
		}

		if (state && driver->carIni) {
			driver->maxRpm = tmax(driver->maxRpm, state->engineRPM);
			driver->maxVel = tmax(driver->maxVel, vlen(state->velocity));

			if (state->brake > 0) {
				driver->maxDecel = tmax(driver->maxDecel, vlen(state->accG));
			}
			else if (state->gas > 0) {
				driver->maxAcc = tmax(driver->maxAcc, vlen(state->accG));
			}

			float W = getCarPowerW(state->accG, state->velocity, driver->carIni->mass, state->gas, state->brake);
			driver->maxPowerKw = tmax(driver->maxPowerKw, W * 0.001f);
		}
	}
}

void CheaterDetector::dumpState()
{
	writeConsole(L"dump state");

	log_printf(L"\n# simulated cars #");
	for (auto avatar : _sim->cars) {
		if (avatar->physics) {
			auto car = avatar->physics;
			auto maxTorq = car->drivetrain.acEngine.maxTorqueNM;
			auto maxPower = car->drivetrain.acEngine.maxPowerW;
			auto powerRatio = maxPower * 0.001f / car->mass;

			log_printf(L"%.1fkg ; %.1fNm ; %.2fkW ; %.2fkW/kg ; %s ; %s", 
				car->mass, maxTorq, maxPower * 0.001f, powerRatio,
				avatar->unixName.c_str(), avatar->guiName.c_str()
			);
		}
	}

	log_printf(L"\n# drivers #");
	for (auto& iter : _drivers) {
		auto* driver = iter.second;
		log_printf(L"Rpm=%.1f V=%.1f A=%.1f D=%.1f kW=%.1f # \"%s\" -> %s (%s)", 
			driver->maxRpm, driver->maxVel, driver->maxAcc, driver->maxDecel, driver->maxPowerKw,
			driver->name.c_str(), driver->carName.c_str(), driver->configName.c_str()
		);
	}

	log_printf(L"");
}

void CheaterDetector::changeCar(DriverState* driver, std::wstring& carName, std::wstring& configName)
{
	log_printf(L"change car \"%s\" -> %s (%s)", driver->name.c_str(), carName.c_str(), configName.c_str());

	CarIni* car;
	auto icar = _carIni.find(carName);
	if (icar != _carIni.end()) {
		car = icar->second;
	}
	else {
		log_printf(L"new car \"%s\"", carName.c_str());
		car = new CarIni();
		car->name = carName;
		_carIni.insert(std::pair<const std::wstring, CarIni*>(carName, car));
		parseCarIni(car);
	}

	driver->reset();
	driver->carName = carName;
	driver->configName = configName;
	driver->carIni = car;
}

void CheaterDetector::parseCarIni(CarIni* car)
{
	log_printf(L"parse car \"%s\"", car->name.c_str());

	std::wstring path(L"content/cars/");
	path.append(car->name);
	path.append(L"/data/car.ini");

	auto ini = new_udt<INIReader>(path);

	std::wstring section(L"BASIC");
	std::wstring key(L"TOTALMASS");
	car->mass = ini->getFloat(section, key);

	del_udt(ini);
}

float CheaterDetector::getCarPowerW(const vec3f& accG, const vec3f& vel, float mass, float gas, float brake)
{
	float Wscale = (gas > 0.0f ? 1.0f : 0.0f) * (brake > 0.0f ? 0.0f : 1.0f); // * (dot(accG, vel) > 0.0f ? 1.0f : 0.0f);
	vec3f F = vmul(accG, mass * CONST_G);
	float W = vlen(F) * vlen(vel) * Wscale;
	return W;
}
