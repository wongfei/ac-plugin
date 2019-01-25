#include "CheaterDetector.h"

#define CONST_G 9.80665f
#define WATTS_PER_HP 745.7f

CheaterDetector::CheaterDetector(ACPlugin* plugin) : PluginBase(plugin)
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
	updateDrivers(deltaT);
	updatePlayer();

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

void CheaterDetector::updateDrivers(float deltaT)
{
	const float statRate = 1 / 5.0f;

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

		DriverState* driver = getDriver(avatar, EGetMode::GetOrCreate);

		if (!driver->carIni || driver->carName.compare(avatar->unixName)) {
			log_printf(L"changeCar \"%s\" -> %s (%s)", avatar->driverInfo.name.c_str(), avatar->unixName.c_str(), avatar->configName.c_str());
			driver->carName = avatar->unixName;
			driver->carIni = getCarIni(avatar->unixName, EGetMode::GetOrCreate);
			driver->resetStats();
		}

		if (state && driver->carIni) {

			// instant
			driver->maxRpm = tmax(driver->maxRpm, state->engineRPM);
			driver->maxVel = tmax(driver->maxVel, vlen(state->velocity));
			if (state->brake > 0) {
				driver->maxDecel = tmax(driver->maxDecel, vlen(state->accG));
			}
			else if (state->gas > 0) {
				driver->maxAcc = tmax(driver->maxAcc, vlen(state->accG));
			}

			// over time
			driver->accum += deltaT;
			if (driver->accum >= statRate) {
				driver->accum -= statRate;

				float v1 = vlen(driver->vel);
				float v2 = vlen(state->velocity);
				float e1 = driver->carIni->mass * v1 * v1 * 0.5f;
				float e2 = driver->carIni->mass * v2 * v2 * 0.5f;
				float e = e2 - e1;

				driver->power = e / statRate;
				driver->maxPower = tmax(driver->maxPower, driver->power);
				driver->vel = state->velocity;
			}
		}
	}
}

void CheaterDetector::updatePlayer()
{
	auto avatar = _plugin->carAvatar;
	auto car = avatar->physics;

	auto maxTorq = car->drivetrain.acEngine.maxTorqueNM;
	auto maxPower = car->drivetrain.acEngine.maxPowerW;
	auto maxPowerDyn = car->drivetrain.acEngine.maxPowerW_Dynamic;

	auto accG = avatar->physicsState.accG;
	auto vel = avatar->physicsState.velocity;

	auto driver = getDriver(avatar);
	float W = driver->power * 0.001f;
	float maxW = driver->maxPower * 0.001f;

	std::wstring text;
	text.assign(strf(L"%.1f kg ; %.1f Nm ; %.2f kW ; %.2f kW(d)", car->mass, maxTorq, maxPower * 0.001f, maxPowerDyn * 0.001f));
	_lbSpec->setText(text);

	text.assign(strf(L"% 6.0f kW / % 6.0f kW", W, maxW));
	_lbPower->setText(text);

	text.assign(strf(L"A %.1f %.1f %.1f ; V %.1f %.1f %.1f", accG.x, accG.y, accG.z, vel.x, vel.y, vel.z));
	_lbVectors->setText(text);
}

void CheaterDetector::dumpState()
{
	writeConsole(L"dump state");

	log_printf(L"\n# Simulated #");
	for (auto avatar : _sim->cars) {
		if (avatar->physics) {
			auto car = avatar->physics;
			auto maxTorq = car->drivetrain.acEngine.maxTorqueNM;
			auto maxPower = car->drivetrain.acEngine.maxPowerW;
			auto powerRatio = maxPower * 0.001f / car->mass;

			log_printf(L"%.1fkg ; %.1fNm ; %.2fkW ; %.2fkW/kg ; %s", 
				car->mass, maxTorq, maxPower * 0.001f, powerRatio,
				avatar->unixName.c_str()
			);
		}
	}

	log_printf(L"\n# MP Leaderboard #");
	auto& leaderboard = _sim->raceManager->mpCacheLeaderboard;
	for (auto& entry : leaderboard) {
		auto* avatar = entry.car;
		auto* driver = getDriver(avatar);
		if (driver) {
			double totalSec = entry.bestLap * 0.001;
			int bestLapM = (int)floor(totalSec / 60.0);
			double bestLapS = totalSec - bestLapM * 60;

			log_printf(L"%d:%06.3f # rpm %.1f ; V %.1f ; A %.1f ; D %.1f ; W %.1f # \"%s\" %s (%s)", 
				bestLapM, bestLapS, driver->maxRpm, driver->maxVel, driver->maxAcc, driver->maxDecel, driver->maxPower * 0.001f,
				avatar->driverInfo.name.c_str(), avatar->unixName.c_str(), avatar->configName.c_str()
			);
		}
	}

	log_printf(L"");
}

DriverState* CheaterDetector::getDriver(CarAvatar* avatar, EGetMode mode)
{
	auto idriver = _drivers.find(avatar);
	if (idriver != _drivers.end()) {
		return idriver->second;
	}

	if (mode == EGetMode::GetOrCreate) {
		log_printf(L"new DriverState");
		auto driver = new DriverState();
		driver->avatar = avatar;
		_drivers.insert(std::pair<CarAvatar*, DriverState*>(avatar, driver));
		return driver;
	}

	return nullptr;
}

CarIni* CheaterDetector::getCarIni(const std::wstring& unixName, EGetMode mode)
{
	auto icar = _carIni.find(unixName);
	if (icar != _carIni.end()) {
		return icar->second;
	}

	if (mode == EGetMode::GetOrCreate) {
		log_printf(L"new CarIni");
		auto car = new CarIni();
		car->unixName = unixName;
		parseCarIni(car);
		_carIni.insert(std::pair<const std::wstring, CarIni*>(unixName, car));
		return car;
	}

	return nullptr;
}

void CheaterDetector::parseCarIni(CarIni* car)
{
	log_printf(L"parseCarIni \"%s\"", car->unixName.c_str());

	std::wstring path(L"content/cars/");
	path.append(car->unixName);
	path.append(L"/data/car.ini");

	auto ini = new_udt<INIReader>(path);
	{
		std::wstring section(L"BASIC");
		std::wstring key(L"TOTALMASS");
		car->mass = ini->getFloat(section, key);
	}
	del_udt(ini);
}

float CheaterDetector::computeCarPowerW(const vec3f& accG, const vec3f& vel, float mass, float gas, float brake)
{
	float Wscale = (gas > 0.0f ? 1.0f : 0.0f) * (brake > 0.0f ? 0.0f : 1.0f); // * (dot(accG, vel) > 0.0f ? 1.0f : 0.0f);
	vec3f F = vmul(accG, mass * CONST_G);
	float W = vlen(F) * vlen(vel) * Wscale;
	return W;
}
