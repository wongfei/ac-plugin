#include "CheaterDetector.h"

#define CONST_G 9.80665f
#define WATTS_PER_HP 745.7f

inline vec4f rgba(uint8_t r, uint8_t g, uint8_t b, float a) { 
	const float s = 1 / 255.0f;
	vec4f v; (v.x = r*s), (v.y = g*s), (v.z = b*s), (v.w = a);
	return v;
}

inline std::wstring lapToStr(double lapTimeMs) {
	double lapTimeSec = lapTimeMs * 0.001;
	int m = (int)floor(lapTimeSec / 60.0);
	double s = lapTimeSec - m * 60;
	return strf(L"%d:%06.3f", m, s);
}

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
	_form->setSize(600, 400);
	_form->devApp = false;
	_form->setAutoHideMode(true);
	set_udt_tag(_form, this);
	hook_onMouseDown_vf10(_form);

	std::shared_ptr<Font> font(new_udt<Font>(eFontType::eFontMonospaced, 16.0f, false, false));

	name.assign(L"spec");
	_lbSpec = new_udt<ksgui_Label>(&name, _game->gui);
	_lbSpec->font = font;
	_lbSpec->setPosition(10, 40);
	//_lbSpec->foreColor.ctor(1, 0, 0, 1);
	_form->addControl(_lbSpec);

	name.assign(L"power");
	_lbPower = new_udt<ksgui_Label>(&name, _game->gui);
	_lbPower->font = font;
	_lbPower->setPosition(10, 70);
	//_lbPower->foreColor.ctor(1, 0, 0, 1);
	_form->addControl(_lbPower);

	_gridPerf.reset(new GridView(_game->gui, 10, 8, 70, 25, 11));
	_gridPerf->setPosition(10, 100);
	_gridPerf->getControl()->backColor = rgba(17, 17, 17, 0.8f);
	_form->addControl(_gridPerf->getControl());

	name.assign(L"dump");
	text.assign(L"dump");
	_btnDump = new_udt<ksgui_ActiveButton>(&name, _game->gui);
	_btnDump->font = font;
	_btnDump->setPosition(500, 40);
	_btnDump->setSize(80, 25);
	_btnDump->setText(text);
	_btnDump->drawBorder = true;
	_btnDump->drawBackground = false;
	_btnDump->borderColor.ctor(1, 0, 0, 1);
	_btnDump->unselectedColor.ctor(0, 0, 0, 0);
	_btnDump->selectedColor.ctor(0, 0, 0, 0);
	_btnDump->rollOnColor.ctor(1, 0, 0, 0.3f);
	_btnDump->inactiveColor.ctor(0, 0, 0, 0);
	auto dumpClick = [pthis](const ksgui_OnControlClicked&) { pthis->dumpState(); };
	_btnDump->evClicked.addHandler(_btnDump, dumpClick);
	_form->addControl(_btnDump);

	name.assign(L"perf");
	text.assign(L"perf");
	_btnPerf = new_udt<ksgui_ActiveButton>(&name, _game->gui);
	_btnPerf->font = font;
	_btnPerf->setPosition(500, 70);
	_btnPerf->setSize(80, 25);
	_btnPerf->setText(text);
	_btnPerf->drawBorder = true;
	_btnPerf->drawBackground = false;
	_btnPerf->borderColor.ctor(1, 0, 0, 1);
	_btnPerf->unselectedColor.ctor(0, 0, 0, 0);
	_btnPerf->selectedColor.ctor(0, 0, 0, 0);
	_btnPerf->rollOnColor.ctor(1, 0, 0, 0.3f);
	_btnPerf->inactiveColor.ctor(0, 0, 0, 0);
	auto perfClick = [pthis](const ksgui_OnControlClicked&) { pthis->togglePerfMode(); };
	_btnPerf->evClicked.addHandler(_btnDump, perfClick);
	_form->addControl(_btnPerf);

	_perfMode = EPerfMode::COUNT;
	togglePerfMode();

	loadFormConfig();
	_form->scaleByMult();

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
	const float statRate = 1 / 3.0f;

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
			log_printf(L"OnCarChanged: \"%s\" -> %s (%s)", avatar->driverInfo.name.c_str(), avatar->unixName.c_str(), avatar->configName.c_str());
			driver->carName = avatar->unixName;
			driver->carIni = getCarIni(avatar->unixName, EGetMode::GetOrCreate);
			driver->resetPerf();
		}

		if (driver->carIni) {

			auto& perf = driver->perf[(int)EPerfMode::Actual];
			perf.irpm = state->engineRPM;
			perf.ivel = vlen(state->velocity);
			perf.iacc = 0;
			perf.idec = 0;

			if (state->brake > 0) {
				perf.idec = vlen(state->accG) * CONST_G;
			}
			else if (state->gas > 0) {
				perf.iacc = vlen(state->accG) * CONST_G;
			}

			driver->accum += deltaT;
			if (driver->accum >= statRate) {

				const float dtInv = 1 / driver->accum;
				const vec3f dv = vsub(state->velocity, driver->vel);
				const vec3f a = vmul(dv, dtInv); // a = dv / dt

				const float v1 = vlen(driver->vel);
				const float v2 = vlen(state->velocity);
				const float e1 = driver->carIni->mass * v1 * v1 * 0.5f;
				const float e2 = driver->carIni->mass * v2 * v2 * 0.5f;
				const float e = fabsf(e2 - e1);
				const float w = e * dtInv;

				perf.acc = 0;
				perf.dec = 0;
				perf.accPower = 0;
				perf.decPower = 0;

				if (v1 < v2) { // accel
					perf.acc = vlen(a);
					perf.accPower = w;
				}
				else { // decel
					perf.dec = vlen(a);
					perf.decPower = w;
				}

				driver->vel = state->velocity;
				driver->accum = 0;
			}

			if (avatar->inPitlane) {
				driver->perf[(int)EPerfMode::ActualMax].reset();
			}
			else {
				driver->perf[(int)EPerfMode::ActualMax].assignMax(perf);
			}
		}
	}

	int row = 0, col = 0;
	_gridPerf->setText(row, col++, L"time");
	_gridPerf->setText(row, col++, L"irpm");
	_gridPerf->setText(row, col++, L"ivel");
	_gridPerf->setText(row, col++, L"acc");
	_gridPerf->setText(row, col++, L"dec");
	_gridPerf->setText(row, col++, L"ap");
	_gridPerf->setText(row, col++, L"dp");
	_gridPerf->setText(row, col++, L"name");
	row++;

	const auto& leaderboard = _sim->raceManager->mpCacheLeaderboard;
	for (const auto& entry : leaderboard) {
		if (row >= _gridPerf->_rows) break;

		auto* avatar = entry.car;
		auto* driver = getDriver(avatar);
		if (driver) {

			const auto& perf = driver->perf[(int)_perfMode];
			col = 0;
			_gridPerf->setText(row, col++, lapToStr(entry.bestLap));
			_gridPerf->setText(row, col++, strf(L"%7.0f", perf.irpm));
			_gridPerf->setText(row, col++, strf(L"%6.1f", perf.ivel));
			_gridPerf->setText(row, col++, strf(L"%6.1f", perf.acc));
			_gridPerf->setText(row, col++, strf(L"%6.1f", perf.dec));
			_gridPerf->setText(row, col++, strf(L"%6.1f", perf.accPower * 0.001f));
			_gridPerf->setText(row, col++, strf(L"%6.1f", perf.decPower * 0.001f));
			_gridPerf->setText(row, col++, strf(L"%s", avatar->driverInfo.name.c_str()));
			row++;
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

	auto driver = getDriver(avatar);
	const auto& perf = driver->perf[(int)EPerfMode::Actual];
	const auto& perfM = driver->perf[(int)EPerfMode::ActualMax];

	float accPower = perf.accPower * 0.001f;
	float accPowerMax = perfM.accPower * 0.001f;
	float decPower = perf.decPower * 0.001f;
	float decPowerMax = perfM.decPower * 0.001f;

	std::wstring text;
	text.assign(strf(L"%.1f kg ; %.1f Nm ; %.2f kW ; %.2f kW(d)", car->mass, maxTorq, maxPower * 0.001f, maxPowerDyn * 0.001f));
	_lbSpec->setText(text);

	text.assign(strf(L"ap %6.1f / %6.1f ; dp %6.1f / %6.1f ", accPower, accPowerMax, decPower, decPowerMax));
	_lbPower->setText(text);
}

void CheaterDetector::dumpState()
{
	writeConsole(L"dump state");

	#if 0
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
	#endif

	log_printf(L"\n# Best laps #");
	auto& leaderboard = _sim->raceManager->mpCacheLeaderboard;
	for (auto& entry : leaderboard) {
		auto* avatar = entry.car;
		auto* driver = getDriver(avatar);
		if (driver) {
			auto bestLapStr(lapToStr(entry.bestLap));
			const auto& perf = driver->perf[(int)EPerfMode::BestLap];

			log_printf(L"%s ; irpm %7.0f ; ivel %6.1f ; acc %6.1f ; dec %6.1f ; ap %6.1f ; dp %6.1f ; %s ; \"%s\"", 
				bestLapStr.c_str(), perf.irpm, perf.ivel, 
				perf.acc, perf.dec, perf.accPower * 0.001f, perf.decPower * 0.001f,
				avatar->unixName.c_str(), avatar->driverInfo.name.c_str()
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
		auto driver = new DriverState();
		driver->avatar = avatar;
		_drivers.insert(std::pair<CarAvatar*, DriverState*>(avatar, driver));

		auto pthis = this;
		auto callback = [pthis, driver](const OnLapCompletedEvent& ev) { 

			auto timeStr(lapToStr(ev.lapTime));
			auto bestLap = pthis->_sim->raceManager->getBestLap(driver->avatar);

			pthis->writeConsole(strf(L"OnLapCompleted: %s ; %s ; \"%s\" ; valid=%d ; cuts=%d ; isBest=%d", 
				timeStr.c_str(), driver->avatar->unixName.c_str(), driver->avatar->driverInfo.name.c_str(), 
				(int)ev.isValid, (int)ev.cuts, (int)(ev.lapTime == bestLap.time)), 
				true
			);
			
			if (ev.lapTime > 0 && ev.lapTime == bestLap.time) {
				driver->perf[(int)EPerfMode::BestLap] = driver->perf[(int)EPerfMode::ActualMax];
			}

			driver->perf[(int)EPerfMode::PrevLap] = driver->perf[(int)EPerfMode::ActualMax];
			driver->perf[(int)EPerfMode::ActualMax].reset();
		};

		avatar->evOnLapCompleted.addHandler(driver, callback);
		//_sim->raceManager->evOnLapCompleted.addHandler(driver, callback);

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

void CheaterDetector::togglePerfMode()
{
	int mode = (int)_perfMode;
	if (++mode >= (int)EPerfMode::COUNT) {
		mode = 0;
	}
	_perfMode = (EPerfMode)mode;

	const wchar_t* PerfNames[] = {
		L"act",
		L"actmax",
		L"prev",
		L"best",
	};
	std::wstring text(PerfNames[mode]);
	_btnPerf->setText(text);
}
