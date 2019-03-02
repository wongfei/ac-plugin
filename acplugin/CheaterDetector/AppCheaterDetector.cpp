#include "precompiled.h"
#include "AppCheaterDetector.h"

#define CONST_G 9.80665f
#define KW_TO_HP 1.34102f

inline vec3f getTransform(const mat44f& m) {
	return makev(m.M41, m.M42, m.M43);
}

inline std::wstring lapToStr(double lapTimeMs) {
	double lapTimeSec = lapTimeMs * 0.001;
	int m = (int)floor(lapTimeSec / 60.0);
	double s = lapTimeSec - m * 60;
	return strf(L"%d:%06.3f", m, s);
}

AppCheaterDetector::AppCheaterDetector(ACPlugin* plugin) : PluginApp(plugin, L"cheater_detector")
{
	log_printf(L"+AppCheaterDetector %p", this);
	auto pthis = this;

	initForm(L"Cheater Detector", 600, 400, PluginApp::FFAutoHide, this);
	hook_onMouseDown_vf10(_form);

	std::shared_ptr<Font> font(new_udt<Font>(eFontType::eFontMonospaced, 16.0f, false, false));
	std::wstring name, text;

	name.assign(L"spec");
	_lbSpec = new_udt<ksgui_Label>(&name, _game->gui);
	_lbSpec->font = font;
	_lbSpec->setPosition(10, 40);
	_form->addControl(_lbSpec);

	name.assign(L"power");
	_lbPower = new_udt<ksgui_Label>(&name, _game->gui);
	_lbPower->font = font;
	_lbPower->setPosition(10, 70);
	_form->addControl(_lbPower);

	_gridPerf.reset(new GridView(_game->gui, 10, 7, 65, 25, 11));
	_gridPerf->setPosition(10, 100);
	_gridPerf->getControl()->drawBorder = false;
	_gridPerf->getControl()->drawBackground = false;
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
	_btnDump->evClicked.add(_btnDump, dumpClick);
	_form->addControl(_btnDump);

	_datalogTmp.reserve(DPERF_DATALOG_BUFFER_MAX);
	addTimer(DPERF_UPDATE_FREQ_MS * 0.001f, [pthis](float deltaT) { pthis->updateState(deltaT); });
	addTimer(DPERF_DATALOG_FREQ_MS * 0.001f, [pthis](float deltaT) { pthis->writeDatalog(); });
	addTimer(0.2f, [pthis](float deltaT) { pthis->redrawControls(); });

	activateForm();

	writeConsole(strf(L"APP \"%s\" initialized", _appName.c_str()));
}

AppCheaterDetector::~AppCheaterDetector()
{
	log_printf(L"~AppCheaterDetector %p", this);

	for (auto& iter : _drivers) {
		iter.second->onDestroy();
		delete(iter.second);
	}
	_drivers.clear();

	for (auto& iter : _carIni) {
		delete(iter.second);
	}
	_carIni.clear();
}

bool AppCheaterDetector::acpUpdate(ACCarState* carState, float deltaT)
{
	return PluginApp::acpUpdate(carState, deltaT);
}

bool AppCheaterDetector::acpOnGui(ACPluginContext* context)
{
	return PluginApp::acpOnGui(context);
}

bool AppCheaterDetector::onMouseDown_vf10(OnMouseDownEvent& ev)
{
	//writeConsole(L"click");
	bool res = _form->onMouseDown_impl(ev);
	return res;
}

//
// INTERNALS
//

void AppCheaterDetector::updateState(float deltaT)
{
	for (auto* avatar : _sim->cars) {

		CarPhysicsState* state = nullptr;
		if (!avatar->driverInfo.name.empty()) {
			state = getCarState(avatar);
		}

		DriverState* driver = getDriver(avatar);
		if (!driver && state) {
			driver = initDriver(avatar);
		}

		if (driver) {
			driver->online = (state ? true : false);
			if (driver->online) {
				updateDriver(driver, state, deltaT);
			}
		}
	}
}

DriverState* AppCheaterDetector::initDriver(CarAvatar* avatar)
{
	log_printf(L"initDriver ID%d", avatar->guid);

	auto driver = new DriverState();
	driver->avatar = avatar;
	_drivers.insert(std::pair<CarAvatar*, DriverState*>(avatar, driver));

	auto pthis = this;
	auto onLapCompleted = [pthis](const OnLapCompletedEvent& ev) { 
		DriverState* driver = pthis->getDriver(ev.carIndex);
		if (!driver) return;
		if (!driver->online) return;

		// onLapCompleted called multiple times, save lapTime to execute once
		if (driver->complLapTime != ev.lapTime) {
			driver->complLapTime = ev.lapTime;

			bool isBest = false;
			for (const auto& entry : pthis->_sim->raceManager->mpCacheLeaderboard) {
				if (entry.car == driver->avatar && ((double)ev.lapTime <= entry.bestLap || fabs(entry.bestLap - (double)ev.lapTime) < 1)) {
					isBest = true;
					break;
				}
			}
			const int maxSamples = (int)(ev.lapTime / DPERF_DATALOG_FREQ_MS); // total time ms / sample freq ms
			auto timeStr(lapToStr(ev.lapTime));

			pthis->writeConsole(strf(L"OnLapCompleted: ID%d \"%s\" ; %s ; Time %s (%u) ; NumCuts %d ; IsValid %d ; IsBest %d ; NumSamples %d / %d", 
				driver->avatar->guid, driver->avatar->driverInfo.name.c_str(), driver->avatar->unixName.c_str(), 
				timeStr.c_str(), ev.lapTime, (int)ev.cuts, (int)ev.isValid, (int)isBest, (int)driver->datalog.size(), (int)maxSamples
				), true
			);
			if (driver->datalog.size() > maxSamples) {
				std::vector<PerfTable<float> > tmp(driver->datalog.end() - maxSamples, driver->datalog.end());
				driver->datalog.swap(tmp);
			}
			if (driver->datalog.size() > 1) {
				pthis->analyzeDatalog(driver, ePerfParam::AccP, KW_TO_HP, 20, L"AccPower(hp) distribution");
				pthis->analyzeDatalog(driver, ePerfParam::Rpm, 1, 500, L"RPM distribution");
			}
			driver->datalog.clear();
			driver->avg.reset();
		}
	};
	_sim->raceManager->evOnLapCompleted.add(driver, onLapCompleted);

	driver->onDestroy = [pthis, driver]() { 
		pthis->_sim->raceManager->evOnLapCompleted.swapAndPop(driver);
	};

	return driver;
}

void AppCheaterDetector::updateDriver(DriverState* driver, CarPhysicsState* state, float deltaT)
{
	auto avatar = driver->avatar;
	const vec3f curPos = getTransform(avatar->bodyMatrix);
	const vec3f curVel = state->velocity;

	if (driver->driverName.compare(avatar->driverInfo.name)) {
		writeConsole(strf(L"OnDriverChanged: ID%d -> \"%s\"", avatar->guid, avatar->driverInfo.name.c_str()), true);
		driver->driverName = avatar->driverInfo.name;
		driver->carIni = nullptr;
	}

	if (!driver->carIni || driver->carName.compare(avatar->unixName)) {
		writeConsole(strf(L"OnCarChanged: ID%d \"%s\" -> %s (%s)", avatar->guid, avatar->driverInfo.name.c_str(), avatar->unixName.c_str(), avatar->configName.c_str()), true);
		driver->carName = avatar->unixName;

		driver->carIni = getCarIni(avatar->unixName);
		if (!driver->carIni) {
			driver->carIni = loadCarIni(avatar->unixName);
		}

		driver->resetPerf();
	}

	auto& perf = driver->perf;
	perf[ePerfParam::Rpm] = state->engineRPM;
	perf[ePerfParam::Vel] = vlen(curVel);

	const float dtinv = 1.0f / deltaT;
	const vec3f dvel = vsub(curVel, driver->vel);
	const vec3f accel = vmul(dvel, dtinv); // accel_avg = dv / dt

	const float v1 = vlen(driver->vel);
	const float v2 = vlen(curVel);

	if (v1 < v2 && state->gas > 0) { // accel
		const float e1 = driver->carIni->mass * v1 * v1 * 0.5f;
		const float e2 = driver->carIni->mass * v2 * v2 * 0.5f;
		const float e = fabsf(e2 - e1);
		const float accPower = e * dtinv * 0.001f; // kW
		perf[ePerfParam::Acc] = vlen(accel);
		perf[ePerfParam::AccP] = accPower;

		/*
		const vec3f dpos = vsub(curPos, driver->pos);
		const float dist = vlen(dpos);
		const vec3f force = vmul(accel, driver->carIni->mass); // F = ma
		const float power = vlen(force) * dist * dtinv * 0.001f; // P = F * dist / dt
		*/
	}
	else { // decel
		perf[ePerfParam::Acc] = 0;
		perf[ePerfParam::AccP] = 0;
	}

	driver->pos = curPos;
	driver->vel = curVel;

	driver->computeAvg();
}

void AppCheaterDetector::writeDatalog()
{
	for (auto& entry : _drivers) {
		auto driver = entry.second;
		if (driver->online && !driver->avatar->inPitlane) {

			if (driver->datalog.size() + 1 >= DPERF_DATALOG_BUFFER_MAX) {
				// overflow
				_datalogTmp.clear();
				_datalogTmp.assign(driver->datalog.begin() + DPERF_DATALOG_BUFFER_MAX / 10, driver->datalog.end());
				driver->datalog.swap(_datalogTmp);
			}

			driver->datalog.emplace_back();
			driver->datalog.back() = driver->avg;
		}
	}
}

void AppCheaterDetector::analyzeDatalog(DriverState* driver, ePerfParam paramId, float unitScale, float step, const wchar_t* prefix)
{
	const size_t sampleCount = driver->datalog.size();
	if (!sampleCount) {
		log_printf(L"  datalog is empty");
		return;
	}

	float sum = 0;
	float maxValue = 0;
	size_t nzSampleCount = 0;

	for (const auto& iter : driver->datalog) {
		const float value = iter[paramId] * unitScale;
		if (value > 0) {
			maxValue = tmax(maxValue, value);
			sum += value;
			++nzSampleCount;
		}
	}

	if (!nzSampleCount) {
		log_printf(L"  no valid samples");
		return;
	}

	const float avg = sum / (float)nzSampleCount;
	const float histRange = maxValue;
	const size_t histSize = (size_t)(histRange / step) + 1;

	if (histSize <= 1 || histSize > 100) {
		log_printf(L"  invalid histSize=%u histRange=%.2f step=%.2f", (unsigned int)histSize, histRange, step);
		return;
	}

	if (_histogram.size() < histSize) {
		_histogram.resize(histSize);
	}
	memset(&_histogram[0], 0, sizeof(_histogram[0]) * histSize);

	size_t bad = 0;
	for (const auto& iter : driver->datalog) {
		const float value = iter[paramId] * unitScale;
		if (value > 0) {
			const size_t id = (size_t)(value / step);
			if (id < histSize) {
				++_histogram[id];
			}
			else {
				++bad;
			}
		}
	}

	writeConsole(strf(L"  %s: avg %.2f ; max %.2f ; histSize %d ; nzSamples %u ; bad %u", 
		prefix, avg, maxValue, histSize, (unsigned int)nzSampleCount, (unsigned int)bad
	), true);

	for (size_t i = 1; i <= histSize; ++i) {
		const size_t id = histSize - i;
		if (_histogram[id]) {
			const float perc = _histogram[id] / (float)nzSampleCount;
			if (perc >= 0.01f) {
				const float value =  (id + 1) * step;
				writeConsole(strf(L"  %.0f -> %.2f", value, perc), true);
			}
		}
	}

	return;
}

void AppCheaterDetector::redrawControls()
{
	updatePlayerStats();
	updatePerfGrid();
}

void AppCheaterDetector::updatePlayerStats()
{
	auto avatar = _plugin->carAvatar;

	auto car = avatar->physics;
	auto maxTorq = car->drivetrain.acEngine.maxTorqueNM;
	auto maxPower = car->drivetrain.acEngine.maxPowerW * 0.001f * KW_TO_HP;
	auto maxPowerDyn = car->drivetrain.acEngine.maxPowerW_Dynamic * 0.001f * KW_TO_HP;

	auto driver = getDriver(avatar);
	const float accPower = driver->avg[ePerfParam::AccP].val() * KW_TO_HP;
	const float power = driver->avg[ePerfParam::AccP].maxv() * KW_TO_HP;

	std::wstring text;
	text.assign(strf(L"Spec: %.1f kg ; %.1f nm ; %.1f hp ; %.1f hp(d)", car->mass, maxTorq, maxPower, maxPowerDyn));
	_lbSpec->setText(text);

	text.assign(strf(L"AccPower(hp): %6.1f / %6.1f ; LapTime %u ; SplinePos %.2f", accPower, power, avatar->physicsState.lapTime, avatar->physicsState.normalizedSplinePosition));
	_lbPower->setText(text);
}

void AppCheaterDetector::updatePerfGrid()
{
	std::wstring name;

	int row = 0, col = 0;
	_gridPerf->setText(row, col++, L"time");
	_gridPerf->setText(row, col++, L"rpm");
	_gridPerf->setText(row, col++, L"vel");
	_gridPerf->setText(row, col++, L"acc");
	_gridPerf->setText(row, col++, L"ap");
	_gridPerf->setText(row, col++, L"ap_max");
	_gridPerf->setText(row, col++, L"name");
	row++;

	const auto& leaderboard = _sim->raceManager->mpCacheLeaderboard;
	for (const auto& entry : leaderboard) {
		if (row >= _gridPerf->_rows) break;

		auto* avatar = entry.car;
		auto* driver = getDriver(avatar);
		if (driver) {

			name = avatar->guiShortName;
			name.append(L" / ");
			name.append(avatar->driverInfo.name);

			col = 0;
			_gridPerf->setText(row, col++, lapToStr(entry.bestLap));
			_gridPerf->setText(row, col++, strf(L"%7.0f", driver->avg[ePerfParam::Rpm].val()));
			_gridPerf->setText(row, col++, strf(L"%6.1f", driver->avg[ePerfParam::Vel].val()));
			_gridPerf->setText(row, col++, strf(L"%6.1f", driver->avg[ePerfParam::Acc].val()));
			_gridPerf->setText(row, col++, strf(L"%6.1f", driver->avg[ePerfParam::AccP].val() * KW_TO_HP));
			_gridPerf->setText(row, col++, strf(L"%6.1f", driver->avg[ePerfParam::AccP].maxv() * KW_TO_HP));
			_gridPerf->setText(row, col++, strf(L"%s", name.c_str()));
			row++;
		}
	}
}

void AppCheaterDetector::dumpState()
{
	writeConsole(L"dump state");

	#if 1
	log_printf(L"\n# Car spec #");
	for (auto avatar : _sim->cars) {
		if (avatar->physics) {
			auto car = avatar->physics;
			auto maxTorq = car->drivetrain.acEngine.maxTorqueNM;
			auto maxPower = car->drivetrain.acEngine.maxPowerW * 0.001f * KW_TO_HP;
			auto powerRatio = maxPower / car->mass;

			log_printf(L"%.1f kg ; %.1f nm ; %.2f hp ; %.2f hp/kg ; %s", 
				car->mass, maxTorq, maxPower, powerRatio, avatar->unixName.c_str()
			);
		}
	}
	#endif

	log_printf(L"\n# Leaderboard #");
	auto& leaderboard = _sim->raceManager->mpCacheLeaderboard;
	for (auto& entry : leaderboard) {
		auto* avatar = entry.car;
		auto* driver = getDriver(avatar);
		if (driver) {
			auto bestLapStr(lapToStr(entry.bestLap));
			log_printf(L"%s ; %s ; \"%s\"", 
				bestLapStr.c_str(), avatar->unixName.c_str(), avatar->driverInfo.name.c_str()
			);
		}
	}

	log_printf(L"");
}

DriverState* AppCheaterDetector::getDriver(CarAvatar* avatar)
{
	auto idriver = _drivers.find(avatar);
	if (idriver != _drivers.end()) {
		return idriver->second;
	}
	return nullptr;
}

DriverState* AppCheaterDetector::getDriver(int id)
{
	for (auto iter : _sim->cars) {
		if (iter->guid == id) {
			return getDriver(iter);
		}
	}
	return nullptr;
}

CarIni* AppCheaterDetector::loadCarIni(const std::wstring& unixName)
{
	log_printf(L"loadCarIni %s", unixName.c_str());

	auto car = new CarIni();
	car->unixName = unixName;
	_carIni.insert(std::pair<const std::wstring, CarIni*>(unixName, car));

	std::wstring path(L"content/cars/");
	path.append(unixName);
	path.append(L"/data/car.ini");

	scoped_udt<INIReader> ini(new_udt<INIReader>(path));
	std::wstring section(L"BASIC");
	std::wstring key(L"TOTALMASS");
	car->mass = ini->getFloat(section, key);

	return car;
}

CarIni* AppCheaterDetector::getCarIni(const std::wstring& unixName)
{
	auto icar = _carIni.find(unixName);
	if (icar != _carIni.end()) {
		return icar->second;
	}
	return nullptr;
}

CarPhysicsState* AppCheaterDetector::getCarState(CarAvatar* avatar) {
	if (avatar->physics) {
		return &avatar->physicsState;
	}
	else if (avatar->netCarStateProvider) {
		if (!avatar->netCarStateProvider->isDisconnected) {
			return &avatar->netCarStateProvider->state;
		}
	}
	return nullptr;
}
