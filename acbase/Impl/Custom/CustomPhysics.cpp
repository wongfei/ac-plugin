#include "precompiled.h"
#include "CustomPhysics.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

// tested on:
// ver 1.16.3
// build 2426371

#define AC_ENABLE_CUSTOM_PHYSICS
#define AC_DUMP_CAR_STEP0

#include "Game/Sim.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

#include "SDK/ac_ser.h"
#include "Utils/Timer.h"
#include "Utils/Mathlib.h"
#include "Utils/Curve.h"
#include "Utils/INIReader.h"
#include "Utils/PIDController.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

#define dSINGLE
#include "ode/ode.h"
#include "SDK/ac_ode.h"

#include "Physics/InternalsODE.h"
#include "Physics/CollisionMeshODE.h"
#include "Physics/RigidBodyODE.h"
#include "Physics/JointODE.h"

#include "Physics/PhysicsCore.h"
#include "Physics/PhysicsEngine.h"

#include "Physics/Track.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

static void* _orig_Car_step = nullptr;

#include "Car/Car.h"
#include "Car/CarUtils.h"

#include "Car/CarColliderManager.h"
#include "Car/DynamicController.h"
#include "Car/ThermalObject.h"
#include "Car/BrakeSystem.h"

#include "Car/GearChanger.h"
#include "Car/Drivetrain.h"
#include "Car/Drivetrain2WD.h"
#include "Car/Engine.h"
#include "Car/Turbo.h"

#include "Car/Damper.h"
#include "Car/Suspension.h"
#include "Car/SuspensionStrut.h"
#include "Car/SuspensionAxle.h"
#include "Car/SuspensionML.h"
#include "Car/HeaveSpring.h"
#include "Car/AntirollBar.h"

#include "Car/TyreUtils.h"
#include "Car/Tyre.h"
#include "Car/TyreForces.h"
#include "Car/TyreForcesLegacy.h"
#include "Car/TyreModel.h"
#include "Car/TyreThermalModel.h"
#include "Car/BrushTyreModel.h"
#include "Car/BrushSlipProvider.h"

#include "Car/SlipStream.h"
#include "Car/AeroMap.h"
#include "Car/Wing.h"
#include "Car/DynamicWingController.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

static CustomPhysics* s_custom_physics = nullptr;

#define RVA_PhysicsDriveThread_run 1192272
#define RVA_Car_pollControls 2575984

static void* _orig_PhysicsDriveThread_run = nullptr;
static void* _orig_Car_pollControls = nullptr;

void PhysicsDriveThread_run(PhysicsDriveThread* pThis)
{
	s_custom_physics->run(pThis);
}

void Car_pollControls(Car* pThis, float dt)
{
	s_custom_physics->pollControls(pThis, dt);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CustomPhysics::CustomPhysics()
{
	log_printf(L"+CustomPhysics %p", this);
	s_custom_physics = this;
}

CustomPhysics::~CustomPhysics()
{
	log_printf(L"~CustomPhysics %p", this);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomPhysics::printCarInfo(CarAvatar* car)
{
	log_printf(L"");
	log_printf(L"carName=%s", car->physics->screenName.c_str());
	log_printf(L"unixName=%s", car->physics->unixName.c_str());
	log_printf(L"tyreVersion=%d", (int)car->physics->tyres[0].modelData.version);
	log_printf(L"suspensionTypeF=%d", (int)car->physics->suspensionTypeF);
	log_printf(L"suspensionTypeR=%d", (int)car->physics->suspensionTypeR);
	log_printf(L"torqueModeEx=%d", (int)car->physics->torqueModeEx);
	log_printf(L"tractionType=%d", (int)car->physics->drivetrain.tractionType);
	log_printf(L"diffType=%d", (int)car->physics->drivetrain.diffType);
	log_printf(L"numWings=%d", (int)car->physics->aeroMap.wings.size());
	log_printf(L"");

	#if 0
	{
		std::wstring data = L"content/cars/" + car->physics->unixName + L"/data/";

		//auto ini(new_udt_unique<INIReader>(data + L"car.ini"));
		//log_printf(L"mass=%.3f", (float)ini->getFloat(L"BASIC", L"TOTALMASS"));

		ini_dump(data + L"aero.ini");
		ini_dump(data + L"brakes.ini");
		ini_dump(data + L"car.ini");
		ini_dump(data + L"drivetrain.ini");
		ini_dump(data + L"engine.ini");
		ini_dump(data + L"suspensions.ini");
		ini_dump(data + L"tyres.ini");
	}
	#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomPhysics::installHooks()
{
	log_printf(L"installHooks");

	#if 1
	ksInitTimerVars();
	HOOK_FUNC_RVA_ORIG(PhysicsDriveThread_run);
	HOOK_FUNC_RVA_ORIG(Car_pollControls);
	HOOK_OBJ(Sim);
	//addConsoleCommands();
	#endif

	#if !defined(AC_ENABLE_CUSTOM_PHYSICS)

		#if defined(AC_DUMP_CAR_STEP0)
			HOOK_METHOD_RVA_ORIG(Car, step);
		#endif

	#else

		#if 1
		HOOK_FUNC_RVA(mat44f_createFromAxisAngle);
		HOOK_OBJ(Curve);
		HOOK_OBJ(INIReader);
		HOOK_OBJ(PIDController);
		#endif

		#if 1
		HOOK_OBJ(PhysicsEngine);
		HOOK_OBJ(PhysicsCore);
		HOOK_OBJ(CollisionMeshODE);
		HOOK_OBJ(RigidBodyODE);
		HOOK_OBJ(Track);
		#endif

		#if 1
		HOOK_OBJ(Car);
		HOOK_OBJ(CarColliderManager);
		HOOK_OBJ(DynamicController);
		HOOK_OBJ(ThermalObject);
		HOOK_OBJ(BrakeSystem);

		HOOK_OBJ(GearChanger);
		HOOK_OBJ(Drivetrain);
		HOOK_OBJ(Engine);
		HOOK_OBJ(Turbo);

		HOOK_OBJ(Damper);
		HOOK_OBJ(Suspension); // DoubleWishbone
		HOOK_OBJ(SuspensionStrut);
		HOOK_OBJ(SuspensionAxle);
		HOOK_OBJ(SuspensionML);
		HOOK_OBJ(HeaveSpring);
		HOOK_OBJ(AntirollBar);

		HOOK_OBJ(Tyre);
		HOOK_OBJ(SCTM);
		HOOK_OBJ(TyreThermalModel);
		HOOK_OBJ(BrushSlipProvider);
		HOOK_OBJ(BrushTyreModel);

		HOOK_OBJ(SlipStream);
		HOOK_OBJ(AeroMap);
		HOOK_OBJ(Wing);
		HOOK_OBJ(DynamicWingController);
		#endif

	#endif // AC_ENABLE_CUSTOM_PHYSICS

	hook_enable();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomPhysics::run(PhysicsDriveThread* pThis)
{
	_sim = _Sim_instance;
	GUARD(_sim);

	_car = _sim->getCar(0);
	GUARD(_car);

	#if 0
		_car->physics->controlsProvider->ffEnabled = false;
	#endif

	#if 0
	auto pCar = plugin->sim->addCar(plugin->car->unixName, plugin->car->configName, plugin->carAvatar->currentSkin);
	
	DriverInfo di = plugin->carAvatar->driverInfo;
	di.name = L"test";
	pCar->setDriverInfo(di);
	pCar->setSpawnPositionIndex(L"PIT", 2);
	pCar->goToSpawnPosition(L"PIT");
	pCar->resetTimeTransponder();
	pCar->armFirstLap();

	_aiDriver = new_udt<AIDriver>(pCar->physics);
	_aiDriver->currentSessionInfo.type = SessionType::Qualify;
	pCar->setControlsProvider(_aiDriver);

	//plugin->sim->raceManager->carsRealTimePosition
	plugin->sim->raceManager->resetCurrentLaps();
	plugin->sim->raceManager->resetMandatoryPit();
	#endif

	while (!(*(volatile bool*)&pThis->shuttingDown))
	{
		if (!pThis->isPhysicsInitialized)
		{
			//srand(timeGetTime());
			srand(RND_SEED);

			if (pThis->setAffinityMask)
			{
				//ksSetCurrentThreadAffinityMask(4);
				SetThreadAffinityMask(GetCurrentThread(), 4);
			}

			PhysicsCore* pPhysCore = (PhysicsCore*)(pThis->engine->getCore());
			pPhysCore->initMultithreading_impl();
			pThis->isPhysicsInitialized = true;
		}

		if (!pThis->directInput && pThis->useDirectInput)
		{
			pThis->directInput = DirectInput::singleton();
		}

		if (pThis->isPaused)
		{
			if (pThis->directInput && pThis->useDirectInput)
			{
				pThis->directInput->poll();
				pThis->diCommandManager.step();
			}

			pThis->occupancy.exchange(0);
			pThis->cpuTimeLocal = 0.0;
			pThis->engine->stepPaused();
			Sleep(10);
		}
		else
		{
			processCustomInput();

			switch (_controlMode)
			{
				case EControlMode::Default:
				case EControlMode::Record:
					stepNormal(pThis);
					break;

				case EControlMode::Replay:
					stepReplay(pThis);
					break;
			}
		}

		pThis->cpuTimeAtomic.exchange(pThis->cpuTimeLocal);

		if (_controlMode != EControlMode::Replay)
			Sleep(0);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomPhysics::stepNormal(PhysicsDriveThread* pThis)
{
	double fGt = ksGetTime() * pThis->timeScale;
	int iNumLoops = 0;

	if (fGt - pThis->currentTime > 1000.0)
	{
		log_printf(L"RESET PHYSICS TIMER");
		pThis->currentTime = fGt;
	}

	if (fGt > pThis->currentTime)
	{
		do
		{
			double fCurTime = pThis->currentTime + 3.0;

			if (pThis->directInput && pThis->useDirectInput)
			{
				pThis->directInput->poll();
				pThis->diCommandManager.step();
			}

			if (_car && _controlMode == EControlMode::Record)
			{
				CarControlsSample sample;
				sample.sampleId = _sampleId;
				sample.stepCounter = pThis->engine->stepCounter;
				sample.currentTime = fCurTime;
				sample.gameTime = fGt;
				sample.controls = _car->physics->controls;
				_controlSamples.push_back(sample);

				if (_sampleId == 0)
				{
					srand(RND_SEED);
					_recStartTime = ksGetQPTTime();

					_bodyMat = _car->physics->body->getWorldMatrix(0.0f);
					CarAvatar_teleport(_car, _bodyMat);
					resetTelemetry();

					printMatrix(_car->physics->body->getWorldMatrix(0.0f));
				}

				_sampleId++;
			}

			stepPhysicsEngine(pThis, fCurTime, fGt);
			++iNumLoops;
		} 
		while (fGt > pThis->currentTime);

		if (iNumLoops > 1)
		{
			//log_printf(L"HAD TO LOOP %u times", (unsigned int)iNumLoops);
			if (fGt > 30000.0)
				pThis->physicsLateLoops++;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomPhysics::stepReplay(PhysicsDriveThread* pThis)
{
	if (_controlMode != EControlMode::Replay) 
		return;

	if (_sampleId >= _controlSamples.size())
		return;

	double t0 = ksGetQPTTime();
	const auto& sample = _controlSamples[_sampleId];

	if (_car)
	{
		_car->physics->controls = sample.controls;

		if (_sampleId == 0)
		{
			srand(RND_SEED);
			_recStartTime = t0;

			pThis->engine->stepCounter = (unsigned int)sample.stepCounter;
			pThis->currentTime = sample.currentTime;
			pThis->engine->gameTime = sample.gameTime;

			CarAvatar_teleport(_car, _bodyMat);
			resetTelemetry();

			printMatrix(_car->physics->body->getWorldMatrix(0.0f));
		}
	}

	stepPhysicsEngine(pThis, sample.currentTime, sample.gameTime);
	_sampleId++;

	if (_sampleId < _controlSamples.size())
	{
		const auto& nextSample = _controlSamples[_sampleId];
		double dur = nextSample.gameTime - sample.gameTime;

		for (;;)
		{
			double t1 = ksGetQPTTime();
			if ((t1 - t0) >= dur)
				break;
			else
				Sleep(0);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomPhysics::stepPhysicsEngine(PhysicsDriveThread* pThis, double fCurTime, double fGt)
{
	double fBeginTime = ksGetQPTTime();
	{
		pThis->currentTime = fCurTime;

		pThis->engine->step(0.003, fCurTime, fGt);
		// modifies:
		// pThis->engine.physicsTime = fCurTime;
		// pThis->engine.gameTime = fGt;

		pThis->lastStepTimestamp = fGt;
	}
	double fEndTime = ksGetQPTTime();

	float fElapsed = (float)(fEndTime - fBeginTime);
	pThis->cpuTimeLocal = fElapsed;

	float fOccupancy = tmin((fElapsed * 0.3333333f) * 100.0f, 300.0f);
	pThis->occupancy.exchange((int)fOccupancy);

	for (auto& h : pThis->evPhysicsStepCompleted.handlers)
	{
		if (h.second)
		{
			h.second(pThis->engine->physicsTime);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomPhysics::pollControls(Car* pCar, float dt)
{
	if (!_gameStarted)
	{
		_gameStarted = true;
		if (_controlMode != EControlMode::Default)
		{
			_sim->startGame();
		}
	}

	if (_controlMode == EControlMode::Replay && (_car && _car->physics == pCar))
	{
		// ignore controls when replaying
		return;
	}

	#if 0
		((void(*)(Car*, float))_orig_Car_pollControls)(pCar, dt);
	#else
		(pCar->*(xcast<void (Car::*)(float)>(_orig_Car_pollControls)))(dt);
	#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomPhysics::processCustomInput()
{
	if (!_car)
		return;

	if (_cmdRecord || GetAsyncKeyState(VK_OEM_4) & 1) // {
	{
		if (_controlMode == EControlMode::Default)
		{
			if (_cmdRecord) _sim->console->show(false);
			if (!_cmdRecord || _recName.empty()) _recName = L"tmp";
			//writeConsole(strf(L"start record \"%s\"", _recName.c_str()), true);

			_sampleId = 0;
			_controlSamples.clear();
			_controlMode = EControlMode::Record;
		}
		else if (_controlMode == EControlMode::Record)
		{
			_controlMode = EControlMode::Default;
			_recStopTime = ksGetQPTTime();

			//writeConsole(strf(L"stop record \"%s\"", _recName.c_str()), true);
			printMatrix(_car->physics->body->getWorldMatrix(0.0f));
			saveControlSamples(_recName + L".input");
			saveTelemetry(_recName + L"_orig_telem.csv");
		}
		_cmdRecord = false;
		_cmdReplay = false;
	}

	bool stopReplay = false;

	if (_cmdReplay || GetAsyncKeyState(VK_OEM_6) & 1) // }
	{
		if (_controlMode == EControlMode::Default)
		{
			if (!_cmdReplay || _recName.empty()) _recName = L"tmp";
			//writeConsole(strf(L"start replay \"%s\"", _recName.c_str()), true);

			_sampleId = 0;
			loadControlSamples(_recName + L".input");
			
			if (_controlSamples.size())
			{
				if (_cmdReplay) _sim->console->show(false);
				_car->physics->controlsProvider->ffEnabled = false;
				_controlMode = EControlMode::Replay;
			}
		}
		else if (_controlMode == EControlMode::Replay)
		{
			stopReplay = true;
		}
		_cmdRecord = false;
		_cmdReplay = false;
	}

	if ((_controlMode == EControlMode::Replay) && (stopReplay || (_sampleId >= _controlSamples.size())))
	{
		_controlMode = EControlMode::Default;
		_recStopTime = ksGetQPTTime();

		//writeConsole(strf(L"stop replay \"%s\"", _recName.c_str()), true);
		printMatrix(_car->physics->body->getWorldMatrix(0.0f));
		saveTelemetry(_recName + L"_replay_telem.csv");
		_car->physics->controlsProvider->ffEnabled = true;
	}

	#if 0
	if (GetAsyncKeyState(VK_OEM_MINUS) & 1)
	{
		_aiEnabled = !_aiEnabled;
		writeConsole(strf(L"aiEnabled=%d", (int)_aiEnabled), true);
		if (_aiEnabled)
		{
			if (!_playerControls)
				_playerControls = _plugin->car->controlsProvider;

			if (_aiDriver)
				del_udt(_aiDriver);

			_aiDriver = new_udt<AIDriver>(_plugin->car);
			//_aiDriver->currentSessionInfo.type = SessionType::Qualify;

			_plugin->carAvatar->setControlsProvider(_aiDriver);
			//_plugin->car->setControllerProvider(_aiDriver);
		}
		else
		{
			_plugin->carAvatar->setControlsProvider(_playerControls);
		}
	}
	#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomPhysics::addConsoleCommands()
{
	auto pThis = this;
	{
		std::wstring name(L"rname");
		std::function<bool(std::wstring)> func = [pThis](std::wstring s) {
			if (s.find_first_of(L"rname ") == 0 && s.length() > 6)
			{
				pThis->_recName = s.substr(6);
			}
			return true;
		};
		std::function<std::wstring(void)> help = []() { return L""; };
		_sim->console->addCommand(name, &func, &help);
	}
	{
		std::wstring name(L"rec");
		std::function<bool(std::wstring)> func = [pThis](std::wstring s) {
			if (s.find_first_of(L"rec ") == 0 && s.length() > 4)
			{
				pThis->cmdRecord(s.substr(4));
			}
			return true;
		};
		std::function<std::wstring(void)> help = []() { return L""; };
		_sim->console->addCommand(name, &func, &help);
	}
	{
		std::wstring name(L"rep");
		std::function<bool(std::wstring)> func = [pThis](std::wstring s) {
			if (s.find_first_of(L"rep ") == 0 && s.length() > 4)
			{
				pThis->cmdReplay(s.substr(4));
			}
			return true;
		};
		std::function<std::wstring(void)> help = []() { return L""; };
		_sim->console->addCommand(name, &func, &help);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomPhysics::cmdRecord(const std::wstring& name)
{
	if (!_cmdRecord && _controlMode == EControlMode::Default)
	{
		_recName = name;
		_cmdRecord = true;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomPhysics::cmdReplay(const std::wstring& name)
{
	if (!_cmdReplay && _controlMode == EControlMode::Default)
	{
		_recName = name;
		_cmdReplay = true;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomPhysics::resetTelemetry()
{
	_car->physics->telemetry.isEnabled = true;

	for (auto& chan : _car->physics->telemetry.channels)
	{
		chan.data.values.clear();
		chan.data.frequency = 200;
		chan.lastTickTime = 0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

static const wchar_t* TelemetryUnitsNames[] = 
{
	L"m",
	L"C",
	L"G",
	L"rad/s",
	L"",
	L"m/s",
	L"bar",
	L"MS",
	L"",
	L"%",
	L"W",
	L"vmq",
	L"V",
	L"NM",
	L"rad",
	L"N",
	L"mm",
	L"deg",
	L"fkg",
	L"rpm",
};

void CustomPhysics::saveTelemetry(const std::wstring& filename)
{
	double elapsed = _recStopTime - _recStartTime;
	//writeConsole(strf(L"saveTelemetry filename=%s elapsed=%f", filename.c_str(), elapsed), true);

	//std::wstring tmp(filename + L"_ac");
	//_plugin->car->telemetry.save(&tmp);

	#if 1
	FILE* fd = NULL;
	_wfopen_s(&fd, filename.c_str(), L"w");
	if (!fd)
	{
		log_printf(L"ERR: fopen failed");
	}
	else
	{
		fwprintf(fd, L"\"Format\",\"MoTeC CSV File\",,,\"Workbook\",\"\"\n");
		fwprintf(fd, L"\"Venue\",\"%s\",,,\"Worksheet\",\"\"\n", filename.c_str());
		fwprintf(fd, L"\"Vehicle\",\"test\",,,\"Vehicle Desc\",\"\"\n");
		fwprintf(fd, L"\"Driver\",\"test\",,,\"Engine ID\",\"\"\n");
		fwprintf(fd, L"\"Device\",\"ADL\"\n");
		fwprintf(fd, L"\"Comment\",\"Tires: test\",,,\"Session\",\"PRACTICE\"\n");
		fwprintf(fd, L"\"Log Date\",\"05.12.2018\",,,\"Origin Time\",\"0.000\",\"s\"\n");
		fwprintf(fd, L"\"Log Time\",\"17:42:00\",,,\"Start Time\",\"0.000\",\"s\"\n");
		fwprintf(fd, L"\"Sample Rate\",\"200.000\",\"Hz\",,\"End Time\",\"%.3f\",\"s\"\n", elapsed);
		fwprintf(fd, L"\"Duration\",\"%.3f\",\"s\",,\"Start Distance\",\"0.0\",\"m\"\n", elapsed);
		fwprintf(fd, L"\"Range\",\"Lap 1\",,,\"End Distance\",\"9999.9\",\"m\"\n");

		fwprintf(fd, L"\n\n");

		const Telemetry& telem = _car->physics->telemetry;
		const size_t numChannels = telem.channels.size();
		size_t numSamples = numChannels > 0 ? telem.channels[0].data.values.size() : 0;

		for (size_t chanId = 0; chanId < numChannels; ++chanId)
		{
			const auto& chan = telem.channels[chanId];
			fwprintf(fd, L"\"%S\"", chan.name.c_str());
			if (chanId + 1 < numChannels) fwprintf(fd, L",");

			numSamples = tmin<size_t>(numSamples, chan.data.values.size());
		}

		fwprintf(fd, L"\n");

		for (size_t chanId = 0; chanId < numChannels; ++chanId)
		{
			const auto& chan = telem.channels[chanId];
			fwprintf(fd, L"\"%s\"", TelemetryUnitsNames[(int)chan.data.units]);
			if (chanId + 1 < numChannels) fwprintf(fd, L",");
		}

		fwprintf(fd, L"\n\n");

		for (size_t sampleId = 0; sampleId < numSamples; ++sampleId)
		{
			for (size_t chanId = 0; chanId < numChannels; ++chanId)
			{
				const auto& chan = telem.channels[chanId];
				fwprintf(fd, L"\"%.3f\"", chan.data.values[sampleId]);
				if (chanId + 1 < numChannels) fwprintf(fd, L",");
			}
			fwprintf(fd, L"\n");
		}

		fclose(fd);
	}
	#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomPhysics::saveControlSamples(const std::wstring& filename)
{
	log_printf(L"saveControlSamples filename=%s samples=%u sampleSize=%u",
	   filename.c_str(), (unsigned int)_controlSamples.size(), (unsigned int)sizeof(CarControlsSample));

	if (!_controlSamples.size())
	{
		log_printf(L"ERR: no data samples");
	}
	else
	{
		FILE* fd = NULL;
		_wfopen_s(&fd, filename.c_str(), L"wb");
		if (!fd)
		{
			log_printf(L"ERR: fopen failed");
		}
		else
		{
			CarControlsRawHeader h;
			h.sampleCount = (uint32_t)_controlSamples.size();
			h.sampleSize = (uint32_t)sizeof(_controlSamples[0]);
			h.bodyMat = _bodyMat;

			fwrite(&h, sizeof(h), 1, fd);
			fwrite(&_controlSamples[0], h.sampleCount * h.sampleSize, 1, fd);
			fclose(fd);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomPhysics::loadControlSamples(const std::wstring& filename)
{
	log_printf(L"loadControlSamples filename=%s", filename.c_str());

	_controlSamples.clear();

	FILE* fd = NULL;
	_wfopen_s(&fd, filename.c_str(), L"rb");
	if (!fd)
	{
		log_printf(L"ERR: fopen failed");
	}
	else
	{
		CarControlsRawHeader h;
		if (fread(&h, sizeof(h), 1, fd) != 1)
		{
			log_printf(L"ERR: can't read header");
		}
		else
		{
			if (h.sampleSize != sizeof(CarControlsSample))
			{
				log_printf(L"ERR: invalid sample size: %u", (unsigned int)h.sampleSize);
			}
			else
			{
				if (!h.sampleCount)
				{
					log_printf(L"ERR: invalid sample count");
				}
				else
				{
					_bodyMat = h.bodyMat;
					_controlSamples.resize(h.sampleCount);

					if (fread(&_controlSamples[0], h.sampleCount * h.sampleSize, 1, fd) != 1)
					{
						log_printf(L"ERR: invalid data size");
						_controlSamples.clear();
					}
					else
					{
						log_printf(L"loaded %u samples", (unsigned int)_controlSamples.size());
					}
				}
			}
		}

		fclose(fd);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomPhysics::printMatrix(const mat44f& mat)
{
	const float* m = &mat.M11;
	for (int i = 0; i < 4; ++i)
	{
		log_printf(L"%.4f %.4f %.4f %.4f", m[0], m[1], m[2], m[3]);
		m += 4;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
