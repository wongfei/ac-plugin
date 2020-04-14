#include "precompiled.h"
#include "AppCustomPhysics.h"

#include "utils/log.h"
#include "utils/common.h"
#include "Timer.h"

#include "Impl/AntirollBar.h"
#include "Impl/BrakeSystem.h"
#include "Impl/BrushSlipProvider.h"
#include "Impl/CarUtils.h"
#include "Impl/Car.h"
#include "Impl/Damper.h"
#include "Impl/Drivetrain.h"
#include "Impl/Drivetrain2WD.h"
#include "Impl/Engine.h"
#include "Impl/Suspension.h"
#include "Impl/SuspensionAxle.h"
#include "Impl/SuspensionML.h"
#include "Impl/SuspensionStrut.h"
#include "Impl/ThermalObject.h"
#include "Impl/Turbo.h"
#include "Impl/TyreUtils.h"
#include "Impl/Tyre.h"
#include "Impl/TyreForces.h"
#include "Impl/TyreModel.h"

#define HOOK_FUNC_RVA(func) hook_create(UT_WSTRING(func), _drva(RVA_##func), &::func)
#define HOOK_FUNC_RVA_2(func) hook_create(UT_WSTRING(func), _drva(RVA_##func), &::func, &_orig_##func)

#define RVA_PhysicsDriveThread_run 1192272

static AppCustomPhysics* s_custom_physics = nullptr;

void PhysicsDriveThread_run(PhysicsDriveThread* pThis)
{
	s_custom_physics->run(pThis);
}

void Car_pollControls(Car* pThis, float dt)
{
	s_custom_physics->pollControls(pThis, dt);
}

AppCustomPhysics::AppCustomPhysics(ACPlugin* plugin) : PluginApp(plugin, L"custom_physics")
{
	log_printf(L"+AppCustomPhysics %p", this);
	s_custom_physics = this;

	log_printf(L"carName=%s", plugin->car->screenName.c_str());
	log_printf(L"suspensionTypeF=%d", (int)plugin->car->suspensionTypeF);
	log_printf(L"suspensionTypeR=%d", (int)plugin->car->suspensionTypeR);
	log_printf(L"torqueModeEx=%d", (int)plugin->car->torqueModeEx);
	log_printf(L"tractionType=%d", (int)plugin->car->drivetrain.tractionType);
	log_printf(L"diffType=%d", (int)plugin->car->drivetrain.diffType);

	#if 0
	ksInitTimerVars();
	HOOK_FUNC_RVA_2(PhysicsDriveThread_run);
	HOOK_FUNC_RVA_2(Car_pollControls);
	addConsoleCommands();
	#endif

	#if 0
		_plugin->car->controlsProvider->ffEnabled = false;
	#endif

	#if 1
	HOOK_FUNC_RVA(AntirollBar_step);
	HOOK_FUNC_RVA(BrakeSystem_step);
	HOOK_FUNC_RVA(BrakeSystem_stepTemps);

	HOOK_FUNC_RVA(BrushTyreModel_solve);
	HOOK_FUNC_RVA(BrushTyreModel_solveV5);
	HOOK_FUNC_RVA(BrushTyreModel_getCFFromSlipAngle);
	HOOK_FUNC_RVA(BrushSlipProvider_getSlipForce);

	HOOK_FUNC_RVA(Car_step);
	HOOK_FUNC_RVA(Car_stepComponents);
	HOOK_FUNC_RVA(Car_updateAirPressure);
	HOOK_FUNC_RVA(Car_updateBodyMass);
	HOOK_FUNC_RVA(Car_calcBodyMass);

	HOOK_FUNC_RVA(Damper_getForce);

	HOOK_FUNC_RVA(Drivetrain_step);
	HOOK_FUNC_RVA(Drivetrain_step2WD);
	HOOK_FUNC_RVA(Drivetrain_stepControllers);
	HOOK_FUNC_RVA(Drivetrain_getInertiaFromWheels);
	HOOK_FUNC_RVA(Drivetrain_getInertiaFromEngine);
	HOOK_FUNC_RVA(Drivetrain_reallignSpeeds);
	HOOK_FUNC_RVA(Drivetrain_accelerateDrivetrainBlock);
	HOOK_FUNC_RVA(Drivetrain_getEngineRPM);

	HOOK_FUNC_RVA(Engine_step);
	HOOK_FUNC_RVA(Engine_stepTurbos);

	HOOK_FUNC_RVA(Suspension_step);
	HOOK_FUNC_RVA(SuspensionAxle_step);
	HOOK_FUNC_RVA(SuspensionML_step);
	HOOK_FUNC_RVA(SuspensionStrut_step);

	HOOK_FUNC_RVA(ThermalObject_step);

	HOOK_FUNC_RVA(Turbo_step);

	HOOK_FUNC_RVA(Tyre_step);
	HOOK_FUNC_RVA(Tyre_addGroundContact);
	HOOK_FUNC_RVA(Tyre_updateAngularSpeed);
	HOOK_FUNC_RVA(Tyre_updateLockedState);
	HOOK_FUNC_RVA(Tyre_stepRotationMatrix);
	HOOK_FUNC_RVA(Tyre_stepThermalModel);
	HOOK_FUNC_RVA(Tyre_stepTyreBlankets);
	HOOK_FUNC_RVA(Tyre_stepGrainBlister);
	HOOK_FUNC_RVA(Tyre_stepFlatSpot);

	HOOK_FUNC_RVA(Tyre_addTyreForcesV10);
	HOOK_FUNC_RVA(Tyre_stepRelaxationLength);
	HOOK_FUNC_RVA(Tyre_getCorrectedD);
	HOOK_FUNC_RVA(Tyre_stepDirtyLevel);
	HOOK_FUNC_RVA(Tyre_stepPuncture);
	HOOK_FUNC_RVA(Tyre_addTyreForceToHub);

	HOOK_FUNC_RVA(SCTM_solve);
	#endif

	writeConsole(strf(L"APP \"%s\" initialized", _appName.c_str()));
}

AppCustomPhysics::~AppCustomPhysics()
{
	log_printf(L"~AppCustomPhysics %p", this);
}

void AppCustomPhysics::run(PhysicsDriveThread* pThis)
{
	while (!*(volatile bool*)&pThis->shuttingDown)
	{
		if (!pThis->isPhysicsInitialized)
		{
			//srand(timeGetTime());
			srand(666);

			if (pThis->setAffinityMask)
			{
				//ksSetCurrentThreadAffinityMask(4ui64);
				//v3 = SetThreadAffinityMask(GetCurrentThread(), v1);
			}

			PhysicsCore* pPhysCore = (PhysicsCore*)(pThis->engine.getCore());
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
			pThis->engine.stepPaused();
			Sleep(10);
		}
		else
		{
			processUserInput();

			int iNumLoops = 0;
			double fGt = ksGetTime() * pThis->timeScale;

			if (fGt - pThis->currentTime > 1000.0)
			{
				log_printf(L"RESET PHYSICS TIMER");
				pThis->currentTime = fGt;
			}

			if (fGt > pThis->currentTime)
			{
				do
				{
					if (pThis->directInput && pThis->useDirectInput)
					{
						pThis->directInput->poll();
						pThis->diCommandManager.step();
					}

					if (_controlMode == EControlMode::Record)
					{
						CarControlsSample sample;
						sample.sampleId = _sampleId;
						sample.currentTime = pThis->currentTime;
						sample.gameTime = fGt;
						sample.controls = _plugin->car->controls;
						_controlSamples.push_back(sample);
						_sampleId++;
					}
					else if (_controlMode == EControlMode::Replay)
					{
						if (_sampleId < _controlSamples.size())
						{
							_plugin->car->controls = _controlSamples[_sampleId].controls;
							_sampleId++;
						}
					}

					double fCurTime = pThis->currentTime + 3.0;
					pThis->currentTime = fCurTime;
					++iNumLoops;

					double fBeginTime = ksGetQPTTime();
					{
						pThis->engine.step(0.003, fCurTime, fGt);
					}
					double fEndTime = ksGetQPTTime();

					double fElapsed = fEndTime - fBeginTime;
					pThis->cpuTimeLocal = (float)fElapsed;

					const double fElapsedLimit = 300;
					double fElapsedX = ((fElapsed * 0.33333334) * 100.0);

					if (fElapsedX >= fElapsedLimit)
						fElapsedX = fElapsedLimit;

					pThis->occupancy.exchange((int)fElapsedX);

					for (auto& h : pThis->evPhysicsStepCompleted.handlers)
					{
						if (h.second)
						{
							h.second(pThis->engine.physicsTime);
						}
					}
				}
				while (fGt > pThis->currentTime);

				if (iNumLoops > 0)
				{
					pThis->lastStepTimestamp = fGt;
				}

				if (iNumLoops > 1)
				{
					log_printf(L"HAD TO LOOP %u times", (unsigned int)iNumLoops);
					if (fGt > 30000.0)
						pThis->physicsLateLoops++;
				}
			}
		}

		pThis->cpuTimeAtomic.exchange(pThis->cpuTimeLocal);
		Sleep(0);
	}
}

void AppCustomPhysics::pollControls(Car* pCar, float dt)
{
	if (!_gameStarted)
	{
		_gameStarted = true;
		if (_controlMode != EControlMode::Default)
		{
			_sim->startGame();
		}
	}

	if (pCar == _plugin->car && _controlMode == EControlMode::Replay)
	{
		// ignore controls when replaying
		return;
	}

	((void(*)(Car*, float))_orig_Car_pollControls)(pCar, dt);
}

void AppCustomPhysics::processUserInput()
{
	if (_cmdRecord || GetAsyncKeyState(VK_OEM_4) & 1) // {
	{
		if (_controlMode == EControlMode::Default)
		{
			if (!_cmdRecord || _recName.empty()) _recName = L"tmp";
			writeConsole(strf(L"start record \"%s\"", _recName.c_str()), true);
			_sampleId = 0;
			_controlSamples.clear();
			_bodyMat = _plugin->car->body->getWorldMatrix(0.0f);
			CarAvatar_teleport(_plugin->carAvatar, _bodyMat);
			resetTelemetry();

			if (_cmdRecord) _sim->console->show(false);
			_recStartTime = ksGetQPTTime();
			_controlMode = EControlMode::Record;
		}
		else if (_controlMode == EControlMode::Record)
		{
			writeConsole(strf(L"stop record \"%s\"", _recName.c_str()), true);
			saveControlSamples(L"record_" + _recName + L".raw");
			saveTelemetry(L"record_" + _recName + L".csv");
			_controlMode = EControlMode::Default;
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
			writeConsole(strf(L"start replay \"%s\"", _recName.c_str()), true);
			loadControlSamples(L"record_" + _recName + L".raw");
			if (_controlSamples.size())
			{
				_plugin->car->controlsProvider->ffEnabled = false;
				CarAvatar_teleport(_plugin->carAvatar, _bodyMat);
				resetTelemetry();

				if (_cmdReplay) _sim->console->show(false);
				_recStartTime = ksGetQPTTime();
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
		writeConsole(strf(L"stop replay \"%s\"", _recName.c_str()), true);
		saveTelemetry(L"replay_" + _recName + L".csv");
		_plugin->car->controlsProvider->ffEnabled = true;
		_controlMode = EControlMode::Default;
	}
}

void AppCustomPhysics::addConsoleCommands()
{
	auto pThis = this;
	{
		std::wstring name(L"rec");
		std::function<bool(std::wstring)> func = [pThis](std::wstring s) {
			if (s.find_first_of(L"rec ") == 0 && s.length() > 4)
			{
				pThis->cmdRecord(s.substr(4));
			}
			return true;
		};
		std::function<std::wstring(void)> help = []() {
			return L"";
		};
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
		std::function<std::wstring(void)> help = []() {
			return L"";
		};
		_sim->console->addCommand(name, &func, &help);
	}
	{
		std::wstring name(L"dumpcar");
		std::function<bool(std::wstring)> func = [pThis](std::wstring s) {
			if (s.compare(L"dumpcar") == 0)
			{
				pThis->cmdDumpCar();
			}
			return true;
		};
		std::function<std::wstring(void)> help = []() {
			return L"";
		};
		_sim->console->addCommand(name, &func, &help);
	}
}

void AppCustomPhysics::cmdRecord(const std::wstring& name)
{
	if (!_cmdRecord && _controlMode == EControlMode::Default)
	{
		_recName = name;
		_cmdRecord = true;
	}
}

void AppCustomPhysics::cmdReplay(const std::wstring& name)
{
	if (!_cmdReplay && _controlMode == EControlMode::Default)
	{
		_recName = name;
		_cmdReplay = true;
	}
}

void AppCustomPhysics::cmdDumpCar()
{
	FILE* fd = fopen("car.raw", "wb");
	if (fd)
	{
		char magic[4] = { 'C','A','R','!' };
		fwrite(magic, sizeof(magic), 1, fd);

		uint64_t carSize = sizeof(Car);
		fwrite(&carSize, sizeof(carSize), 1, fd);
		fwrite(_plugin->car, carSize, 1, fd);

		uint64_t numSusp = _plugin->car->suspensions.size();
		fwrite(&numSusp, sizeof(numSusp), 1, fd);

		for (auto* susp : _plugin->car->suspensions)
		{
			fwrite(susp, sizeof(ISuspension), 1, fd);
		}

		fclose(fd);
		writeConsole(strf(L"dumped!"), true);
	}
}

void AppCustomPhysics::resetTelemetry()
{
	for (auto& chan : _plugin->car->telemetry.channels)
	{
		chan.data.values.clear();
		chan.data.frequency = 200;
	}
}

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

void AppCustomPhysics::saveTelemetry(const std::wstring& filename)
{
	log_printf(L"saveTelemetry filename=%s", filename.c_str());

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
		double elapsed = ksGetQPTTime() - _recStartTime;

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

		const Telemetry& telem = _plugin->car->telemetry;
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

void AppCustomPhysics::saveControlSamples(const std::wstring& filename)
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

void AppCustomPhysics::loadControlSamples(const std::wstring& filename)
{
	log_printf(L"loadControlSamples filename=%s", filename.c_str());

	_controlSamples.clear();
	_sampleId = 0;

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
