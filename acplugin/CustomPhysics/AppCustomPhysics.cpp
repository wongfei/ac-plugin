#include "precompiled.h"
#include "AppCustomPhysics.h"
#include "GameHooks.h"
#include "Timer.h"

static AppCustomPhysics* s_custom_physics = nullptr;

inline void rotateCar(Car* pCar, mat44f& mat)
{
	pCar->body->setRotation(mat);
	pCar->fuelTankBody->setRotation(mat);

	for (auto& s : pCar->suspensions)
	{
		s->attach();
	}

	pCar->body->stop(1.0f);
	pCar->fuelTankBody->stop(1.0f);
}

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

	ksInitTimerVars();
	HOOK_FUNC_RVA_2(PhysicsDriveThread_run);
	HOOK_FUNC_RVA_2(Car_pollControls);

	#if 1
	HOOK_FUNC_RVA(Car_step);
	HOOK_FUNC_RVA(Car_stepComponents);

	HOOK_FUNC_RVA(Engine_step);
	HOOK_FUNC_RVA(Engine_stepTurbos);
	HOOK_FUNC_RVA(Turbo_step);

	HOOK_FUNC_RVA(Drivetrain_step);
	HOOK_FUNC_RVA(Drivetrain_stepControllers);
	HOOK_FUNC_RVA(Drivetrain_step2WD);

	HOOK_FUNC_RVA(Damper_getForce);
	HOOK_FUNC_RVA(Suspension_step);
	HOOK_FUNC_RVA(SuspensionAxle_step);
	HOOK_FUNC_RVA(SuspensionML_step);
	//HOOK_FUNC_RVA(SuspensionStrut_step); // TODO

	HOOK_FUNC_RVA(Tyre_step);
	HOOK_FUNC_RVA(Tyre_addGroundContact);
	HOOK_FUNC_RVA(Tyre_addTyreForcesV10);
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
			pThis->directInput = DirectInput::singleton();

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
			Sleep(0xAu);
		}
		else
		{
			if (GetAsyncKeyState(VK_OEM_4) & 1) // {
			{
				if (_controlMode == EControlMode::Default)
				{
					log_printf(L"begin record");
					_sampleId = 0;
					_controlSamples.clear();
					_controlMode = EControlMode::Record;
					_bodyMat = _plugin->car->body->getWorldMatrix(0.0f);
				}
				else if (_controlMode == EControlMode::Record)
				{
					log_printf(L"end record");
					_controlMode = EControlMode::Default;
					saveControlSamples(L"control_samples.raw");
				}
			}

			if (GetAsyncKeyState(VK_OEM_6) & 1) // }
			{
				if (_controlMode == EControlMode::Default)
				{
					log_printf(L"begin replay");
					loadControlSamples(L"control_samples.raw");
					if (_controlSamples.size())
					{
						_plugin->car->controlsProvider->ffEnabled = false;
						float height = _plugin->carAvatar->raceEngineer->getBaseCarHeight();
						vec3f pos = makev(_bodyMat.M41, _bodyMat.M42 - height, _bodyMat.M43);
						_plugin->car->forcePosition(pos, true);
						rotateCar(_plugin->car, _bodyMat);
						_controlMode = EControlMode::Replay;
					}
				}
				else if (_controlMode == EControlMode::Replay)
				{
					log_printf(L"end replay");
					_plugin->car->controlsProvider->ffEnabled = true;
					_controlMode = EControlMode::Default;
				}
			}

			{
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

						if (_controlMode == EControlMode::Replay)
						{
							if (_sampleId < _controlSamples.size())
							{
								_plugin->car->controls = _controlSamples[_sampleId].controls;
								_sampleId++;
							}
							if (_sampleId >= _controlSamples.size())
							{
								log_printf(L"replay done");
								_controlMode = EControlMode::Default;
							}
						}

						double fCurTime = pThis->currentTime + 3.0;
						pThis->currentTime = fCurTime;
						++iNumLoops;

						double fBeginTime = ksGetQPTTime();
						pThis->engine.step(0.003, fCurTime, fGt);
						double fEndTime = ksGetQPTTime();

						double fElapsed = fEndTime - fBeginTime;
						pThis->cpuTimeLocal = (float)fElapsed;

						double fElapsedLimit = 300;
						double fElapsedX = (signed int)(float)((float)(fElapsed * 0.33333334) * 100.0);

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
		}

		pThis->cpuTimeAtomic.exchange(pThis->cpuTimeLocal);
		Sleep(0);
	}
}

void AppCustomPhysics::pollControls(Car* pCar, float dt)
{
	if (!_gameStarted && _controlMode != EControlMode::Default)
	{
		_gameStarted = true;
		_sim->startGame();
	}

	if (pCar == _plugin->car && _controlMode == EControlMode::Replay)
	{
		return;
	}

	((void(*)(Car*, float))_orig_Car_pollControls)(pCar, dt);
}

void AppCustomPhysics::saveControlSamples(const wchar_t* filename)
{
	log_printf(L"saveControlSamples filename=%s samples=%u sampleSize=%u", 
		filename, (unsigned int)_controlSamples.size(), (unsigned int)sizeof(CarControlsSample));

	if (!_controlSamples.size())
	{
		log_printf(L"ERR: no data samples");
	}
	else
	{
		FILE* fd = NULL;
		_wfopen_s(&fd, filename, L"wb");
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

void AppCustomPhysics::loadControlSamples(const wchar_t* filename)
{
	log_printf(L"loadControlSamples filename=%s", filename);

	_controlSamples.clear();
	_sampleId = 0;

	FILE* fd = NULL;
	_wfopen_s(&fd, filename, L"rb");
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
