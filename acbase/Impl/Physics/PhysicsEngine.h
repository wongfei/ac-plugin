#pragma once

static PhysicsEngine* _instance_PhysicsEngine = nullptr;
static void* _orig_PhysicsEngine_ctor = nullptr;
static void* _orig_PhysicsEngine_step = nullptr;

BEGIN_HOOK_OBJ(PhysicsEngine)
	
	#define RVA_PhysicsEngine_vtable 0x4F5A88
	#define RVA_PhysicsEngine_ctor 2499632
	#define RVA_PhysicsEngine_initLowSpeedFF 2505872
	#define RVA_PhysicsEngine_setSessionInfo 2508128
	#define RVA_PhysicsEngine_setDynamicTempData 1315344
	#define RVA_PhysicsEngine_setWind 2508192
	#define RVA_PhysicsEngine_step 2508640
	#define RVA_PhysicsEngine_stepWind 2511744
	#define RVA_PhysicsEngine_onCollisionCallBack 2506784

	static void _hook()
	{
		HOOK_METHOD_RVA_ORIG(PhysicsEngine, ctor);
		HOOK_METHOD_RVA(PhysicsEngine, initLowSpeedFF);
		HOOK_METHOD_RVA(PhysicsEngine, setSessionInfo);
		HOOK_METHOD_RVA(PhysicsEngine, setDynamicTempData);
		HOOK_METHOD_RVA(PhysicsEngine, setWind);
		HOOK_METHOD_RVA_ORIG(PhysicsEngine, step);
		HOOK_METHOD_RVA(PhysicsEngine, stepWind);
		HOOK_METHOD_RVA(PhysicsEngine, onCollisionCallBack);
	}

	PhysicsEngine* _ctor();
	void _initLowSpeedFF();
	void _setSessionInfo(const SessionInfo& info);
	void _setDynamicTempData(DynamicTempData& data);
	void _setWind(Speed speed, float directionDEG);
	void _step(float dt, double currentTime, double gt);
	void _stepWind(float dt);
	void _onCollisionCallBack(void* userData0, void* shape0, void* userData1, void* shape1, const vec3f& normal, const vec3f& pos, float depth);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

PhysicsEngine* _PhysicsEngine::_ctor()
{
	_instance_PhysicsEngine = this;

	AC_CTOR_THIS_VT(PhysicsEngine);

	if (!g_bCustomPhysics)
	{
		auto orig = ORIG_METHOD(PhysicsEngine, ctor);
		return THIS_CALL(orig)();
	}

	AC_CTOR_UDT(this->wind.speed)();
	AC_CTOR_UDT(this->dynamicTemp)();

	this->allowTyreBlankets = true;
	this->fuelConsumptionRate = 1.0f;
	this->tyreConsumptionRate = 1.0f;
	this->penaltyMode = PenaltyMode::Nothing;
	this->penaltyRules.jumpStartPenaltyMode = JumpStartPenaltyMode::eTeleportToPitMode;
	this->penaltyRules.basePitPenaltyLaps = 3;
	this->ambientTemperature = 26.0f;
	this->roadTemperature = 30.0f;
	this->mechanicalDamageRate = 1.0f;
	this->flatSpotFFGain = 0.05f;
	this->mzLowSpeedReduction.speedKMH = 3.0f;
	this->mzLowSpeedReduction.minValue = 0.5f;
	this->damperGain = 1.0f;
	this->dynamicTemp.baseRoad = 26.0f;
	this->dynamicTemp.baseAir = 26.0f;
	this->isEngineStallEnabled = true;
	this->allowedTyresOut = -1;

	auto ini(new_udt_unique<INIReader>(L"system/cfg/assetto_corsa.ini"));
	if (!ini->ready)
	{
		SHOULD_NOT_REACH_FATAL;
		return this;
	}

	if (ini->hasSection(L"FF_EXPERIMENTAL"))
	{
		bool bEnableGyro = (ini->getInt(L"FF_EXPERIMENTAL", L"ENABLE_GYRO") != 0);
		if (bEnableGyro)
			this->gyroWheelGain = 0.004f;
	}

	this->damperMinValue = ini->getFloat(L"FF_EXPERIMENTAL", L"DAMPER_MIN_LEVEL");
	this->damperGain = ini->getFloat(L"FF_EXPERIMENTAL", L"DAMPER_GAIN");

	int iNumThreads = -1;
	if (ini->hasSection(L"PHYSICS_THREADING"))
	{
		iNumThreads = ini->getInt(L"PHYSICS_THREADING", L"THREADS");
	}

	this->core = new_udt<PhysicsCore>();
	this->physicsTime = ksGetTime();
	this->core->setCollisionCallback(this);
	this->initLowSpeedFF();

	#if 0
	_SYSTEM_INFO SystemInfo = {};
	GetSystemInfo(&SystemInfo);
	int iNumProcessors = (int)SystemInfo.dwNumberOfProcessors;
	bool bUseThreading = false; // TODO: implement

	if (bUseThreading)
	{
		TODO_NOT_IMPLEMENTED;
	}
	#endif

	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _PhysicsEngine::_initLowSpeedFF()
{
	auto ini(new_udt_unique<INIReader>(L"system/cfg/assetto_corsa.ini"));
	if (!ini->ready)
	{
		SHOULD_NOT_REACH_FATAL;
		return;
	}

	if (ini->hasSection(L"LOW_SPEED_FF"))
	{
		this->mzLowSpeedReduction.speedKMH = ini->getFloat(L"LOW_SPEED_FF", L"SPEED_KMH");
		this->mzLowSpeedReduction.minValue = ini->getFloat(L"LOW_SPEED_FF", L"MIN_VALUE");
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _PhysicsEngine::_setSessionInfo(const SessionInfo& info)
{
	this->sessionInfo = info;
	this->core->resetCollisions();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _PhysicsEngine::_setDynamicTempData(DynamicTempData& data)
{
	// ACHTUNG!!! data.temperatureCurve contains random shit
	// RaceManager::initOffline(INIReaderDocuments&) Line 628
	/*
	Curve::addValue(&data.temperatureCurve, 0.0, 0.0);
	int n = (int)ksRand(4.0, 10.0);
	for (int i = 0; i < n; ++i)
		Curve::addValue(&data.temperatureCurve, k * (720000000.0 / (float)n), ksRand(-20.0, 20.0))
	*/

	data.baseAir = tmax(1.0f, data.baseAir);
	data.baseRoad = tmax(1.0f, data.baseRoad);
	this->dynamicTemp = data;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _PhysicsEngine::_setWind(Speed speed, float directionDEG)
{
	vec3f vAxis(0, 1, 0);
	mat44f mxWind = mat44f_createFromAxisAngle(vAxis, -(directionDEG * 0.017453f));

	this->wind.vector = vec3f(&mxWind.M31) * speed.value;
	this->wind.speed = speed;
	this->wind.directionDeg = directionDEG;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

static bool _dump_accum_enabled = true;
static float _dump_accum = 0;

static bool _dump_pre_step = true;
static bool _dump_post_step = false;

static void dump_PhysicsEngine(PhysicsEngine* pe, const std::wstring& prefix)
{
	#if defined(AC_ENABLE_CUSTOM_PHYSICS)
		auto dumpName = L"_custom.json";
	#else
		auto dumpName = L"_orig.json";
	#endif

	auto strFileName = prefix + pe->cars[0]->unixName + dumpName;
	log_printf(L"dump_PhysicsEngine: \"%s\"", strFileName.c_str());

	ac_ostream out;
	out << *pe;
	auto str = out.str();

	std::wofstream fd(strFileName);
	fd << str;
	fd.close();
}

void _PhysicsEngine::_step(float dt, double currentTime, double gt)
{
	if (_dump_accum_enabled)
	{
		_dump_accum += dt;
		if (_dump_accum >= 5.0f)
		{
			_dump_accum_enabled = false;
			_dump_post_step = true;
		}
	}

	// dump state before stepping
	#if defined(AC_DBG_DUMP_STATE)
	if (_dump_pre_step)
	{
		_dump_pre_step = false;
		dump_PhysicsEngine(this, L"_pre_step_");
	}
	#endif

	#if defined(AC_ENABLE_CUSTOM_PHYSICS)

	this->gameTime = gt;
	this->physicsTime = currentTime;
	this->stepCounter++;

	for (auto& h : this->evOnPreStep.handlers)
	{
		if (h.second)
		{
			(h.second)(currentTime);
		}
	}

	this->track->step(dt);
	this->stepWind(dt);

	double t0 = ksGetQPTTime();

	for (auto* pCar : this->cars)
	{
		pCar->stepPreCacheValues(dt);
	}
	
	#if 0
	if (this->pool)
	{
		TODO_NOT_IMPLEMENTED; // multithreaded step
	}
	else
	#endif
	{
		for (auto* pCar : this->cars)
		{
			pCar->step(dt);
		}
	}

	this->physicsCPUTimes.carStep = ksGetQPTTime() - t0;

	this->core->step(dt);

	this->physicsCPUTimes.coreCPUTimes = this->core->getCoreCPUTimes();

	for (auto& h : this->evOnStepCompleted.handlers)
	{
		if (h.second)
		{
			(h.second)(this->physicsTime);
		}
	}

	this->physicsCPUTimes.currentCPU = 0;

	#if 0
	if (PhysicsEngine::isTestMode)
	{
		TODO_NOT_IMPLEMENTED;
	}
	#endif

	#else // ORIG step

	auto orig = ORIG_METHOD(PhysicsEngine, step);
	THIS_CALL(orig)(dt, currentTime, gt);

	#endif

	#if defined(AC_DBG_DUMP_STATE)
	if (_dump_post_step)
	{
		_dump_post_step = false;
		dump_PhysicsEngine(this, L"_post_step_");
	}
	#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _PhysicsEngine::_stepWind(float dt)
{
	if (this->wind.speed.value >= 0.01f)
	{
		float fChange = (float)sin((this->physicsTime - this->sessionInfo.startTimeMS) * 0.0001);
		float fSpeed = this->wind.speed.value * ((fChange * 0.1f) + 1.0f);
		vec3f vWind = this->wind.vector.get_norm() * fSpeed;

		if (isfinite(vWind.x) && isfinite(vWind.y) && isfinite(vWind.z))
		{
			this->wind.vector = vWind;
		}
		else
		{
			SHOULD_NOT_REACH_WARN;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _PhysicsEngine::_onCollisionCallBack(
	void* userData0, void* shape0, 
	void* userData1, void* shape1, 
	const vec3f& normal, const vec3f& pos, float depth)
{
	bool bFlag0 = false;
	bool bFlag1 = false;

	for (auto* pCar : this->cars)
	{
		if (pCar->body == userData0)
			bFlag0 = true;

		if (pCar->body == userData1)
			bFlag1 = true;
	}

	if (!bFlag0 && bFlag1)
	{
		std::swap(userData0, userData1);
		std::swap(shape0, shape1);
	}

	for (auto* pCar : this->cars)
	{
		pCar->onCollisionCallBack(userData0, shape0, userData1, shape1, normal, pos, depth);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
