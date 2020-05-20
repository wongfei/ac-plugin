#pragma once

BEGIN_HOOK_OBJ(Track)
	
	#define RVA_Track_vtable 0x4F7AB0
	#define RVA_Track_ctor 2584832
	#define RVA_Track_initDynamicTrack 2589440
	#define RVA_Track_step 2592032
	#define RVA_Track_rayCast 2591664
	#define RVA_Track_rayCastWithRayCaster 2591872
	#define RVA_Track_addSurface 2588240

	static void _hook()
	{
		HOOK_METHOD_RVA(Track, ctor);
		HOOK_METHOD_RVA(Track, initDynamicTrack);
		HOOK_METHOD_RVA(Track, step);
		HOOK_METHOD_RVA(Track, rayCast);
		HOOK_METHOD_RVA(Track, rayCastWithRayCaster);
	}

	Track* _ctor(PhysicsEngine* pe, const std::wstring& iname, const std::wstring& config);
	void _initDynamicTrack();
	void _step(float dt);
	bool _rayCast(const vec3f& org, const vec3f& dir, RayCastResult* result, float length);
	bool _rayCastWithRayCaster(const vec3f& org, const vec3f& dir, RayCastResult* result, float length, IRayCaster* rayCaster);
	uint64_t _addSurface(const std::wstring& iname, float* vertices, int numVertices, unsigned short* indices, int indexCount, const SurfaceDef& surfaceDef, unsigned int sector_id);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

Track* _Track::_ctor(PhysicsEngine* pe, const std::wstring& iname, const std::wstring& config)
{
	AC_CTOR_THIS_VT(Track);

	if (pe->track)
		pe->track->dtor();

	pe->track = this;

	this->ksPhysics = pe;
	this->name = iname;
	this->config = config;

	this->dynamicTrack.sessionStartGrip = 1.0f;
	this->dynamicTrack.baseGrip = 1.0f;
	this->dynamicTrack.randomGrip = 0.01f;
	this->dynamicTrack.gripPerLap = 0.1f;
	this->dynamicGripLevel = 1.0f;

	this->dataFolder = std::wstring(L"content/tracks/") + iname;
	if (!config.empty())
		this->dataFolder += (L"/" + config);

	this->worldMatrix.M11 = 1.0f;
	this->worldMatrix.M22 = 1.0f;
	this->worldMatrix.M33 = 1.0f;
	this->worldMatrix.M44 = 1.0f;

	this->initDynamicTrack();
	this->drsMamanger.reset(new_udt<DRSManager>(*this->ksPhysics));

	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Track::_initDynamicTrack()
{
	this->dynamicTrack.enabled = false;

	std::wstring strIni(L"cfg/race.ini");
	auto ini(new_udt_unique<INIReaderDocuments>(&strIni, false));
	if (!ini->ready)
		return;

	if (!ini->hasSection(L"DYNAMIC_TRACK"))
		return;

	this->dynamicTrack.enabled = true;

	float fLapGain = ini->getFloat(L"DYNAMIC_TRACK", L"LAP_GAIN");
	if (fLapGain > 0.0f)
		this->dynamicTrack.gripPerLap = 0.01f / fLapGain;
	else
		this->dynamicTrack.gripPerLap = 0.0f;

	this->dynamicTrack.sessionStartGrip = ini->getFloat(L"DYNAMIC_TRACK", L"SESSION_START") * 0.01f;
	if (this->dynamicTrack.sessionStartGrip < 0.85f)
		this->dynamicTrack.sessionStartGrip = 0.85f;

	this->dynamicTrack.randomGrip = ini->getFloat(L"DYNAMIC_TRACK", L"RANDOMNESS") * 0.01f;
	this->dynamicTrack.sessionTransfer = ini->getFloat(L"DYNAMIC_TRACK", L"SESSION_TRANSFER") * 0.01f;

	float fRnd = ((((float)rand() * 0.000030518509f) * 2.0f) - 1.0f);
	this->dynamicTrack.baseGrip = this->dynamicTrack.sessionStartGrip + (this->dynamicTrack.randomGrip * fRnd);

	// TODO: check grip transfer
	auto pThis = this;
	this->ksPhysics->evOnNewSessionPhysics.add(this, [pThis](const SessionInfo& si)
	{
		auto& dt = pThis->dynamicTrack;
		float fGripTransfer = 0.0f;

		if (si.index) // next session
		{
			fGripTransfer = (pThis->dynamicGripLevel - dt.sessionStartGrip) * dt.sessionTransfer;
		}

		float fRnd = ((((float)rand() * 0.000030518509f) * 2.0f) - 1.0f);
		dt.baseGrip = dt.sessionStartGrip + fGripTransfer + (dt.randomGrip * fRnd);

		log_printf(L"evOnNewSessionPhysics: index=%d startGrip=%f transfer=%f baseGrip=%f dynamicGrip=%f", 
			si.index, dt.sessionStartGrip, fGripTransfer, dt.baseGrip, pThis->dynamicGripLevel);

		GUARD_DEBUG(dt.baseGrip >= 0.85f && dt.baseGrip <= 1.0f);
		dt.baseGrip = tclamp(dt.baseGrip, 0.85f, 1.0f); // TODO: be safe?
	});
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Track::_step(float dt)
{
	if (this->dynamicTrack.enabled)
	{
		int iLapCount = 0;
		for (auto* pCar : this->ksPhysics->cars)
		{
			iLapCount += pCar->transponder.lapCount;
		}

		float fGrip = iLapCount * this->dynamicTrack.gripPerLap + this->dynamicTrack.baseGrip;
		fGrip = tclamp(fGrip, 0.85f, 1.0f);

		this->dynamicGripLevel = fGrip;
	}
	else if (!this->dynamicTrack.isExternal)
	{
		this->dynamicGripLevel = 1.0f;
	}

	if (this->aiSplineRecorder)
		this->aiSplineRecorder->step(dt);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool _Track::_rayCast(const vec3f& org, const vec3f& dir, RayCastResult* result, float length)
{
	auto hit = this->ksPhysics->getCore()->rayCast(org, dir, length);
	if (hit.hasContact)
	{
		result->pos = hit.pos;
		result->normal = hit.normal;
		result->hasHit = hit.hasContact;
		result->collisionObject = hit.collisionObject;
		result->surfaceDef = (SurfaceDef*)hit.collisionObject->getUserPointer();
	}
	return hit.hasContact;
}

bool _Track::_rayCastWithRayCaster(const vec3f& org, const vec3f& dir, RayCastResult* result, float length, IRayCaster* rayCaster)
{
	auto hit = rayCaster->rayCast(org, dir);
	if (hit.hasContact)
	{
		result->pos = hit.pos;
		result->normal = hit.normal;
		result->hasHit = hit.hasContact;
		result->collisionObject = hit.collisionObject;
		result->surfaceDef = (SurfaceDef*)hit.collisionObject->getUserPointer();
	}
	return hit.hasContact;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

uint64_t _Track::_addSurface(const std::wstring& iname, float* vertices, int numVertices, unsigned short* indices, int indexCount, const SurfaceDef& surfaceDef, unsigned int sector_id)
{
	TODO_NOT_IMPLEMENTED;
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
