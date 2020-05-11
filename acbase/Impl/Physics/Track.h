#pragma once

BEGIN_HOOK_OBJ(Track)
	
	#define RVA_Track_step 2592032
	#define RVA_Track_rayCast 2591664
	#define RVA_Track_rayCastWithRayCaster 2591872
	#define RVA_Track_addSurface 2588240

	static void _hook()
	{
		HOOK_METHOD_RVA(Track, step);
		HOOK_METHOD_RVA(Track, rayCast);
		HOOK_METHOD_RVA(Track, rayCastWithRayCaster);
	}

	void _step(float dt);
	bool _rayCast(const vec3f& org, const vec3f& dir, RayCastResult* result, float length);
	bool _rayCastWithRayCaster(const vec3f& org, const vec3f& dir, RayCastResult* result, float length, IRayCaster* rayCaster);
	uint64_t _addSurface(const std::wstring& iname, float* vertices, int numVertices, unsigned short* indices, int indexCount, const SurfaceDef& surfaceDef, unsigned int sector_id);

END_HOOK_OBJ()

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
