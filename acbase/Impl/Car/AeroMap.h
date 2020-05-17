#pragma once

BEGIN_HOOK_OBJ(AeroMap)

	#define RVA_AeroMap_init 2841760
	#define RVA_AeroMap_loadINI 2841936
	#define RVA_AeroMap_step 2847056
	#define RVA_AeroMap_addDrag 2840672
	#define RVA_AeroMap_addLift 2841232
	#define RVA_AeroMap_getCurrentDragKG 2841440
	#define RVA_AeroMap_getCurrentLiftKG 2841488

	static void _hook()
	{
		HOOK_METHOD_RVA(AeroMap, init);
		HOOK_METHOD_RVA(AeroMap, loadINI);
		HOOK_METHOD_RVA(AeroMap, step);
		HOOK_METHOD_RVA(AeroMap, addDrag);
		HOOK_METHOD_RVA(AeroMap, addLift);
		HOOK_METHOD_RVA(AeroMap, getCurrentDragKG);
		HOOK_METHOD_RVA(AeroMap, getCurrentLiftKG);
	}

	void _init(Car* car, const vec3f& frontAP, const vec3f& rearAP, const std::wstring& dataPath);
	void _loadINI(const std::wstring& dataPath);
	void _step(float dt);
	void _addDrag(const vec3f& lv);
	void _addLift(const vec3f& lv);
	float _getCurrentDragKG();
	float _getCurrentLiftKG();

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

void _AeroMap::_init(Car* car, const vec3f& frontAP, const vec3f& rearAP, const std::wstring& dataPath)
{
	AC_CTOR_POD(AeroMap); // ctor is optimized, do it here

	this->car = car;
	this->carBody = car->body;
	this->CDA = 0.1;
	this->referenceArea = 1.0;
	this->frontShare = 0.5;
	this->airDensity = 1.221;

	this->loadINI(dataPath);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _AeroMap::_loadINI(const std::wstring& dataPath)
{
	auto ini(new_udt_unique<INIReader>(dataPath + L"aero.ini"));
	if (!ini->ready)
	{
		SHOULD_NOT_REACH_FATAL;
		return;
	}

	if (ini->hasSection(L"SLIPSTREAM"))
	{
		this->car->slipStream.effectGainMult = ini->getFloat(L"SLIPSTREAM", L"EFFECT_GAIN_MULT");
		this->car->slipStream.speedFactorMult = ini->getFloat(L"SLIPSTREAM", L"SPEED_FACTOR_MULT");
	}

	for (int id = 0; ; ++id)
	{
		auto strId = strf(L"WING_%d", id);
		if (!ini->hasSection(strId))
			break;

		auto wing(new_udt_unique<Wing>(this->car, *ini.get(), id, false));
		this->wings.push_back(*wing.get());
	}

	for (int id = 0; ; ++id)
	{
		auto strId = strf(L"FIN_%d", id);
		if (!ini->hasSection(strId))
			break;

		auto wing(new_udt_unique<Wing>(this->car, *ini.get(), id, true));
		this->wings.push_back(*wing.get());
	}

	if (this->wings.empty())
	{
		if (!ini->hasSection(L"DATA")) // required
		{
			SHOULD_NOT_REACH_FATAL;
			return;
		}

		this->referenceArea = ini->getFloat(L"DATA", L"REFERENCE_AREA");
		this->CD = ini->getFloat(L"DATA", L"CD");
		this->CL = ini->getFloat(L"DATA", L"CL");
		this->frontShare = ini->getFloat(L"DATA", L"FRONT_SHARE");
		this->CDX = ini->getFloat(L"DATA", L"CDX");
		this->CDY = ini->getFloat(L"DATA", L"CDY");
	}
	else
	{
		if (ini->hasSection(L"DATA")) // redundant
		{
			SHOULD_NOT_REACH_FATAL;
			return;
		}
	}

	for (int id = 0; ; ++id)
	{
		auto strId = strf(L"DYNAMIC_CONTROLLER_%d", id);
		if (!ini->hasSection(strId))
			break;

		int iWing = ini->getInt(strId, L"WING");
		if (iWing >= 0 && iWing < (int)this->wings.size())
		{
			auto dc(new_udt_unique<DynamicWingController>(this->car, *ini.get(), strId));
			this->wings[iWing].dynamicControllers.push_back(*dc.get());
			this->wings[iWing].data.hasController = true;
		}
		else
		{
			SHOULD_NOT_REACH_WARN;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _AeroMap::_step(float dt)
{
	if (this->wings.empty())
	{
		vec3f lv = this->carBody->getLocalVelocity();
		this->addDrag(lv);
		this->addLift(lv);
	}

	for (auto& wing : this->wings)
	{
		wing.step(dt);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _AeroMap::_addDrag(const vec3f& lv)
{
	float fDot = lv.sqlen();
	if (fDot != 0.0f)
	{
		vec3f vNorm = lv / sqrtf(fDot);
		this->dynamicCD = (((fabsf(vNorm.x) * this->CD) * this->CDX) + this->CD) + ((fabsf(vNorm.y) * this->CD) * this->CDY);

		float fDrag = ((this->dynamicCD * fDot) * this->airDensity) * this->referenceArea;
		this->carBody->addLocalForce(vNorm * -(fDrag * 0.5f));

		vec3f vAngVel = this->carBody->getAngularVelocity();
		fDot = vAngVel.sqlen();
		if (fDot != 0.0f)
		{
			vNorm = vAngVel / sqrtf(fDot);
			this->carBody->addLocalTorque(vNorm * -(fDot * this->CDA));
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _AeroMap::_addLift(const vec3f& lv)
{
	float fZZ = lv.z * lv.z;
	if (fZZ != 0.0f)
	{
		float fLift = (((fZZ * this->CL) * this->airDensity) * this->referenceArea) * 0.5f;

		float fFrontLift = fLift * this->frontShare;
		this->carBody->addLocalForceAtLocalPos(vec3f(0, -fFrontLift, 0), this->frontApplicationPoint);

		float fRearLift = fLift * (1.0f - this->frontShare);
		this->carBody->addLocalForceAtLocalPos(vec3f(0, -fRearLift, 0), this->rearApplicationPoint);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _AeroMap::_getCurrentDragKG()
{
	float fSum = 0;
	for (auto& wing : this->wings)
	{
		fSum += wing.status.dragKG;
	}
	return fSum;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float _AeroMap::_getCurrentLiftKG()
{
	float fSum = 0;
	for (auto& wing : this->wings)
	{
		fSum += wing.status.liftKG;
	}
	return fSum;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
