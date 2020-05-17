#pragma once

BEGIN_HOOK_OBJ(Wing)

	#define RVA_Wing_ctor 2822976
	#define RVA_Wing_step 2829248
	#define RVA_Wing_stepDynamicControllers 2829776
	#define RVA_Wing_addDrag 2827296
	#define RVA_Wing_addLift 2828080

	static void _hook()
	{
		HOOK_METHOD_RVA(Wing, ctor);
		HOOK_METHOD_RVA(Wing, step);
		HOOK_METHOD_RVA(Wing, stepDynamicControllers);
		HOOK_METHOD_RVA(Wing, addDrag);
		HOOK_METHOD_RVA(Wing, addLift);
	}

	Wing* _ctor(Car* car, INIReader* ini, int index, bool isVertical);
	void _step(float dt);
	void _stepDynamicControllers(float dt);
	void _addDrag(const vec3f& lv);
	void _addLift(const vec3f& lv);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

Wing* _Wing::_ctor(Car* car, INIReader* ini, int index, bool isVertical)
{
	AC_CTOR_POD(Wing);

	AC_CTOR_UDT(this->data)();
	AC_CTOR_UDT(this->engineer)(car);

	this->car = car;
	this->status.angleMult = 1.0;
	this->SPEED_DAMAGE_COEFF = 300.0;
	this->SURFACE_DAMAGE_COEFF = 300.0;

	auto strId = strf(L"%s_%d", (isVertical ? L"FIN" : L"WING"), index);

	this->data.isVertical = isVertical;
	this->data.name = ini->getString(strId, L"NAME");
	this->data.chord = ini->getFloat(strId, L"CHORD");
	this->data.span = ini->getFloat(strId, L"SPAN");
	this->data.area = this->data.chord * this->data.span;
	this->data.position = ini->getFloat3(strId, L"POSITION");

	auto strPath = car->carDataPath + ini->getString(strId, L"LUT_AOA_CL");
	this->data.lutAOA_CL.load(strPath);

	strPath = car->carDataPath + ini->getString(strId, L"LUT_AOA_CD");
	this->data.lutAOA_CD.load(strPath);

	strPath = car->carDataPath + ini->getString(strId, L"LUT_GH_CL");
	if (Path::fileExists(strPath, false))
		this->data.lutGH_CL.load(strPath);

	strPath = car->carDataPath + ini->getString(strId, L"LUT_GH_CD");
	if (Path::fileExists(strPath, false))
		this->data.lutGH_CD.load(strPath);

	this->data.cdGain = ini->getFloat(strId, L"CD_GAIN");
	this->data.clGain = ini->getFloat(strId, L"CL_GAIN");

	this->status.angle = ini->getFloat(strId, L"ANGLE");
	this->status.inputAngle = this->status.angle;
	this->status.frontShare = this->engineer.getPointFrontShare(this->data.position);

	int iVer = ini->getInt(L"HEADER", L"VERSION");
	if (iVer >= 2)
	{
		const wchar_t* strZones[] = {L"FRONT", L"REAR", L"LEFT", L"RIGHT", L"CENTER"};
		for (int i = 0; i < 5; ++i)
		{
			this->damageCD[i] = ini->getFloat(strId, strf(L"ZONE_%s_CD", strZones[i]));
			this->damageCL[i] = ini->getFloat(strId, strf(L"ZONE_%s_CL", strZones[i]));
		}
		this->hasDamage = true;
	}

	if (iVer >= 3)
	{
		this->data.yawGain = ini->getFloat(strId, L"YAW_CL_GAIN");
	}

	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Wing::_step(float dt)
{
	if (!this->overrideStatus.isActive)
		this->stepDynamicControllers(dt);

	vec3f vGroundWind = this->car->getGroundWindVector();
	vec3f vWorldVel = this->car->body->getLocalPointVelocity(this->data.position);
	vec3f vLocalVel = this->car->body->worldToLocalNormal(vWorldVel + vGroundWind);
	vec3f vWingWorld = this->car->body->localToWorld(this->data.position);
	this->status.groundHeight = this->engineer.getPointGroundHeight(vWingWorld);
  
	float fAngle = this->status.angle;
	if (this->overrideStatus.isActive)
		this->status.angle = this->overrideStatus.overrideAngle;
	
	if (vLocalVel.z == 0.0f)
	{
		this->status.aoa = 0;
		this->status.cd = 0;
		this->status.cl = 0;
		this->status.yawAngle = 0;
	}
	else
	{
		this->status.aoa = atanf((1.0f / vLocalVel.z) * vLocalVel.y) * 57.29578f;
		this->status.yawAngle = atanf((1.0f / vLocalVel.z) * vLocalVel.x) * 57.29578f;
		this->addDrag(vLocalVel);
		this->addLift(vLocalVel);
	}
  
	if (this->overrideStatus.isActive)
		this->status.angle = fAngle;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Wing::_stepDynamicControllers(float dt)
{
	float fAngle = this->status.inputAngle;
	for (auto& dc : this->dynamicControllers)
	{
		dc.step();
		
		if (dc.combinatorMode == DynamicWingController_eCombinatorMode::eAdd)
		{
			fAngle += dc.outputAngle;
		}
		else if (dc.combinatorMode == DynamicWingController_eCombinatorMode::eMult)
		{
			fAngle *= dc.outputAngle;
		}

		fAngle = tclamp(fAngle, dc.downLimit, dc.upLimit);
	}
	this->status.angle = fAngle;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Wing::_addDrag(const vec3f& lv)
{
	float fAngleOff = this->data.isVertical ? this->status.yawAngle : this->status.aoa;
	this->status.cd = this->data.lutAOA_CD.getValue((this->status.angleMult * this->status.angle) + fAngleOff) * this->data.cdGain;

	if (this->hasDamage) // TODO: implement
	{
	}

	if (this->data.lutGH_CD.getCount())
	{
		float fLut = this->data.lutGH_CD.getValue(this->status.groundHeight);
		this->status.groundEffectDrag = fLut;
		this->status.cd *= fLut;
	}

	float fDot = lv.sqlen();
	float fDrag = (((fDot * this->status.cd) * this->car->aeroMap.airDensity) * this->data.area) * 0.5f;
	this->status.dragKG = fDrag * 0.10197838f;

	if (fDot != 0.0f)
	{
		this->car->body->addLocalForceAtLocalPos(lv.get_norm() * -fDrag, this->data.position);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Wing::_addLift(const vec3f& lv)
{
	float fAngleOff, fAxis;

	if (this->data.isVertical)
	{
		fAngleOff = this->status.yawAngle;
		fAxis = lv.x;
	}
	else
	{
		fAngleOff = this->status.aoa;
		fAxis = lv.y;
	}

	this->status.cl = this->data.lutAOA_CL.getValue((this->status.angleMult * this->status.angle) + fAngleOff) * this->data.clGain;

	if (lv.z < 0.0f)
		this->status.cl = 0.0f;

	if (!this->data.isVertical && this->data.yawGain != 0.0f)
	{
		float v8 = (sinf(fabsf(this->status.yawAngle) * 0.017453f) * this->data.yawGain) + 1.0f;
		this->status.cl *= tclamp(v8, 0.0f, 1.0f);
	}
  
	if (this->data.lutGH_CL.getCount())
	{
		float fLut = this->data.lutGH_CL.getValue(this->status.groundHeight);
		this->status.groundEffectLift = fLut;
		this->status.cl *= fLut;
	}
  
	if (this->hasDamage) // TODO: implement
	{
	}

	float fDot = (fAxis * fAxis) + (lv.z * lv.z);
	float fLift = (((fDot * this->status.cl) * this->car->aeroMap.airDensity) * this->data.area) * 0.5f;
	this->status.liftKG = fLift * 0.10197838f;
  
	if (fDot != 0.0f)
	{
		// TODO: check

		vec3f vNorm = lv.get_norm();
		vec3f vOut = this->data.isVertical ? vec3f(-vNorm.z, 0, vNorm.x) : vec3f(0, vNorm.z, -vNorm.y);
		vec3f vForce = vOut * -fLift;

		this->status.liftVector = vForce;
		this->car->body->addLocalForceAtLocalPos(vForce, this->data.position);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
