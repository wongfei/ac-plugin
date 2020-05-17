#pragma once

BEGIN_HOOK_OBJ(Tyre)

	#define RVA_Tyre_ctor 2546640
	#define RVA_Tyre_init 2623056
	#define RVA_Tyre_initCompounds 2623488
	#define RVA_Tyre_step 2635776
	#define RVA_Tyre_addGroundContact 2611584
	#define RVA_Tyre_updateLockedState 2642032
	#define RVA_Tyre_updateAngularSpeed 2641824
	#define RVA_Tyre_stepRotationMatrix 2640768
	#define RVA_Tyre_stepThermalModel 2641056
	#define RVA_Tyre_stepTyreBlankets 2641680
	#define RVA_Tyre_stepGrainBlister 2639360
	#define RVA_Tyre_stepFlatSpot 2639104

	#define RVA_Tyre_addTyreForcesV10 2616672
	#define RVA_Tyre_getCorrectedD 2621840
	#define RVA_Tyre_stepDirtyLevel 2638800
	#define RVA_Tyre_stepPuncture 2640304
	#define RVA_Tyre_addTyreForceToHub 2612224

	#define RVA_Tyre_addTyreForces 2613664
	#define RVA_Tyre_stepRelaxationLength 2640448

	static void _hook()
	{
		HOOK_METHOD_RVA(Tyre, ctor);
		HOOK_METHOD_RVA(Tyre, init);
		HOOK_METHOD_RVA(Tyre, initCompounds);
		HOOK_METHOD_RVA(Tyre, step);
		HOOK_METHOD_RVA(Tyre, addGroundContact);
		HOOK_METHOD_RVA(Tyre, updateLockedState);
		HOOK_METHOD_RVA(Tyre, updateAngularSpeed);
		HOOK_METHOD_RVA(Tyre, stepRotationMatrix);
		HOOK_METHOD_RVA(Tyre, stepThermalModel);
		HOOK_METHOD_RVA(Tyre, stepTyreBlankets);
		HOOK_METHOD_RVA(Tyre, stepGrainBlister);
		HOOK_METHOD_RVA(Tyre, stepFlatSpot);

		HOOK_METHOD_RVA(Tyre, addTyreForcesV10);
		HOOK_METHOD_RVA(Tyre, getCorrectedD);
		HOOK_METHOD_RVA(Tyre, stepDirtyLevel);
		HOOK_METHOD_RVA(Tyre, stepPuncture);
		HOOK_METHOD_RVA(Tyre, addTyreForceToHub);
	}

	Tyre* _ctor();
	void _init(ISuspension* ihub, IRayTrackCollisionProvider* rcp, const std::wstring& dataPath, int index, int carID, Car* car);
	void _initCompounds(const std::wstring& dataPath, int index);
	void _step(float dt);
	void _addGroundContact(const vec3f& pos, const vec3f& normal);
	void _updateLockedState(float dt);
	void _updateAngularSpeed(float dt);
	void _stepRotationMatrix(float dt);
	void _stepThermalModel(float dt);
	void _stepTyreBlankets(float dt);
	void _stepGrainBlister(float dt, float hubVelocity);
	void _stepFlatSpot(float dt, float hubVelocity);

	void _addTyreForcesV10(const vec3f& pos, const vec3f& normal, SurfaceDef* pSurface, float dt);
	float _getCorrectedD(float d, float* outWearMult);
	void _stepDirtyLevel(float dt, float hubSpeed);
	void _stepPuncture(float dt, float hubSpeed);
	void _addTyreForceToHub(const vec3f& pos, const vec3f& force);

	void _stepRelaxationLength(float svx, float svy, float hubVelocity, float dt);

END_HOOK_OBJ()

///////////////////////////////////////////////////////////////////////////////////////////////////

Tyre* _Tyre::_ctor()
{
	AC_CTOR_POD(Tyre);

	AC_CTOR_UDT(this->modelData)();
	AC_CTOR_UDT(this->thermalModel)();
	AC_CTOR_UDT(this->slipProvider)();
	AC_CTOR_UDT(this->shakeGenerator)();
	AC_CTOR_UDT(this->scTM)();

	this->data.blisterThreshold = 9000.0;
	this->data.grainGamma = 1.0;
	this->data.blisterGamma = 1.0;
	this->data.optimumTemp = 80.0;
	this->data.width = 0.15;
	this->data.radius = 0.3;
	this->data.k = 220000.0;
	this->data.d = 400.0;
	this->data.angularInertia = 1.6;
	this->data.thermalFrictionK = 0.03;
	this->data.thermalRollingK = 0.5;
	//this->status.inflation = 1.0;
	//this->status.wearMult = 1.0;
	//memset(&this->status, 0, sizeof(this->status)); // TODO: weird shit
	this->status.pressureStatic = 26.0;
	this->status.pressureDynamic = 26.0;
	this->status.lastTempIMO[0] = -200.0;
	this->status.lastTempIMO[1] = -200.0;
	this->status.lastTempIMO[2] = -200.0;
	this->aiMult = 1.0;
	this->tyreBlanketsOn = true;
	this->flatSpotK = 0.15;
	this->explosionTemperature = 350.0;
	this->blanketTemperature = 80.0;
	this->pressureTemperatureGain = 0.16;

	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_init(ISuspension* ihub, IRayTrackCollisionProvider* rcp, const std::wstring& dataPath, int index, int carID, Car* car)
{
	this->car = car;
	this->tyreModel = &this->scTM;
	this->thermalModel.init(12, 3, car);
	this->index = index;
	this->rayCaster = rcp->createRayCaster(3.0);
	this->rayCollisionProvider = rcp;
	this->hub = ihub;
	this->localWheelRotation.M11 = 1.0;
	this->localWheelRotation.M22 = 1.0;
	this->localWheelRotation.M33 = 1.0;
	this->localWheelRotation.M44 = 1.0;
	this->absOverride = 1.0;

	this->initCompounds(dataPath, index);
	this->setCompound(0);
	this->shakeGenerator.step(0.003f);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_initCompounds(const std::wstring& dataPath, int index)
{
	auto ini(new_udt_unique<INIReader>(dataPath + L"tyres.ini"));
	if (!ini->ready)
	{
		SHOULD_NOT_REACH_FATAL;
		return;
	}

	int iVer = ini->getInt(L"HEADER", L"VERSION");

	std::wstring strIndex[4] = {L"FRONT", L"FRONT", L"REAR", L"REAR"};
	const auto& strSection = strIndex[index];

	if (ini->hasSection(L"EXPLOSION"))
	{
		this->explosionTemperature = ini->getFloat(L"EXPLOSION", L"TEMPERATURE");
	}

	if (ini->hasSection(L"VIRTUALKM"))
	{
		this->useLoadForVKM = ini->getInt(L"VIRTUALKM", L"USE_LOAD") != 0;
	}

	if (ini->hasSection(L"ADDITIONAL1"))
	{
		this->blanketTemperature = ini->getFloat(L"ADDITIONAL1", L"BLANKETS_TEMP");
		this->pressureTemperatureGain = ini->getFloat(L"ADDITIONAL1", L"PRESSURE_TEMPERATURE_GAIN");

		float fSpread = ini->getFloat(L"ADDITIONAL1", L"CAMBER_TEMP_SPREAD_K");
		if (fSpread != 0.0f)
			this->thermalModel.camberSpreadK = fSpread;
	}

	for (int id = 0; ; id++)
	{
		auto strCompound(strSection);
		if (id > 0)
			strCompound.append(strf(L"_%d", id));

		if (!ini->hasSection(strCompound))
			break;

		auto tcd(new_udt_unique<TyreCompoundDef>());

		tcd->modelData.version = iVer;
		tcd->index = id;

		tcd->name = ini->getString(strCompound, L"NAME");
		if (iVer >= 4)
		{
			tcd->shortName = ini->getString(strCompound, L"SHORT_NAME");
			tcd->name.append(L" (");
			tcd->name.append(tcd->shortName);
			tcd->name.append(L")");
		}

		tcd->data.width = ini->getFloat(strCompound, L"WIDTH");
		if (tcd->data.width <= 0)
			tcd->data.width = 0.15f;

		tcd->data.radius = ini->getFloat(strCompound, L"RADIUS");
		if (iVer < 3)
			tcd->data.rimRadius = 0.13f;
		else
			tcd->data.rimRadius = ini->getFloat(strCompound, L"RIM_RADIUS");

		tcd->modelData.flexK = ini->getFloat(strCompound, L"FLEX");

		float fFLA = ini->getFloat(strCompound, L"FRICTION_LIMIT_ANGLE");
		float fXMU = ini->getFloat(strCompound, L"XMU");

		if (fFLA == 0.0f)
			fFLA = 7.5f;
		if (iVer >= 5)
			fXMU = 0.0f;

		auto bsp(new_udt_unique<BrushSlipProvider>(fFLA, fXMU, tcd->modelData.flexK));

		if (iVer >= 10)
		{
			tcd->modelData.cfXmult = ini->getFloat(strCompound, L"CX_MULT");
			tcd->data.radiusRaiseK = ini->getFloat(strCompound, L"RADIUS_ANGULAR_K") * 0.001f;
			
			if (ini->hasKey(strCompound, L"BRAKE_DX_MOD"))
			{
				tcd->modelData.brakeDXMod = ini->getFloat(strCompound, L"BRAKE_DX_MOD");
				if (tcd->modelData.brakeDXMod == 0.0f)
					tcd->modelData.brakeDXMod = 1.0f;
				else
					tcd->modelData.brakeDXMod += 1.0f;
			}

			if (ini->hasKey(strCompound, L"COMBINED_FACTOR"))
			{
				tcd->modelData.combinedFactor = ini->getFloat(strCompound, L"COMBINED_FACTOR");
			}
		}

		if (iVer < 5)
		{
			tcd->modelData.Dy0 = ini->getFloat(strCompound, L"DY0");
			tcd->modelData.Dy1 = ini->getFloat(strCompound, L"DY1");
			tcd->modelData.Dx0 = ini->getFloat(strCompound, L"DX0");
			tcd->modelData.Dx1 = ini->getFloat(strCompound, L"DX1");
			bsp->asy = 0.85f;
			bsp->brushModel.data.xu = fXMU;
		}
		else
		{
			float fFZ0 = ini->getFloat(strCompound, L"FZ0");
			tcd->modelData.lsExpX = ini->getFloat(strCompound, L"LS_EXPX");
			tcd->modelData.lsExpY = ini->getFloat(strCompound, L"LS_EXPY");
			tcd->modelData.Dx0 = ini->getFloat(strCompound, L"DX_REF");
			tcd->modelData.Dy0 = ini->getFloat(strCompound, L"DY_REF");

			tcd->modelData.lsMultX = calcLoadSensMult(tcd->modelData.Dx0, fFZ0, tcd->modelData.lsExpX);
			tcd->modelData.lsMultY = calcLoadSensMult(tcd->modelData.Dy0, fFZ0, tcd->modelData.lsExpY);
			bsp->asy = 0.92f;
			bsp->brushModel.data.Fz0 = fFZ0;

			float fFlexGain = ini->getFloat(strCompound, L"FLEX_GAIN");
			bsp->brushModel.data.maxSlip0 = tanf(fFLA * 0.017453f);
			bsp->brushModel.data.maxSlip1 = tanf(((fFlexGain + 1.0f) * fFLA) * 0.017453f);
			bsp->version = 5;

			if (ini->hasKey(strCompound, L"DY_CURVE"))
			{
				tcd->modelData.dyLoadCurve = ini->getCurve(strCompound, L"DY_CURVE");
			}

			if (ini->hasKey(strCompound, L"DX_CURVE"))
			{
				tcd->modelData.dxLoadCurve = ini->getCurve(strCompound, L"DX_CURVE");
			}

			//float fMaximum = 0, fMaxSlip = 0;
			//bsp->calcMaximum(fFZ0 * 0.5f, &fMaximum, &fMaxSlip);
			//this->loadSensExpD(tcd->modelData.lsExpY, tcd->modelData.lsMultY, fFZ0 * 0.5f);
			//atanf(fMaxSlip);
			//printf
		}

		bsp->recomputeMaximum();

		if (iVer >= 7)
		{
			bsp->asy = ini->getFloat(strCompound, L"FALLOFF_LEVEL");
			bsp->brushModel.data.falloffSpeed = ini->getFloat(strCompound, L"FALLOFF_SPEED");
		}

		tcd->slipProvider = *bsp.get();

		tcd->modelData.speedSensitivity = ini->getFloat(strCompound, L"SPEED_SENSITIVITY");
		tcd->modelData.relaxationLength = ini->getFloat(strCompound, L"RELAXATION_LENGTH");
		tcd->modelData.rr0 = ini->getFloat(strCompound, L"ROLLING_RESISTANCE_0");
		tcd->modelData.rr1 = ini->getFloat(strCompound, L"ROLLING_RESISTANCE_1");

		if (iVer == 1)
		{
			tcd->modelData.rr_sa = ini->getFloat(strCompound, L"ROLLING_RESISTANCE_SA");
			tcd->modelData.rr_sr = ini->getFloat(strCompound, L"ROLLING_RESISTANCE_SR");
		}
		else
		{
			tcd->modelData.rr_slip = ini->getFloat(strCompound, L"ROLLING_RESISTANCE_SLIP");
		}

		tcd->modelData.camberGain = ini->getFloat(strCompound, L"CAMBER_GAIN");
		tcd->modelData.dcamber0 = ini->getFloat(strCompound, L"DCAMBER_0");
		tcd->modelData.dcamber1 = ini->getFloat(strCompound, L"DCAMBER_1");

		if (tcd->modelData.dcamber0 == 0.0f || tcd->modelData.dcamber1 == 0.0f)
		{
			tcd->modelData.dcamber0 = 0.1f;
			tcd->modelData.dcamber1 = -0.8f;
		}

		if (ini->hasKey(strCompound, L"DCAMBER_LUT"))
		{
			tcd->modelData.dCamberCurve = ini->getCurve(strCompound, L"DCAMBER_LUT");
			tcd->modelData.useSmoothDCamberCurve = ini->getInt(strCompound, L"DCAMBER_LUT_SMOOTH") != 0;
		}

		tcd->data.angularInertia = ini->getFloat(strCompound, L"ANGULAR_INERTIA");
		tcd->data.d = ini->getFloat(strCompound, L"DAMP");
		tcd->data.k = ini->getFloat(strCompound, L"RATE");

		if (tcd->data.angularInertia == 0.0f)
			tcd->data.angularInertia = 1.2f;
		if (tcd->data.d == 0.0f)
			tcd->data.d = 400.0f;
		if (tcd->data.k == 0.0f)
			tcd->data.k = 220000.0f;
		if (tcd->modelData.Dx0 == 0.0f)
			tcd->modelData.Dx0 = tcd->modelData.Dy0 * 1.2f;
		if (tcd->modelData.Dx1 == 0.0f)
			tcd->modelData.Dx1 = tcd->modelData.Dy1 * 0.1f;

		tcd->pressureStatic = ini->getFloat(strCompound, L"PRESSURE_STATIC");
		if (tcd->pressureStatic == 0.0f)
			tcd->pressureStatic = 26.0f;

		tcd->modelData.pressureRef = tcd->pressureStatic;
		this->status.pressureDynamic = tcd->pressureStatic;

		tcd->modelData.pressureSpringGain = ini->getFloat(strCompound, L"PRESSURE_SPRING_GAIN");
		if (tcd->modelData.pressureSpringGain == 0.0f)
			tcd->modelData.pressureSpringGain = 1000.0f;

		tcd->modelData.pressureFlexGain = ini->getFloat(strCompound, L"PRESSURE_FLEX_GAIN");
		tcd->modelData.pressureRRGain = ini->getFloat(strCompound, L"PRESSURE_RR_GAIN");
		tcd->modelData.pressureGainD = ini->getFloat(strCompound, L"PRESSURE_D_GAIN");

		tcd->modelData.idealPressure = ini->getFloat(strCompound, L"PRESSURE_IDEAL");
		if (tcd->modelData.idealPressure == 0.0f)
			tcd->modelData.idealPressure = 26.0f;

		auto strThermal(L"THERMAL_" + strCompound);

		if (ini->hasSection(strThermal))
		{
			tcd->thermalPatchData.surfaceTransfer = ini->getFloat(strThermal, L"SURFACE_TRANSFER");
			tcd->thermalPatchData.patchTransfer = ini->getFloat(strThermal, L"PATCH_TRANSFER");
			tcd->thermalPatchData.patchCoreTransfer = ini->getFloat(strThermal, L"CORE_TRANSFER");
			tcd->data.thermalFrictionK = ini->getFloat(strThermal, L"FRICTION_K");
			tcd->data.thermalRollingK = ini->getFloat(strThermal, L"ROLLING_K");

			if (iVer >= 5)
			{
				tcd->thermalPatchData.internalCoreTransfer = ini->getFloat(strThermal, L"INTERNAL_CORE_TRANSFER");

				if (ini->hasKey(strThermal, L"COOL_FACTOR"))
				{
					tcd->thermalPatchData.coolFactorGain = (ini->getFloat(strThermal, L"COOL_FACTOR") - 1.0f) * 0.000324f;
				}
			}

			if (iVer >= 6)
			{
				tcd->data.thermalRollingSurfaceK = ini->getFloat(strThermal, L"SURFACE_ROLLING_K");
			}

			auto strFile = ini->getString(strThermal, L"PERFORMANCE_CURVE");
			tcd->thermalPerformanceCurve.load(dataPath + strFile);
		}

		auto strFile = ini->getString(strCompound, L"WEAR_CURVE");
		tcd->modelData.wearCurve.load(dataPath + strFile);
		tcd->modelData.wearCurve.scale(0.01f);

		int iTpcCount = tcd->thermalPerformanceCurve.getCount();
		if (iTpcCount > 0)
		{
			for (int n = 0; n < iTpcCount; ++n)
			{
				auto pair = tcd->thermalPerformanceCurve.getPairAtIndex(n);
				if (pair.second >= 1.0f)
				{
					tcd->data.grainThreshold = pair.first;
					break;
				}
			}

			for (int n = iTpcCount - 1; n > 0; --n)
			{
				auto pair = tcd->thermalPerformanceCurve.getPairAtIndex(n);
				if (pair.second >= 1.0f)
				{
					tcd->data.blisterThreshold = pair.first;
					tcd->data.optimumTemp = pair.first;
					break;
				}
			}
		}

		tcd->modelData.maxWearMult = 100.0f;
		int iWcCount = tcd->modelData.wearCurve.getCount();
		for (int n = 0; n < iWcCount; ++n)
		{
			auto pair = tcd->modelData.wearCurve.getPairAtIndex(n);
			if (pair.second < tcd->modelData.maxWearMult)
			{
				tcd->modelData.maxWearKM = pair.first;
				tcd->modelData.maxWearMult = pair.second;
			}
		}

		if (iVer >= 3)
		{
			tcd->data.blisterGamma = ini->getFloat(strThermal, L"BLISTER_GAMMA");
			tcd->data.blisterGain = ini->getFloat(strThermal, L"BLISTER_GAIN");
			tcd->data.grainGamma = ini->getFloat(strThermal, L"GRAIN_GAMMA");
			tcd->data.grainGain = ini->getFloat(strThermal, L"GRAIN_GAIN");
		}

		float fSens;
		if (iVer < 5)
			fSens = Tyre::loadSensLinearD(tcd->modelData.Dy0, tcd->modelData.Dy1, 3000.0f);
		else
			fSens = Tyre::loadSensExpD(tcd->modelData.lsExpY, tcd->modelData.lsMultY, 3000.0f);

		tcd->data.softnessIndex = tmax(0.0f, fSens - 1.0f);

		this->compoundDefs.push_back(*tcd.get()); // TODO: isTyreLegal
	}

	if (this->compoundDefs.empty())
	{
		SHOULD_NOT_REACH_FATAL;
		return;
	}

	if (iVer < 4)
	{
		this->generateCompoundNames();
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_step(float dt)
{
	this->status.feedbackTorque = 0.0f;
	this->status.Fx = 0.0f;
	this->status.Mz = 0.0f;
	this->status.slipFactor = 0.0f;
	this->status.rollingResistence = 0.0f;
	this->slidingVelocityY = 0.0f;
	this->slidingVelocityX = 0.0f;
	this->totalSlideVelocity = 0.0f;
	this->totalHubVelocity = 0.0f;
	this->surfaceDef = nullptr;

	mat44f mxWorld = this->hub->getHubWorldMatrix();
	vec3f vWorldM2(&mxWorld.M21);

	this->worldPosition.x = mxWorld.M41;
	this->worldPosition.y = mxWorld.M42;
	this->worldPosition.z = mxWorld.M43;

	this->worldRotation = mxWorld;
	this->worldRotation.M41 = 0.0f;
	this->worldRotation.M42 = 0.0f;
	this->worldRotation.M43 = 0.0f;

	if (!isfinite(this->status.angularVelocity))
	{
		SHOULD_NOT_REACH_WARN;
		this->status.angularVelocity = 0.0f;
	}

	if (!this->status.isLocked)
	{
		if (this->car && this->car->torqueModeEx == TorqueModeEX::reactionTorques)
		{
			float fTorq = this->inputs.electricTorque + this->inputs.brakeTorque + this->inputs.handBrakeTorque;
			this->hub->addTorque(vec3f(&mxWorld.M11) * fTorq);
		}
	}

	vec3f vWorldPos = this->worldPosition;
	vec3f vHitPos(0, 0, 0);
	vec3f vHitNorm(0, 0, 0);

	ICollisionObject* pCollisionObject = nullptr;
	SurfaceDef* pSurface = nullptr;
	bool bHasContact = false;

	auto pCollisionProvider = this->rayCollisionProvider;
	if (pCollisionProvider)
	{
		vec3f vRayPos(vWorldPos.x, vWorldPos.y + 2.0f, vWorldPos.z);
		vec3f vRayDir(0.0f, -1.0f, 0.0f);

		if (this->rayCaster)
		{
			RayCastHit hit = this->rayCaster->rayCast(vRayPos, vRayDir);
			bHasContact = hit.hasContact;

			if (bHasContact)
			{
				vHitPos = hit.pos;
				vHitNorm = hit.normal;
				pCollisionObject = (ICollisionObject*)hit.collisionObject;
				pSurface = (SurfaceDef*)pCollisionObject->getUserPointer();
			}
		}
		else
		{
			RayCastResult hit;
			pCollisionProvider->rayCast(vRayPos, vRayDir, &hit, 2.0f);
			bHasContact = hit.hasHit;

			if (bHasContact)
			{
				vHitPos = hit.pos;
				vHitNorm = hit.normal;
				pCollisionObject = (ICollisionObject*)hit.collisionObject;
				pSurface = (SurfaceDef*)hit.surfaceDef;
			}
		}
	}

	float fTest = 0;

	if (!bHasContact || mxWorld.M22 <= 0.35f)
	{
		this->status.ndSlip = 0.0f;
		this->status.Fy = 0.0f;
		goto LB_COMPUTE_TORQ;
	}

	this->surfaceDef = pSurface;
	this->unmodifiedContactPoint = vHitPos;

	fTest = vHitNorm * vWorldM2;
	if (fTest <= 0.96f)
	{
		float fTestAcos;
		if (fTest <= -1.0f || fTest >= 1.0f)
			fTestAcos = 0.0f;
		else
			fTestAcos = acosf(fTest);

		float fAngle = fTestAcos - acosf(0.96f);

		// TODO: cross product?
		vec3f vAxis(
			(vWorldM2.z * vHitNorm.y) - (vWorldM2.y * vHitNorm.z),
			(vWorldM2.x * vHitNorm.z) - (vWorldM2.z * vHitNorm.x),
			(vWorldM2.y * vHitNorm.x) - (vWorldM2.x * vHitNorm.y)
		);

		mat44f mxHit = mat44f::createFromAxisAngle(vAxis.get_norm(), fAngle);

		vHitNorm = vec3f(
			(((mxHit.M11 * vHitNorm.x) + (mxHit.M21 * vHitNorm.y)) + (mxHit.M31 * vHitNorm.z)) + mxHit.M41,
			(((mxHit.M12 * vHitNorm.x) + (mxHit.M22 * vHitNorm.y)) + (mxHit.M32 * vHitNorm.z)) + mxHit.M42,
			(((mxHit.M13 * vHitNorm.x) + (mxHit.M23 * vHitNorm.y)) + (mxHit.M33 * vHitNorm.z)) + mxHit.M43
		);
	}
	else
	{
		vec3f vHitOff = vHitPos - this->worldPosition;
		float fDot = vHitNorm * vHitOff;
		vHitPos = (vHitNorm * fDot) + this->worldPosition;
	}

	this->contactPoint = vHitPos;
	this->contactNormal = vHitNorm;

	if (pSurface)
	{
		float fSinHeight = pSurface->sinHeight;
		if (fSinHeight != 0.0f)
		{
			float fSinLength = pSurface->sinLength;
			this->contactPoint.y -= (((sinf(fSinLength * this->contactPoint.x) * cosf(fSinLength * this->contactPoint.z)) + 1.0f) * fSinHeight);
		}
	}

	if (pSurface->granularity != 0.0f)
	{
		float v1[3] = { 1.0f, 5.8f, 11.4f };
		float v2[3] = { 0.005f, 0.005f, 0.01f };

		float cx = this->contactPoint.x;
		float cy = this->contactPoint.y;
		float cz = this->contactPoint.z;

		for (int id = 0; id < 3; ++id)
		{
			float v = v1[id];
			cy = cy + ((((sinf(v * cx) * cosf(v * cz)) + 1.0f) * v2[id]) * -0.6f);
		}

		this->contactPoint.y = cy;
	}

	this->addGroundContact(this->contactPoint, this->contactNormal);

	if (this->modelData.version < 10)
	{
		TODO_WTF_IS_THIS;

		this->addTyreForces(this->contactPoint, this->contactNormal, pSurface, dt); // TODO: implement
	}
	else
	{
		this->addTyreForcesV10(this->contactPoint, this->contactNormal, pSurface, dt);
	}

	if (pSurface)
	{
		if (pSurface->damping > 0.0f)
		{
			auto vBodyVel = this->car->body->getVelocity();
			float fMass = this->car->body->getMass();

			vec3f vForce = vBodyVel * -(fMass * pSurface->damping);
			vec3f vPos(0, 0, 0);

			this->car->body->addForceAtLocalPos(vForce, vPos);
		}
	}

LB_COMPUTE_TORQ:

	float fHandBrakeTorque = this->inputs.handBrakeTorque;
	float fBrakeTorque = this->inputs.brakeTorque * this->absOverride;

	if (fBrakeTorque <= fHandBrakeTorque)
		fBrakeTorque = fHandBrakeTorque;

	float fAngularVelocitySign = signf(this->status.angularVelocity);
	float fTorq = 0.0f;

	if (this->modelData.version < 10)
	{
		TODO_WTF_IS_THIS;

		fTorq = ((this->status.loadedRadius * this->status.Fx) - (fAngularVelocitySign * fBrakeTorque)) + this->status.rollingResistence;
	}
	else
	{
		fTorq = this->status.rollingResistence - ((fAngularVelocitySign * fBrakeTorque) + this->localMX);
	}

	if (!isfinite(fTorq))
	{
		SHOULD_NOT_REACH_WARN;
		fTorq = 0.0f;
	}

	float fFeedbackTorque = fTorq + this->inputs.electricTorque;

	if (!isfinite(fFeedbackTorque))
	{
		SHOULD_NOT_REACH_WARN;
		fFeedbackTorque = 0.0f;
	}

	this->status.feedbackTorque = fFeedbackTorque;

	if (this->driven)
	{
		this->updateLockedState(dt);

		float fS0 = signf(this->oldAngularVelocity);
		float fS1 = signf(this->status.angularVelocity);

		if (fS0 != fS1 && this->totalHubVelocity < 1.0f)
			this->status.isLocked = true;

		this->oldAngularVelocity = this->status.angularVelocity;
	}
	else
	{
		this->updateAngularSpeed(dt);
		this->stepRotationMatrix(dt);
	}

	if (this->totalHubVelocity < 10.0f)
		this->status.slipFactor = fabsf(this->totalHubVelocity * 0.1f) * this->status.slipFactor;

	this->stepThermalModel(dt);

	this->status.pressureDynamic = ((this->thermalModel.coreTemp - 26.0f) * this->pressureTemperatureGain) + this->status.pressureStatic;

	this->stepGrainBlister(dt, this->totalHubVelocity);
	this->stepFlatSpot(dt, this->totalHubVelocity);

	if (this->onStepCompleted)
	{
		this->onStepCompleted();
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_addGroundContact(const vec3f& pos, const vec3f& normal)
{
	vec3f vOffset = this->worldPosition - pos;
	float fDistToGround = vOffset.len();
	this->status.distToGround = fDistToGround;

	float fRadius;
	if (this->data.radiusRaiseK == 0.0f)
		fRadius = this->data.radius;
	else
		fRadius = (fabsf(this->status.angularVelocity) * this->data.radiusRaiseK) + this->data.radius;

	if (this->status.inflation < 1.0f)
		fRadius = ((fRadius - this->data.rimRadius) * this->status.inflation) + this->data.rimRadius;

	this->status.liveRadius = fRadius;
	this->status.effectiveRadius = fRadius;

	if (fDistToGround > fRadius)
	{
		this->status.loadedRadius = fRadius;
		this->status.depth = 0;
		this->status.load = 0;
		this->status.Fy = 0.0;
		this->status.Fx = 0;
		this->status.Mz = 0;
		this->rSlidingVelocityX = 0;
		this->rSlidingVelocityY = 0;
		this->status.ndSlip = 0;
	}
	else
	{
		float fDepth = fRadius - fDistToGround;
		float fLoadedRadius = fRadius - fDepth;

		this->status.depth = fDepth;
		this->status.loadedRadius = fLoadedRadius;

		float fMaybePressure;
		if (fLoadedRadius <= this->data.rimRadius)
		{
			fMaybePressure = 200000.0f;
		}
		else
		{
			fMaybePressure = ((this->status.pressureDynamic - this->modelData.pressureRef) * this->modelData.pressureSpringGain) + this->data.k;
			if (fMaybePressure < 0.0f)
				fMaybePressure = 0.0f;
		}

		vec3f vHubVel = this->hub->getPointVelocity(pos);
		float fLoad = -((vHubVel * normal) * this->data.d) + (fDepth * fMaybePressure);
		this->status.load = fLoad;

		this->hub->addForceAtPos(normal * fLoad, pos, this->driven, false);

		if (this->status.load < 0.0f)
			this->status.load = 0.0f;
	}

	if (this->externalInputs.isActive)
		this->status.load = this->externalInputs.load;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_updateLockedState(float dt)
{
	if (this->status.isLocked)
	{
		float fBrake = tmax(this->absOverride * this->inputs.brakeTorque, this->inputs.handBrakeTorque);

		this->status.isLocked =
			(fabsf(fBrake) >= fabsf(this->status.loadedRadius * this->status.Fx))
			&& (fabsf(this->status.angularVelocity) < 1.0f)
			&& (!this->driven);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_updateAngularSpeed(float dt)
{
	this->updateLockedState(dt);

	float fAngVel = this->status.angularVelocity + ((this->status.feedbackTorque / this->data.angularInertia) * dt);

	if (signf(fAngVel) != signf(this->oldAngularVelocity))
		this->status.isLocked = true;

	this->oldAngularVelocity = fAngVel;
	this->status.angularVelocity = this->status.isLocked ? 0.0f : fAngVel;

	if (fabsf(this->status.angularVelocity) < 1.0f)
		this->status.angularVelocity *= 0.9f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_stepRotationMatrix(float dt)
{
	if (this->car && !this->car->isSleeping() && fabsf(this->status.angularVelocity) > 0.1f)
	{
		mat44f m = mat44f::createFromAxisAngle(vec3f(1, 0, 0), this->status.angularVelocity * dt);
		auto xm = DirectX::XMMatrixMultiply(xmload(m), xmload(this->localWheelRotation));

		this->localWheelRotation = xmstore(xm);
		this->localWheelRotation.M41 = 0;
		this->localWheelRotation.M42 = 0;
		this->localWheelRotation.M43 = 0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_stepThermalModel(float dt)
{
	Car* pCar = this->car;

	float fGripLevel = 1.0f;
	if (pCar)
		fGripLevel = pCar->ksPhysics->track->dynamicGripLevel;

	float fThermalInput = (
		sqrtf((this->slidingVelocityX * this->slidingVelocityX) + (this->slidingVelocityY * this->slidingVelocityY))
		* ((this->status.D * this->status.load) * this->data.thermalFrictionK))
		* fGripLevel;

	SurfaceDef* pSurface = this->surfaceDef;
	if (pSurface)
		fThermalInput *= pSurface->gripMod;

	this->status.thermalInput = fThermalInput;

	if (isfinite(fThermalInput))
	{
		float fPressureDynamic = this->status.pressureDynamic;
		float fIdealPressure = this->modelData.idealPressure;
		float fThermalRollingK = this->data.thermalRollingK;

		float fScale = (((fIdealPressure / fPressureDynamic) - 1.0f) * this->modelData.pressureRRGain) + 1.0f;
		if (fPressureDynamic >= 0.0)
			fThermalRollingK *= fScale;

		int iVer = this->modelData.version;
		if (iVer < 5)
			this->status.thermalInput += (((fThermalRollingK * this->status.angularVelocity) * this->status.load) * 0.001f);

		if (iVer >= 6)
			this->status.thermalInput += ((((fScale * this->data.thermalRollingSurfaceK) * this->status.angularVelocity) * this->status.load) * 0.001f);

		this->thermalModel.addThermalInput(this->status.camberRAD, (fPressureDynamic / fIdealPressure) - 1.0f, this->status.thermalInput);

		if (this->modelData.version >= 5)
			this->thermalModel.addThermalCoreInput(((fThermalRollingK * this->status.angularVelocity) * this->status.load) * 0.001f);

		this->thermalModel.step(dt, this->status.angularVelocity, this->status.camberRAD);
	}
	else
	{
		SHOULD_NOT_REACH_WARN;
	}

	if (pCar)
	{
		if (pCar->ksPhysics->allowTyreBlankets)
			this->stepTyreBlankets(dt);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_stepTyreBlankets(float dt)
{
	if (this->tyreBlanketsOn)
	{
		if (getSpeedKMH(this->car) > 10.0f)
			this->tyreBlanketsOn = false;

		float fTemp = tmin(this->blanketTemperature, this->data.optimumTemp);
		this->thermalModel.setTemperature(fTemp);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_stepGrainBlister(float dt, float hubVelocity)
{
	Car* pCar = this->car;

	if (pCar && this->car->ksPhysics->tyreConsumptionRate > 0.0f)
	{
		if (this->status.load > 0.0f)
		{
			float fGrainGain = this->data.grainGain;
			float fCoreTemp = this->thermalModel.coreTemp;
			float fNdSlip = tmin(this->status.ndSlip, 2.5f);

			if (fGrainGain > 0.0f)
			{
				float fGrainThreshold = this->data.grainThreshold;
				if (fCoreTemp < fGrainThreshold)
				{
					auto pSurface = this->surfaceDef;
					if (pSurface)
					{
						if (hubVelocity > 2.0f)
						{
							float fGripMod = pSurface->gripMod;
							if (fGripMod >= 0.95f)
							{
								float fTest = ((fGripMod * hubVelocity) * fGrainGain) * ((fGrainThreshold - fCoreTemp) * 0.0001f);
								if (isfinite(fTest))
								{
									this->status.grain += fTest * powf(fNdSlip, this->data.grainGamma) * this->car->ksPhysics->tyreConsumptionRate * dt;
								}
								else
								{
									SHOULD_NOT_REACH_WARN;
								}
							}
						}
					}
				}
			}

			auto pSurface = this->surfaceDef;
			if (pSurface)
			{
				float fTest = (pSurface->gripMod * hubVelocity) * fGrainGain * 0.00005f;
				if (isfinite(fTest))
				{
					if (fTest > 0.0f)
						this->status.grain -= fTest * powf(fNdSlip, this->data.grainGamma) * this->car->ksPhysics->tyreConsumptionRate * dt;
				}
				else
				{
					SHOULD_NOT_REACH_WARN;
				}
			}

			float fBlisterGain = this->data.blisterGain;
			if (fBlisterGain > 0.0f)
			{
				float fBlisterThreshold = this->data.blisterThreshold;
				if (fCoreTemp > fBlisterThreshold)
				{
					pSurface = this->surfaceDef;
					if (pSurface)
					{
						if (hubVelocity > 2.0f)
						{
							float fGripMod = pSurface->gripMod;
							if (fGripMod >= 0.95f)
							{
								float fTest = ((fGripMod * this->totalHubVelocity) * fBlisterGain) * ((fCoreTemp - fBlisterThreshold) * 0.0001f);
								if (isfinite(fTest))
								{
									this->status.blister += fTest * powf(fNdSlip, this->data.blisterGamma) * this->car->ksPhysics->tyreConsumptionRate * dt;
								}
								else
								{
									SHOULD_NOT_REACH_WARN;
								}
							}
						}
					}
				}
			}

			this->status.grain = tclamp<double>(this->status.grain, 0.0, 100.0);
			this->status.blister = tclamp<double>(this->status.blister, 0.0, 100.0);
		}
	}
	else
	{
		this->status.grain = 0.0;
		this->status.blister = 0.0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void _Tyre::_stepFlatSpot(float dt, float hubVelocity)
{
	if (fabsf(this->status.angularVelocity) <= 0.3f || this->status.slipRatio < -0.98f)
	{
		if (this->surfaceDef)
		{
			if (hubVelocity > 3.0f)
			{
				if (this->car)
				{
					float fDamage = this->car->ksPhysics->mechanicalDamageRate;
					if (fDamage != 0.0f)
					{
						float fGrip = this->surfaceDef->gripMod;
						if (fGrip >= 0.95f)
						{
							this->status.flatSpot += (((hubVelocity * this->flatSpotK) * this->status.load) * fGrip) * 0.00001f * dt * fDamage * this->data.softnessIndex;

							if (this->status.flatSpot > 1.0f)
								this->status.flatSpot = 1.0f;
						}
					}
				}
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
