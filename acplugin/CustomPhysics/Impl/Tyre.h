#pragma once

BEGIN_HOOK_OBJ(Tyre)

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

	#define RVA_Tyre_stepRelaxationLength 2640448

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
		SHOULD_NOT_REACH;
		log_printf(L"INF status.angularVelocity");
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

	if (!bHasContact || mxWorld.M22 <= 0.34999999f)
	{
		this->status.ndSlip = 0.0f;
		this->status.Fy = 0.0f;
		goto LB_COMPUTE_TORQ;
	}

	this->surfaceDef = pSurface;
	this->unmodifiedContactPoint = vHitPos;

	fTest = vHitNorm * vWorldM2;
	if (fTest <= 0.9599999f)
	{
		float fTestAcos;
		if (fTest <= -1.0f || fTest >= 1.0f)
			fTestAcos = 0.0f;
		else
			fTestAcos = acosf(fTest);

		float fAngle = fTestAcos - acosf(0.9599999f);

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
		float v2[3] = { 0.0049999999f, 0.0049999999f, 0.009999999f };

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
		SHOULD_NOT_REACH;
		log_printf(L"INF fTorq");
		fTorq = 0.0f;
	}

	float fFeedbackTorque = fTorq + this->inputs.electricTorque;

	if (!isfinite(fFeedbackTorque))
	{
		SHOULD_NOT_REACH;
		log_printf(L"INF fFeedbackTorque");
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
		this->status.angularVelocity *= 0.8999999f;
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
		SHOULD_NOT_REACH;
		log_printf(L"INF fThermalInput");
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
							if (fGripMod >= 0.94999999f)
							{
								float fTest = ((fGripMod * hubVelocity) * fGrainGain) * ((fGrainThreshold - fCoreTemp) * 0.0001f);
								if (isfinite(fTest))
								{
									this->status.grain += fTest * powf(fNdSlip, this->data.grainGamma) * this->car->ksPhysics->tyreConsumptionRate * dt;
								}
								else
								{
									SHOULD_NOT_REACH;
									log_printf(L"INF fGrainTest1");
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
					SHOULD_NOT_REACH;
					log_printf(L"INF fGrainTest2");
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
							if (fGripMod >= 0.94999999f)
							{
								float fTest = ((fGripMod * this->totalHubVelocity) * fBlisterGain) * ((fCoreTemp - fBlisterThreshold) * 0.0001f);
								if (isfinite(fTest))
								{
									this->status.blister += fTest * powf(fNdSlip, this->data.blisterGamma) * this->car->ksPhysics->tyreConsumptionRate * dt;
								}
								else
								{
									SHOULD_NOT_REACH;
									log_printf(L"INF fBlisterTest");
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
						if (fGrip >= 0.94999999f)
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
