#pragma once

#define RVA_Tyre_step 2635776
#define RVA_Tyre_addGroundContact 2611584
#define RVA_Tyre_updateLockedState 2642032
#define RVA_Tyre_updateAngularSpeed 2641824
#define RVA_Tyre_stepRotationMatrix 2640768
#define RVA_Tyre_stepThermalModel 2641056
#define RVA_Tyre_stepTyreBlankets 2641680
#define RVA_Tyre_stepGrainBlister 2639360
#define RVA_Tyre_stepFlatSpot 2639104

///////////////////////////////////////////////////////////////////////////////////////////////////

void Tyre_step(Tyre* pThis, float dt)
{
	pThis->status.feedbackTorque = 0.0f;
	pThis->status.Fx = 0.0f;
	pThis->status.Mz = 0.0f;
	pThis->status.slipFactor = 0.0f;
	pThis->status.rollingResistence = 0.0f;
	pThis->slidingVelocityY = 0.0f;
	pThis->slidingVelocityX = 0.0f;
	pThis->totalSlideVelocity = 0.0f;
	pThis->totalHubVelocity = 0.0f;
	pThis->surfaceDef = nullptr;

	mat44f mxWorld = pThis->hub->getHubWorldMatrix();
	vec3f vWorldM2(mxWorld.M21, mxWorld.M22, mxWorld.M23);

	pThis->worldPosition.x = mxWorld.M41;
	pThis->worldPosition.y = mxWorld.M42;
	pThis->worldPosition.z = mxWorld.M43;

	pThis->worldRotation = mxWorld;
	pThis->worldRotation.M41 = 0.0f;
	pThis->worldRotation.M42 = 0.0f;
	pThis->worldRotation.M43 = 0.0f;

	if (!isfinite(pThis->status.angularVelocity))
	{
		log_printf(L"INF status.angularVelocity");
		pThis->status.angularVelocity = 0.0f;
	}

	if (!pThis->status.isLocked)
	{
		if (pThis->car && pThis->car->torqueModeEx == TorqueModeEX::reactionTorques)
		{
			float fTorq = pThis->inputs.electricTorque + pThis->inputs.brakeTorque + pThis->inputs.handBrakeTorque;
			vec3f vTorq(mxWorld.M11 * fTorq, mxWorld.M12 * fTorq, mxWorld.M13 * fTorq);
			pThis->hub->addTorque(vTorq);
		}
	}

	vec3f vWorldPos = pThis->worldPosition;
	vec3f vHitPos(0, 0, 0);
	vec3f vHitNorm(0, 0, 0);

	ICollisionObject* pCollisionObject = nullptr;
	SurfaceDef* pSurface = nullptr;
	bool bHasContact = false;

	auto pCollisionProvider = pThis->rayCollisionProvider;
	if (pCollisionProvider)
	{
		vec3f vRayPos(vWorldPos.x, vWorldPos.y + 2.0f, vWorldPos.z);
		vec3f vRayDir(0.0f, -1.0f, 0.0f);

		if (pThis->rayCaster)
		{
			RayCastHit hit = pThis->rayCaster->rayCast(vRayPos, vRayDir);
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
		pThis->status.ndSlip = 0.0f;
		pThis->status.Fy = 0.0f;
		goto LB_COMPUTE_TORQ;
	}

	pThis->surfaceDef = pSurface;
	pThis->unmodifiedContactPoint = vHitPos;

	fTest = vdot(vHitNorm, vWorldM2);
	if (fTest <= 0.96f)
	{
		float fTestAcos;
		if (fTest <= -1.0f || fTest >= 1.0f)
			fTestAcos = 0.0f;
		else
			fTestAcos = acosf(fTest);

		float fAngle = fTestAcos - acosf(0.96f);

		vec3f vAxis(
			(vWorldM2.z * vHitNorm.y) - (vWorldM2.y * vHitNorm.z),
			(vWorldM2.x * vHitNorm.z) - (vWorldM2.z * vHitNorm.x),
			(vWorldM2.y * vHitNorm.x) - (vWorldM2.x * vHitNorm.y)
		);

		vAxis = vnorm(vAxis);
		mat44f mxHit = mat44f::createFromAxisAngle(vAxis, fAngle);

		vHitNorm = vec3f(
			(((mxHit.M11 * vHitNorm.x) + (mxHit.M21 * vHitNorm.y)) + (mxHit.M31 * vHitNorm.z)) + mxHit.M41,
			(((mxHit.M12 * vHitNorm.x) + (mxHit.M22 * vHitNorm.y)) + (mxHit.M32 * vHitNorm.z)) + mxHit.M42,
			(((mxHit.M13 * vHitNorm.x) + (mxHit.M23 * vHitNorm.y)) + (mxHit.M33 * vHitNorm.z)) + mxHit.M43
		);
	}
	else
	{
		vec3f vHitOff = vsub(vHitPos, pThis->worldPosition);
		float fDot = vdot(vHitNorm, vHitOff);
		vHitPos = vadd(vmul(vHitNorm, fDot), pThis->worldPosition);
	}

	pThis->contactPoint = vHitPos;
	pThis->contactNormal = vHitNorm;

	if (pSurface)
	{
		float fSinHeight = pSurface->sinHeight;
		if (fSinHeight != 0.0f)
		{
			float fSinLength = pSurface->sinLength;
			pThis->contactPoint.y -= (((sinf(fSinLength * pThis->contactPoint.x) * cosf(fSinLength * pThis->contactPoint.z)) + 1.0f) * fSinHeight);
		}
	}

	if (pSurface->granularity != 0.0f)
	{
		float v1[3] = { 1.0f, 5.8f, 11.4f };
		float v2[3] = { 0.005f, 0.005f, 0.01f };

		float cx = pThis->contactPoint.x;
		float cy = pThis->contactPoint.y;
		float cz = pThis->contactPoint.z;

		for (int id = 0; id < 3; ++id)
		{
			float v = v1[id];
			cy = cy + ((((sinf(v * cx) * cosf(v * cz)) + 1.0f) * v2[id]) * -0.6f);
		}

		pThis->contactPoint.y = cy;
	}

	pThis->addGroundContact(pThis->contactPoint, pThis->contactNormal);

	if (pThis->modelData.version < 10)
	{
		DEBUG_BREAK; // TODO: what car uses this code?

		pThis->addTyreForces(pThis->contactPoint, pThis->contactNormal, pSurface, dt);
	}
	else
	{
		pThis->addTyreForcesV10(pThis->contactPoint, pThis->contactNormal, pSurface, dt);
	}

	if (pSurface)
	{
		float fDamping = pSurface->damping;
		if (fDamping > 0.0f)
		{
			auto bodyVel = pThis->car->body->getVelocity();
			float fMass = pThis->car->body->getMass();

			vec3f force(vmul(bodyVel, fMass * -fDamping));
			vec3f pos(0, 0, 0);

			pThis->car->body->addForceAtLocalPos(force, pos);
		}
	}

LB_COMPUTE_TORQ:

	float fHandBrakeTorque = pThis->inputs.handBrakeTorque;
	float fBrakeTorque = pThis->inputs.brakeTorque * pThis->absOverride;

	if (fBrakeTorque <= fHandBrakeTorque)
		fBrakeTorque = fHandBrakeTorque;

	float fAngularVelocitySign = signf(pThis->status.angularVelocity);
	float fTorq = 0.0f;

	if (pThis->modelData.version < 10)
	{
		fTorq = ((pThis->status.loadedRadius * pThis->status.Fx) - (fAngularVelocitySign * fBrakeTorque)) + pThis->status.rollingResistence;
	}
	else
	{
		fTorq = pThis->status.rollingResistence - ((fAngularVelocitySign * fBrakeTorque) + pThis->localMX);
	}

	if (!isfinite(fTorq))
	{
		log_printf(L"INF fTorq");
		fTorq = 0.0f;
	}

	float fFeedbackTorque = fTorq + pThis->inputs.electricTorque;

	if (!isfinite(fFeedbackTorque))
	{
		log_printf(L"INF fFeedbackTorque");
		fFeedbackTorque = 0.0f;
	}

	pThis->status.feedbackTorque = fFeedbackTorque;

	if (pThis->driven)
	{
		pThis->updateLockedState(dt);

		float fOldAngularVelocitySign = signf(pThis->oldAngularVelocity);
		fAngularVelocitySign = signf(pThis->status.angularVelocity);

		if (fOldAngularVelocitySign != fAngularVelocitySign && pThis->totalHubVelocity < 1.0)
			pThis->status.isLocked = 1;

		pThis->oldAngularVelocity = pThis->status.angularVelocity;
	}
	else
	{
		pThis->updateAngularSpeed(dt);
		pThis->stepRotationMatrix(dt);
	}

	float fTotalHubVelocity = pThis->totalHubVelocity;
	if (fTotalHubVelocity < 10.0f)
		pThis->status.slipFactor = fabsf(fTotalHubVelocity * 0.1f) * pThis->status.slipFactor;

	pThis->stepThermalModel(dt);

	pThis->status.pressureDynamic = ((pThis->thermalModel.coreTemp - 26.0f) * pThis->pressureTemperatureGain) + pThis->status.pressureStatic;

	pThis->stepGrainBlister(dt, pThis->totalHubVelocity);
	pThis->stepFlatSpot(dt, pThis->totalHubVelocity);

	if (pThis->onStepCompleted)
	{
		pThis->onStepCompleted();
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Tyre_addGroundContact(Tyre* pThis, const vec3f& pos, const vec3f& normal)
{
	vec3f offset = vsub(pThis->worldPosition, pos);
	float fDistToGround = vlen(offset);
	pThis->status.distToGround = fDistToGround;

	float fRadius;
	if (pThis->data.radiusRaiseK == 0.0f)
		fRadius = pThis->data.radius;
	else
		fRadius = (fabsf(pThis->status.angularVelocity) * pThis->data.radiusRaiseK) + pThis->data.radius;

	if (pThis->status.inflation < 1.0f)
		fRadius = ((fRadius - pThis->data.rimRadius) * pThis->status.inflation) + pThis->data.rimRadius;

	pThis->status.liveRadius = fRadius;
	pThis->status.effectiveRadius = fRadius;

	if (fDistToGround > fRadius)
	{
		pThis->status.loadedRadius = fRadius;
		pThis->status.depth = 0;
		pThis->status.load = 0;
		pThis->status.Fy = 0.0;
		pThis->status.Fx = 0;
		pThis->status.Mz = 0;
		pThis->rSlidingVelocityX = 0;
		pThis->rSlidingVelocityY = 0;
		pThis->status.ndSlip = 0;
	}
	else
	{
		float fDepth = fRadius - fDistToGround;
		float fLoadedRadius = fRadius - fDepth;

		pThis->status.depth = fDepth;
		pThis->status.loadedRadius = fLoadedRadius;

		float fMaybePressure;
		if (fLoadedRadius <= pThis->data.rimRadius)
		{
			fMaybePressure = 200000.0f;
		}
		else
		{
			fMaybePressure = ((pThis->status.pressureDynamic - pThis->modelData.pressureRef) * pThis->modelData.pressureSpringGain) + pThis->data.k;
			if (fMaybePressure < 0.0f)
				fMaybePressure = 0.0f;
		}

		vec3f hubVel = pThis->hub->getPointVelocity(pos);
		float fLoad = -(vdot(hubVel, normal) * pThis->data.d) + (fDepth * fMaybePressure);
		pThis->status.load = fLoad;

		vec3f force = vmul(normal, fLoad);
		pThis->hub->addForceAtPos(force, pos, pThis->driven, false);

		if (pThis->status.load < 0.0f)
			pThis->status.load = 0.0f;
	}

	if (pThis->externalInputs.isActive)
		pThis->status.load = pThis->externalInputs.load;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Tyre_updateLockedState(Tyre* pThis, float dt)
{
	if (pThis->status.isLocked)
	{
		float fBrake = pThis->absOverride * pThis->inputs.brakeTorque;
		if (fBrake <= pThis->inputs.handBrakeTorque)
			fBrake = pThis->inputs.handBrakeTorque;

		pThis->status.isLocked =
			(fabsf(fBrake) >= fabsf(pThis->status.loadedRadius * pThis->status.Fx))
			&& (fabsf(pThis->status.angularVelocity) < 1.0f)
			&& (!pThis->driven);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Tyre_updateAngularSpeed(Tyre* pThis, float dt)
{
	pThis->updateLockedState(dt);

	float fAngVel = ((pThis->status.feedbackTorque / pThis->data.angularInertia) * dt) + pThis->status.angularVelocity;
	float fOldAngVel = pThis->oldAngularVelocity;

	const float s1 = signf(fOldAngVel);
	const float s2 = signf(fAngVel);

	if (s1 != s2)
		pThis->status.isLocked = true;

	pThis->status.angularVelocity = pThis->status.isLocked ? 0.0f : fAngVel;
	pThis->oldAngularVelocity = fAngVel;

	if (fabsf(pThis->status.angularVelocity) < 1.0f)
		pThis->status.angularVelocity = pThis->status.angularVelocity * 0.9f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Tyre_stepRotationMatrix(Tyre* pThis, float dt)
{
	if (pThis->car && !pThis->car->isSleeping() && fabsf(pThis->status.angularVelocity) > 0.1)
	{
		vec3f vAxis = makev(1, 0, 0);
		float fAngle = dt * pThis->status.angularVelocity;
		mat44f m = mat44f::createFromAxisAngle(vAxis, fAngle);

		auto xm = DirectX::XMMatrixMultiply(xmload(m), xmload(pThis->localWheelRotation));

		pThis->localWheelRotation = xmstore(xm);
		pThis->localWheelRotation.M41 = 0;
		pThis->localWheelRotation.M42 = 0;
		pThis->localWheelRotation.M43 = 0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Tyre_stepThermalModel(Tyre* pThis, float dt)
{
	Car* pCar = pThis->car;

	float fGripLevel = 1.0f;
	if (pCar)
		fGripLevel = pCar->ksPhysics->track->dynamicGripLevel;

	float fThermalInput = (
		sqrtf((pThis->slidingVelocityX * pThis->slidingVelocityX) + (pThis->slidingVelocityY * pThis->slidingVelocityY))
		* ((pThis->status.D * pThis->status.load) * pThis->data.thermalFrictionK))
		* fGripLevel;

	SurfaceDef* pSurface = pThis->surfaceDef;
	if (pSurface)
		fThermalInput *= pSurface->gripMod;

	pThis->status.thermalInput = fThermalInput;

	if (isfinite(fThermalInput))
	{
		float fPressureDynamic = pThis->status.pressureDynamic;
		float fIdealPressure = pThis->modelData.idealPressure;
		float fThermalRollingK = pThis->data.thermalRollingK;

		float fScale = (((fIdealPressure / fPressureDynamic) - 1.0f) * pThis->modelData.pressureRRGain) + 1.0f;
		if (fPressureDynamic >= 0.0)
			fThermalRollingK = fThermalRollingK * fScale;

		int iVer = pThis->modelData.version;
		if (iVer < 5)
			pThis->status.thermalInput = (((fThermalRollingK * pThis->status.angularVelocity) * pThis->status.load) * 0.001f) + pThis->status.thermalInput;

		if (iVer >= 6)
			pThis->status.thermalInput = ((((fScale * pThis->data.thermalRollingSurfaceK) * pThis->status.angularVelocity) * pThis->status.load) * 0.001f) + pThis->status.thermalInput;

		pThis->thermalModel.addThermalInput(pThis->status.camberRAD, (fPressureDynamic / fIdealPressure) - 1.0f, pThis->status.thermalInput);

		if (pThis->modelData.version >= 5)
			pThis->thermalModel.addThermalCoreInput(((fThermalRollingK * pThis->status.angularVelocity) * pThis->status.load) * 0.001f);

		pThis->thermalModel.step(dt, pThis->status.angularVelocity, pThis->status.camberRAD);
	}
	else
	{
		log_printf(L"INF fThermalInput");
	}

	if (pCar)
	{
		if (pCar->ksPhysics->allowTyreBlankets)
			pThis->stepTyreBlankets(dt);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Tyre_stepTyreBlankets(Tyre* pThis, float dt)
{
	if (pThis->tyreBlanketsOn)
	{
		if (Car_getSpeedValue(pThis->car) * 3.6f > 10.0f)
			pThis->tyreBlanketsOn = 0.0f;

		float fTemp = pThis->blanketTemperature;
		if (fTemp >= pThis->data.optimumTemp)
			fTemp = pThis->data.optimumTemp;

		pThis->thermalModel.setTemperature(fTemp);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Tyre_stepGrainBlister(Tyre* pThis, float dt, float hubVelocity)
{
	Car* pCar = pThis->car;

	if (pCar && pThis->car->ksPhysics->tyreConsumptionRate > 0.0f)
	{
		if (pThis->status.load > 0.0f)
		{
			float fNdSlip = pThis->status.ndSlip;
			if (fNdSlip >= 2.5f)
				fNdSlip = 2.5f;

			float fCoreTemp = pThis->thermalModel.coreTemp;
			float fGrainGain = pThis->data.grainGain;
			float fSlipGamma = powf(fNdSlip, pThis->data.grainGamma);

			if (fGrainGain > 0.0f)
			{
				float fGrainThreshold = pThis->data.grainThreshold;
				if (fCoreTemp < fGrainThreshold)
				{
					auto pSurface = pThis->surfaceDef;
					if (pSurface)
					{
						if (hubVelocity > 2.0f)
						{
							float fGripMod = pSurface->gripMod;
							if (fGripMod >= 0.95f)
							{
								float fTest = ((hubVelocity * fGripMod) * fGrainGain) * ((fGrainThreshold - fCoreTemp) * 0.0001f);
								if (isfinite(fTest))
								{
									pThis->status.grain += fTest * fSlipGamma * pThis->car->ksPhysics->tyreConsumptionRate * dt;
								}
								else
								{
									log_printf(L"INF fGrainTest1");
								}
							}
						}
					}
				}
			}

			auto pSurface = pThis->surfaceDef;
			if (pSurface)
			{
				float fTest = (hubVelocity * pSurface->gripMod) * fGrainGain * 0.00005f;
				if (isfinite(fTest) && fTest > 0.0f)
				{
					pThis->status.grain -= fTest * fSlipGamma * pThis->car->ksPhysics->tyreConsumptionRate * dt;
				}
				else
				{
					log_printf(L"INF fGrainTest2");
				}
			}

			float fBlisterGain = pThis->data.blisterGain;
			if (fBlisterGain > 0.0f)
			{
				float fBlisterThreshold = pThis->data.blisterThreshold;
				if (fCoreTemp > fBlisterThreshold)
				{
					pSurface = pThis->surfaceDef;
					if (pSurface)
					{
						if (hubVelocity > 2.0f)
						{
							float fGripMod = pSurface->gripMod;
							if (fGripMod >= 0.95f)
							{
								float fBlisterGamma = pThis->data.blisterGamma;
								float fTest = ((pThis->totalHubVelocity * fGripMod) * fBlisterGain) * ((fCoreTemp - fBlisterThreshold) * 0.0001f);
								if (isfinite(fTest))
								{
									pThis->status.blister += fTest * powf(fNdSlip, fBlisterGamma) * pThis->car->ksPhysics->tyreConsumptionRate * dt;
								}
								else
								{
									log_printf(L"INF fBlisterTest");
								}
							}
						}
					}
				}
			}

			pThis->status.grain = tclamp<double>(pThis->status.grain, 0.0, 100.0);
			pThis->status.blister = tclamp<double>(pThis->status.blister, 0.0, 100.0);
		}
	}
	else
	{
		pThis->status.grain = 0.0;
		pThis->status.blister = 0.0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Tyre_stepFlatSpot(Tyre* pThis, float dt, float hubVelocity)
{
	if (fabsf(pThis->status.angularVelocity) <= 0.3f || pThis->status.slipRatio < -0.98f)
	{
		if (pThis->surfaceDef)
		{
			if (hubVelocity > 3.0f)
			{
				if (pThis->car)
				{
					float fDamage = pThis->car->ksPhysics->mechanicalDamageRate;
					if (fDamage != 0.0f)
					{
						float fGrip = pThis->surfaceDef->gripMod;
						if (fGrip >= 0.95f)
						{
							pThis->status.flatSpot = (((hubVelocity * pThis->flatSpotK) * pThis->status.load) * fGrip) * 0.00001f * dt * fDamage * pThis->data.softnessIndex + pThis->status.flatSpot;

							if (pThis->status.flatSpot > 1.0f)
								pThis->status.flatSpot = 1.0f;
						}
					}
				}
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
